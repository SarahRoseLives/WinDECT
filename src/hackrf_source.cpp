#include "hackrf_source.h"
#include <hackrf.h>
#include <cstdio>
#include <cstring>
#include <algorithm>
#include <chrono>
#include <thread>

namespace windect {

// Trampoline: libhackrf calls this C-style; we recover the HackrfSource pointer
// from transfer->rx_ctx which we set in start().
struct HackrfTransfer {
    // Mirrors the layout of hackrf_transfer for the fields we need.
    // We just use the real hackrf_transfer* via void* cast.
};

// ── static helpers ──────────────────────────────────────────────────────────

// Convert HackRF raw signed 8-bit IQ pairs to complex<float> normalized to [-1,+1]
static void convert_s8_to_cf32(const int8_t* in, std::complex<float>* out, size_t n_samples) {
    constexpr float scale = 1.0f / 128.0f;
    for (size_t i = 0; i < n_samples; ++i) {
        out[i] = std::complex<float>(in[2*i] * scale, in[2*i+1] * scale);
    }
}

// ── HackrfSource ────────────────────────────────────────────────────────────

HackrfSource::HackrfSource()
    : device_(nullptr), streaming_(false), last_error_("") {}

HackrfSource::~HackrfSource() {
    stop();
    close();
}

bool HackrfSource::open(unsigned device_index) {
    int r = hackrf_init();
    if (r != HACKRF_SUCCESS) {
        last_error_ = hackrf_error_name(static_cast<hackrf_error>(r));
        return false;
    }

    hackrf_device_list_t* list = hackrf_device_list();
    if (!list || list->devicecount == 0) {
        last_error_ = "No HackRF devices found";
        hackrf_exit();
        return false;
    }

    if (device_index >= (unsigned)list->devicecount) {
        last_error_ = "Device index out of range";
        hackrf_device_list_free(list);
        hackrf_exit();
        return false;
    }

    hackrf_device* dev = nullptr;
    r = hackrf_device_list_open(list, device_index, &dev);
    hackrf_device_list_free(list);

    if (r != HACKRF_SUCCESS) {
        last_error_ = hackrf_error_name(static_cast<hackrf_error>(r));
        hackrf_exit();
        return false;
    }

    device_ = dev;
    last_error_ = "";
    return true;
}

void HackrfSource::close() {
    if (device_) {
        hackrf_close(static_cast<hackrf_device*>(device_));
        device_ = nullptr;
        hackrf_exit();
    }
}

bool HackrfSource::set_freq(uint64_t freq_hz) {
    if (!device_) { last_error_ = "Device not open"; return false; }
    int r = hackrf_set_freq(static_cast<hackrf_device*>(device_), freq_hz);
    if (r != HACKRF_SUCCESS) {
        last_error_ = hackrf_error_name(static_cast<hackrf_error>(r));
        return false;
    }
    return true;
}

bool HackrfSource::set_sample_rate(uint32_t sample_rate) {
    if (!device_) { last_error_ = "Device not open"; return false; }
    int r = hackrf_set_sample_rate(static_cast<hackrf_device*>(device_),
                                   static_cast<double>(sample_rate));
    if (r != HACKRF_SUCCESS) {
        last_error_ = hackrf_error_name(static_cast<hackrf_error>(r));
        return false;
    }
    // Baseband filter: set to ~75% of sample rate so aliasing is avoided
    // while maximising useful bandwidth.  hackrf_compute_baseband_filter_bw()
    // rounds down to the nearest valid HackRF bandwidth value.
    uint32_t bw = hackrf_compute_baseband_filter_bw(
        static_cast<uint32_t>(sample_rate * 3 / 4));
    hackrf_set_baseband_filter_bandwidth(
        static_cast<hackrf_device*>(device_), bw);
    return true;
}

bool HackrfSource::set_lna_gain(uint32_t gain_db) {
    if (!device_) { last_error_ = "Device not open"; return false; }
    // LNA gain: 0, 8, 16, 24, 32, 40 dB
    uint32_t clamped = std::min(gain_db, 40u);
    clamped = (clamped / 8) * 8;
    int r = hackrf_set_lna_gain(static_cast<hackrf_device*>(device_), clamped);
    if (r != HACKRF_SUCCESS) {
        last_error_ = hackrf_error_name(static_cast<hackrf_error>(r));
        return false;
    }
    return true;
}

bool HackrfSource::set_vga_gain(uint32_t gain_db) {
    if (!device_) { last_error_ = "Device not open"; return false; }
    // VGA gain: 0–62 dB in steps of 2
    uint32_t clamped = std::min(gain_db, 62u);
    clamped = (clamped / 2) * 2;
    int r = hackrf_set_vga_gain(static_cast<hackrf_device*>(device_), clamped);
    if (r != HACKRF_SUCCESS) {
        last_error_ = hackrf_error_name(static_cast<hackrf_error>(r));
        return false;
    }
    return true;
}

bool HackrfSource::set_amp_enable(bool enable) {
    if (!device_) { last_error_ = "Device not open"; return false; }
    int r = hackrf_set_amp_enable(static_cast<hackrf_device*>(device_), enable ? 1 : 0);
    if (r != HACKRF_SUCCESS) {
        last_error_ = hackrf_error_name(static_cast<hackrf_error>(r));
        return false;
    }
    return true;
}

int HackrfSource::rx_callback(hackrf_transfer* transfer) {
    HackrfSource* self = static_cast<HackrfSource*>(transfer->rx_ctx);

    // Fast path: already stopped — no work needed
    if (!self->streaming_.load(std::memory_order_acquire)) return 0;

    // Hold the run-mutex for the duration of the user callback.
    // stop() acquires this after hackrf_stop_rx() to wait for us to finish.
    std::lock_guard<std::mutex> lock(self->callback_run_mutex_);

    // Re-check inside the lock (stop() may have set streaming_=false while
    // we were waiting for the lock)
    if (!self->streaming_.load(std::memory_order_acquire)) return 0;

    size_t n_samples = transfer->valid_length / 2; // 2 bytes (I+Q) per sample

    if (self->conv_buf_.size() < n_samples)
        self->conv_buf_.resize(n_samples);

    convert_s8_to_cf32(reinterpret_cast<const int8_t*>(transfer->buffer),
                       self->conv_buf_.data(), n_samples);

    self->callback_(self->conv_buf_.data(), n_samples);
    return 0; // returning non-zero stops streaming
}

bool HackrfSource::start(IQCallback cb) {
    if (!device_) { last_error_ = "Device not open"; return false; }
    if (streaming_) return true;

    callback_  = std::move(cb);
    streaming_ = true;

    int r = hackrf_start_rx(static_cast<hackrf_device*>(device_), rx_callback, this);
    if (r != HACKRF_SUCCESS) {
        last_error_ = hackrf_error_name(static_cast<hackrf_error>(r));
        streaming_  = false;
        return false;
    }
    return true;
}

bool HackrfSource::stop() {
    if (!streaming_) return true;
    streaming_.store(false, std::memory_order_release);
    if (device_) {
        hackrf_stop_rx(static_cast<hackrf_device*>(device_));
    }
    // Wait for any callback that was already past the streaming_ check to
    // finish executing.  Once we acquire this mutex, we know the callback
    // thread has exited the user callback — safe for the caller to destroy
    // whatever the callback was pointing at.
    std::lock_guard<std::mutex> lock(callback_run_mutex_);
    return true;
}

bool HackrfSource::wait_idle(int timeout_ms) noexcept {
    if (!device_) return true;
    using namespace std::chrono;
    const auto deadline = steady_clock::now() + milliseconds(timeout_ms);
    while (steady_clock::now() < deadline) {
        // hackrf_is_streaming returns HACKRF_TRUE (1) while the USB transfer
        // thread is alive; anything else means it has stopped.
        if (hackrf_is_streaming(static_cast<hackrf_device*>(device_)) != HACKRF_TRUE)
            return true;
        std::this_thread::sleep_for(milliseconds(10));
    }
    last_error_ = "Timed out waiting for device to become idle";
    return false;
}

bool HackrfSource::is_streaming() const {
    return streaming_;
}

} // namespace windect
