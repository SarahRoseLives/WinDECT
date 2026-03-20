#include "audio_out.h"
#include <algorithm>
#include <cstdio>

// ── WinMM callback (runs on a WinMM internal thread) ─────────────────────────

void CALLBACK AudioOut::wave_cb(HWAVEOUT /*hwo*/, UINT msg,
                                 DWORD_PTR instance,
                                 DWORD_PTR param1, DWORD_PTR /*param2*/)
{
    if (msg == WOM_DONE) {
        auto* self = reinterpret_cast<AudioOut*>(instance);
        auto* hdr  = reinterpret_cast<WAVEHDR*>(param1);
        self->on_buffer_done(hdr);
    }
}

void AudioOut::on_buffer_done(WAVEHDR* hdr) noexcept {
    if (!running_.load(std::memory_order_acquire)) return;

    int16_t* buf = reinterpret_cast<int16_t*>(hdr->lpData);

    // Drain ring buffer
    size_t w = ring_w_.load(std::memory_order_acquire);
    size_t r = ring_r_.load(std::memory_order_relaxed);
    size_t avail = (w >= r) ? (w - r) : (RING_CAP - r + w);
    size_t n = std::min(avail, BUF_SAMPLES);

    for (size_t i = 0; i < n; ++i)
        buf[i] = ring_[(r + i) % RING_CAP];
    if (n < BUF_SAMPLES)
        memset(&buf[n], 0, (BUF_SAMPLES - n) * sizeof(int16_t));

    ring_r_.store((r + n) % RING_CAP, std::memory_order_release);

    hdr->dwBufferLength = static_cast<DWORD>(BUF_SAMPLES * sizeof(int16_t));
    waveOutWrite(wout_, hdr, sizeof(WAVEHDR));
}

// ── Public interface ──────────────────────────────────────────────────────────

bool AudioOut::open(uint32_t sample_rate) noexcept {
    WAVEFORMATEX fmt  = {};
    fmt.wFormatTag      = WAVE_FORMAT_PCM;
    fmt.nChannels       = 1;
    fmt.nSamplesPerSec  = sample_rate;
    fmt.wBitsPerSample  = 16;
    fmt.nBlockAlign     = 2;
    fmt.nAvgBytesPerSec = sample_rate * 2;

    MMRESULT r = waveOutOpen(&wout_, WAVE_MAPPER, &fmt,
                             reinterpret_cast<DWORD_PTR>(wave_cb),
                             reinterpret_cast<DWORD_PTR>(this),
                             CALLBACK_FUNCTION);
    if (r != MMSYSERR_NOERROR) {
        wout_ = nullptr;
        return false;
    }

    running_.store(true, std::memory_order_release);

    // Pre-fill all buffers with silence and submit them
    for (int i = 0; i < NUM_BUFS; ++i) {
        memset(bufs_[i], 0, sizeof(bufs_[i]));
        hdrs_[i]               = {};
        hdrs_[i].lpData        = reinterpret_cast<LPSTR>(bufs_[i]);
        hdrs_[i].dwBufferLength= sizeof(bufs_[i]);
        hdrs_[i].dwUser        = static_cast<DWORD_PTR>(i);
        waveOutPrepareHeader(wout_, &hdrs_[i], sizeof(WAVEHDR));
        waveOutWrite(wout_, &hdrs_[i], sizeof(WAVEHDR));
    }
    return true;
}

void AudioOut::push(const int16_t* samples, size_t count) noexcept {
    size_t w = ring_w_.load(std::memory_order_relaxed);
    for (size_t i = 0; i < count; ++i) {
        size_t next = (w + 1) % RING_CAP;
        if (next == ring_r_.load(std::memory_order_acquire)) break; // full: drop
        ring_[w] = samples[i];
        w = next;
    }
    ring_w_.store(w, std::memory_order_release);
}

void AudioOut::close() noexcept {
    if (!wout_) return;
    running_.store(false, std::memory_order_release);
    waveOutReset(wout_);       // returns all pending buffers (fires WOM_DONE once each)
    Sleep(200);                 // wait for callbacks to finish
    for (int i = 0; i < NUM_BUFS; ++i)
        waveOutUnprepareHeader(wout_, &hdrs_[i], sizeof(WAVEHDR));
    waveOutClose(wout_);
    wout_ = nullptr;
}
