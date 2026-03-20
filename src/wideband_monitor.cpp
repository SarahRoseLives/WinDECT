#include "wideband_monitor.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <chrono>

namespace windect {

static constexpr float PI_F = 3.14159265358979323846f;

// ── Constructor ───────────────────────────────────────────────────────────────

WidebandMonitor::WidebandMonitor(AudioOut* audio) noexcept
    : audio_(audio)
{
    // Atomics are not copyable — initialise by loop
    for (auto& c  : packets_seen_)  c.store(0, std::memory_order_relaxed);
    for (auto& ms : last_voice_ms_) ms.store(0, std::memory_order_relaxed);

    for (size_t i = 0; i < NUM_CHANNELS; ++i) {
        // Mixer step: rotate IQ to shift channel i to DC, then decimate 4:1
        const float offset_hz =
            static_cast<float>(static_cast<int64_t>(US_DECT_CHANNELS[i].freq_hz) -
                               static_cast<int64_t>(WIDEBAND_CENTER_HZ));
        const float angle = -2.0f * PI_F * offset_hz
                          / static_cast<float>(WIDEBAND_SAMPLE_RATE);

        mixer_[i]      = {1.0f, 0.0f};
        mixer_step_[i] = {std::cos(angle), std::sin(angle)};
        decim_accum_[i] = {};
        decim_phase_[i] = 0;

        decoders_[i] = std::make_unique<PacketDecoder>(
            [this, i](const PartInfo parts[], int count) {
                on_channel_update(i, parts, count);
            },
            [this, i](int /*rx_id*/, const int16_t* pcm, size_t count) {
                on_voice_packet(i, pcm, count);
            });

        receivers_[i] = std::make_unique<PacketReceiver>(
            [this, i](const ReceivedPacket& pkt) {
                packets_seen_[i].fetch_add(1, std::memory_order_relaxed);
                decoders_[i]->process_packet(pkt);
            },
            [this, i](int rx_id) {
                decoders_[i]->notify_lost(rx_id);
            });
    }
}

// ── Timing helper ─────────────────────────────────────────────────────────────

int64_t WidebandMonitor::mono_ms() noexcept {
    using namespace std::chrono;
    return duration_cast<milliseconds>(
        steady_clock::now().time_since_epoch()).count();
}

// ── IQ ingestion ──────────────────────────────────────────────────────────────

void WidebandMonitor::ingest(const std::complex<float>* samples, size_t n) noexcept {
    for (size_t i = 0; i < n; ++i)
        process_sample(dc_blocker_.process(samples[i]));
}

// Mix each wideband sample with a per-channel complex oscillator, accumulate
// 4 samples, then deliver the decimated sample to the channel's PhaseDiff and
// PacketReceiver.  This produces a 4.608 Msps (= SAMPLE_RATE) narrowband
// stream per channel, exactly as required by the existing decoder pipeline.

void WidebandMonitor::process_sample(std::complex<float> s) noexcept {
    for (size_t i = 0; i < NUM_CHANNELS; ++i) {
        decim_accum_[i] += s * mixer_[i];
        mixer_[i] *= mixer_step_[i];

        if (++decim_phase_[i] == 4) {
            decim_phase_[i] = 0;
            const float phase = phase_diff_[i].process(decim_accum_[i] * 0.25f);
            decim_accum_[i]  = {};
            receivers_[i]->process_sample(phase);
        }
    }

    // Renormalise oscillators periodically to prevent magnitude creep
    if ((++sample_counter_ & 0x0FFFu) == 0) {
        for (auto& osc : mixer_) {
            const float mag = std::abs(osc);
            if (mag > 0.0f) osc /= mag;
        }
    }
}

// ── PacketDecoder callbacks ───────────────────────────────────────────────────

void WidebandMonitor::on_channel_update(size_t ch,
                                        const PartInfo parts[],
                                        int count) noexcept {
    ChannelStatus s;
    s.active_parts = count;
    for (int i = 0; i < count; ++i) {
        if (parts[i].voice_present) s.voice_detected  = true;
        s.voice_frames_ok += parts[i].voice_frames_ok;
        s.voice_xcrc_fail += parts[i].voice_xcrc_fail;
    }
    s.packets_seen = packets_seen_[ch].load(std::memory_order_relaxed);

    std::lock_guard<std::mutex> lock(status_mutex_);
    status_[ch] = s;
}

void WidebandMonitor::on_voice_packet(size_t ch,
                                      const int16_t* pcm,
                                      size_t count) noexcept {
    last_voice_ms_[ch].store(mono_ms(), std::memory_order_relaxed);

    std::lock_guard<std::mutex> lock(audio_mutex_);
    if (!audio_) return;

    if (audio_channel_ < 0) {
        // Auto mode: lock onto the first channel that delivers voice
        audio_channel_ = static_cast<int>(ch);
    }
    if (static_cast<int>(ch) == audio_channel_)
        audio_->push(pcm, count);
}

// ── Public interface ──────────────────────────────────────────────────────────

int WidebandMonitor::locked_channel() const noexcept {
    std::lock_guard<std::mutex> lock(audio_mutex_);
    return audio_channel_;
}

int64_t WidebandMonitor::last_voice_ms(size_t ch) const noexcept {
    if (ch >= NUM_CHANNELS) return 0;
    return last_voice_ms_[ch].load(std::memory_order_relaxed);
}

void WidebandMonitor::reset_audio_lock() noexcept {
    std::lock_guard<std::mutex> lock(audio_mutex_);
    audio_channel_ = -1;
}

WidebandMonitor::ChannelStatus
WidebandMonitor::channel_status(size_t ch) const noexcept {
    if (ch >= NUM_CHANNELS) return {};
    std::lock_guard<std::mutex> lock(status_mutex_);
    ChannelStatus s = status_[ch];
    s.packets_seen = packets_seen_[ch].load(std::memory_order_relaxed);
    return s;
}

// ── Console rendering ─────────────────────────────────────────────────────────

void WidebandMonitor::render_frame(int locked_ch, int64_t call_dur_sec) const noexcept {
    // ANSI: move cursor to top-left, erase screen
    std::printf("\x1b[H\x1b[J");
    std::printf("WinDECT  US DECT 6.0 Wideband Scanner + Voice Decoder\n");
    std::printf("  Center: %.3f MHz   Rate: %.3f Msps   Channels: %zu\n\n",
                WIDEBAND_CENTER_HZ / 1e6,
                WIDEBAND_SAMPLE_RATE / 1e6,
                NUM_CHANNELS);

    for (size_t i = 0; i < NUM_CHANNELS; ++i) {
        const ChannelStatus  s  = channel_status(i);
        const DectChannel&   ch = US_DECT_CHANNELS[i];

        // X-CRC quality indicator
        char xcrc_buf[20] = "";
        uint64_t xtotal = s.voice_frames_ok + s.voice_xcrc_fail;
        if (xtotal > 0) {
            int pct = static_cast<int>(100 * s.voice_frames_ok / xtotal);
            std::snprintf(xcrc_buf, sizeof(xcrc_buf), "  XCRC:%3d%%", pct);
        }

        // Lock indicator
        const char* tag = (locked_ch >= 0 && static_cast<size_t>(locked_ch) == i)
                        ? "  <<< LOCKED"
                        : "";

        std::printf("  Ch%-2d  %.3f MHz  parts:%-2d  voice:%-3s"
                    "  pkts:%6llu%s%s\n",
                    ch.number,
                    ch.freq_hz / 1e6,
                    s.active_parts,
                    s.voice_detected ? "YES" : "no",
                    static_cast<unsigned long long>(s.packets_seen),
                    xcrc_buf,
                    tag);
    }

    std::printf("\n");
    if (locked_ch < 0) {
        std::printf("  Scanning all channels...  Ctrl-C to stop.\n");
    } else {
        std::printf(
            "  *** LOCKED on ch%d (%.3f MHz)  —  call duration %lld:%02lld"
            "  —  Ctrl-C to stop.\n",
            US_DECT_CHANNELS[locked_ch].number,
            US_DECT_CHANNELS[locked_ch].freq_hz / 1e6,
            (long long)call_dur_sec / 60,
            (long long)call_dur_sec % 60);
    }

    std::fflush(stdout);
}

} // namespace windect
