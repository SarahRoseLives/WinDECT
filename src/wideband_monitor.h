#pragma once
#include "dect_channels.h"
#include "packet_decoder.h"
#include "packet_receiver.h"
#include "phase_diff.h"
#include "audio_out.h"
#include "dc_blocker.h"

#include <array>
#include <atomic>
#include <complex>
#include <cstdint>
#include <memory>
#include <mutex>

namespace windect {

// WidebandMonitor
// ===============
// Captures the entire US DECT 6.0 band in one 18.432 Msps HackRF stream.
// For each of the 10 channels, the stream is mixed (frequency-shifted) and
// decimated 4:1 → 4.608 Msps, then fed into a full PacketReceiver+PacketDecoder
// pipeline.  All channels run simultaneously — no channel hopping required.
//
// Audio routing: auto-locks onto the first channel that delivers decoded voice
// PCM.  Call reset_audio_lock() to release and follow the next call.

class WidebandMonitor {
public:
    // 4× the narrowband rate → covers the whole DECT band in one capture
    static constexpr uint32_t WIDEBAND_SAMPLE_RATE = 18'432'000;

    // Center of the 10-channel US DECT 6.0 band
    // = (ch0 + ch9) / 2 = (1921.536 + 1937.088) / 2 MHz
    static constexpr uint64_t WIDEBAND_CENTER_HZ = 1'929'312'000ULL;

    static constexpr size_t NUM_CHANNELS = US_DECT_CHANNELS.size(); // 10

    explicit WidebandMonitor(AudioOut* audio) noexcept;

    // Feed raw IQ samples from the HackRF callback thread.
    void ingest(const std::complex<float>* samples, size_t n) noexcept;

    // Returns the channel index [0, NUM_CHANNELS) currently locked for audio,
    // or -1 if no voice has arrived yet (auto mode).
    int locked_channel() const noexcept;

    // Monotonic millisecond timestamp of the last voice PCM packet on this
    // channel index.  Returns 0 if no voice has been heard on it.
    int64_t last_voice_ms(size_t channel_index) const noexcept;

    // Release the current audio lock.  The next channel that delivers voice
    // becomes the new lock target automatically.
    void reset_audio_lock() noexcept;

    // Per-channel status snapshot
    struct ChannelStatus {
        bool     voice_detected  = false;
        int      active_parts    = 0;
        uint64_t packets_seen    = 0;
        uint64_t voice_frames_ok = 0;
        uint64_t voice_xcrc_fail = 0;
    };
    ChannelStatus channel_status(size_t ch) const noexcept;

    // Redraw the per-channel activity table in-place (ANSI VT100).
    // locked_ch: index of the locked channel (-1 = scanning).
    // call_dur_sec: seconds elapsed since lock (shown in the status line).
    void render_frame(int locked_ch, int64_t call_dur_sec) const noexcept;

    // Monotonic clock in milliseconds — shared between monitor and main loop.
    static int64_t mono_ms() noexcept;

private:
    void process_sample(std::complex<float> s) noexcept;
    void on_channel_update(size_t ch, const PartInfo parts[], int count) noexcept;
    void on_voice_packet(size_t ch, const int16_t* pcm, size_t count) noexcept;

    AudioOut* audio_;

    // Per-channel DSP state (accessed only from ingest() / HackRF thread)
    std::array<PhaseDiff,             NUM_CHANNELS> phase_diff_;
    std::array<std::complex<float>,   NUM_CHANNELS> mixer_;       // oscillator state
    std::array<std::complex<float>,   NUM_CHANNELS> mixer_step_;  // per-sample rotation
    std::array<std::complex<float>,   NUM_CHANNELS> decim_accum_; // decimation accumulator
    std::array<int,                   NUM_CHANNELS> decim_phase_; // counts 0–3

    std::array<std::unique_ptr<PacketReceiver>, NUM_CHANNELS> receivers_;
    std::array<std::unique_ptr<PacketDecoder>,  NUM_CHANNELS> decoders_;

    // Packet counter — written by HackRF thread, read by render thread
    std::array<std::atomic<uint64_t>, NUM_CHANNELS> packets_seen_;

    // Decoder status — written by HackRF thread (PacketDecoder callback)
    mutable std::mutex status_mutex_;
    std::array<ChannelStatus, NUM_CHANNELS> status_;

    // Audio routing
    mutable std::mutex audio_mutex_;
    int audio_channel_ = -1; // -1 = auto (first voice channel becomes the lock)

    // Last voice timestamp per channel — written by HackRF thread
    std::array<std::atomic<int64_t>, NUM_CHANNELS> last_voice_ms_;

    DCBlocker dc_blocker_;
    uint64_t  sample_counter_ = 0;
};

} // namespace windect
