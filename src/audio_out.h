#pragma once
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#include <mmsystem.h>
#include <atomic>
#include <cstdint>
#include <cstring>

// AudioOut
// ========
// Real-time 8 000 Hz / 16-bit mono audio output via WinMM waveOut.
//
// Thread model:
//   push()          — called from the scanner thread (producer)
//   WOM_DONE cb     — called from a WinMM internal thread (consumer)
//
// Uses a lock-free SPSC ring buffer to decouple the two.
// Buffers of silence are submitted during gaps to keep the waveOut
// pipeline continuously running and avoid underrun clicks.

class AudioOut {
public:
    AudioOut()  = default;
    ~AudioOut() { close(); }

    bool open(uint32_t sample_rate = 8000) noexcept;

    // Push PCM samples from the scanner thread.
    // Thread-safe; samples are silently dropped if the ring is full.
    void push(const int16_t* samples, size_t count) noexcept;

    void close() noexcept;

    bool is_open() const noexcept { return wout_ != nullptr; }

private:
    static void CALLBACK wave_cb(HWAVEOUT, UINT, DWORD_PTR, DWORD_PTR, DWORD_PTR);
    void on_buffer_done(WAVEHDR* hdr) noexcept;

    HWAVEOUT wout_ = nullptr;
    std::atomic<bool> running_{false};

    static constexpr int    NUM_BUFS    = 4;
    static constexpr size_t BUF_SAMPLES = 800;  // 100 ms at 8 kHz

    WAVEHDR  hdrs_[NUM_BUFS]            = {};
    int16_t  bufs_[NUM_BUFS][BUF_SAMPLES] = {};

    // ── Lock-free SPSC ring buffer ────────────────────────────────────────
    static constexpr size_t RING_CAP = 32000; // ~4 s at 8 kHz
    int16_t              ring_[RING_CAP] = {};
    std::atomic<size_t>  ring_w_{0};   // write index (producer)
    std::atomic<size_t>  ring_r_{0};   // read  index (consumer)
};
