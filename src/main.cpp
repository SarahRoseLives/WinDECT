/*
 * WinDECT — US DECT 6.0 Wideband Scanner + Live Voice Decoder
 *
 * SCANNING:  HackRF at 18.432 Msps centred at 1929.312 MHz.  All 10 US DECT
 *            channels are decoded simultaneously via per-channel mix-and-
 *            decimate inside WidebandMonitor.  A live activity table is shown.
 *
 * LOCKED:    When the first voice packet arrives the HackRF is stopped and
 *            retuned to that channel at 4.608 Msps (hardware narrowband).
 *            A simple PhaseDiff → PacketReceiver → PacketDecoder pipeline
 *            decodes G.721 ADPCM and plays PCM through WinMM.
 *            After 2 s of silence the call is considered ended and the HackRF
 *            is retuned back to wideband for the next scan pass.
 *
 * Build:  see CMakeLists.txt
 * Usage:  windect [options]
 *   -g <lna>   LNA gain 0-40 dB (default: 32)
 *   -v <vga>   VGA gain 0-62 dB (default: 20)
 *   -a         Enable HackRF RF amplifier (+14 dB, use carefully)
 *   -q         Quiet: only print lock/unlock events, no live table
 */

#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>

#include "hackrf_source.h"
#include "wideband_monitor.h"
#include "dect_channels.h"
#include "phase_diff.h"
#include "packet_receiver.h"
#include "packet_decoder.h"
#include "audio_out.h"

#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <csignal>
#include <memory>
#include <thread>

using namespace windect;
using Clock = std::chrono::steady_clock;

static std::atomic<bool> g_running{true};
static void signal_handler(int) { g_running = false; }

// ── Narrowband pipeline ───────────────────────────────────────────────────────
// Used while LOCKED on a single channel.
// IQ samples → PhaseDiff → PacketReceiver → PacketDecoder → AudioOut

struct NarrowCtx {
    PhaseDiff      phase_diff;
    PacketReceiver receiver;
    PacketDecoder  decoder;
    std::atomic<int64_t> last_voice_ms{0};

    // Voice mixing: RFP and PP each emit 80 samples per 10 ms DECT frame.
    // Without mixing both streams are pushed independently, filling the ring
    // at 2× the drain rate and causing choppy audio.  We hold the first
    // arrival and mix the second in before flushing — same as DeDECTive.
    int16_t mix_frame[80]  = {};
    int     mix_first_rx_id = -1;   // -1 = empty

    explicit NarrowCtx(AudioOut* audio)
        : receiver(
            [this](const ReceivedPacket& pkt) {
                decoder.process_packet(pkt);
            },
            [this](int rx_id) {
                decoder.notify_lost(rx_id);
            })
        , decoder(
            nullptr, // no part-update UI needed in locked mode
            [this, audio](int rx_id, const int16_t* pcm, size_t n) {
                last_voice_ms.store(
                    std::chrono::duration_cast<std::chrono::milliseconds>(
                        Clock::now().time_since_epoch()).count(),
                    std::memory_order_relaxed);
                if (!audio || n != 80) {
                    if (audio) audio->push(pcm, n);
                    return;
                }
                if (mix_first_rx_id < 0) {
                    // First contributor — hold it
                    std::memcpy(mix_frame, pcm, 80 * sizeof(int16_t));
                    mix_first_rx_id = rx_id;
                } else if (rx_id != mix_first_rx_id) {
                    // Second contributor — mix and flush
                    for (int i = 0; i < 80; ++i) {
                        int32_t m = static_cast<int32_t>(mix_frame[i])
                                  + static_cast<int32_t>(pcm[i]);
                        mix_frame[i] = static_cast<int16_t>(
                            std::min(std::max(m, (int32_t)-32768), (int32_t)32767));
                    }
                    audio->push(mix_frame, 80);
                    mix_first_rx_id = -1;
                } else {
                    // Same part again (other side silent) — flush old, hold new
                    audio->push(mix_frame, 80);
                    std::memcpy(mix_frame, pcm, 80 * sizeof(int16_t));
                }
            })
    {}

    void ingest(const std::complex<float>* s, size_t n) noexcept {
        for (size_t i = 0; i < n; ++i)
            receiver.process_sample(phase_diff.process(s[i]));
    }
};

// ── Helpers ───────────────────────────────────────────────────────────────────

static int64_t mono_ms() noexcept {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
        Clock::now().time_since_epoch()).count();
}

static void print_usage(const char* prog) {
    std::printf(
        "Usage: %s [options]\n"
        "\nOptions:\n"
        "  -g <lna>   LNA gain  0-40 dB, steps of 8  (default: 32)\n"
        "  -v <vga>   VGA gain  0-62 dB, steps of 2  (default: 20)\n"
        "  -a         Enable HackRF RF amplifier (+14 dB)\n"
        "  -q         Quiet: suppress live channel table, show events only\n"
        "  -h         Show this help\n"
        "\nAll %zu US DECT 6.0 channels monitored simultaneously.\n"
        "  Wideband centre: %.3f MHz  @ %.3f Msps\n"
        "  Narrowband (per-channel):     %.3f Msps\n",
        prog,
        WidebandMonitor::NUM_CHANNELS,
        WidebandMonitor::WIDEBAND_CENTER_HZ / 1e6,
        WidebandMonitor::WIDEBAND_SAMPLE_RATE / 1e6,
        SAMPLE_RATE / 1e6);
}

// ── main ──────────────────────────────────────────────────────────────────────

int main(int argc, char* argv[]) {
    // Enable ANSI VT100 escape sequences on Windows 10+
    {
        HANDLE h = GetStdHandle(STD_OUTPUT_HANDLE);
        DWORD  m = 0;
        if (GetConsoleMode(h, &m))
            SetConsoleMode(h, m | ENABLE_VIRTUAL_TERMINAL_PROCESSING);
    }

    uint32_t lna_gain   = 32;
    uint32_t vga_gain   = 20;
    bool     amp_enable = false;
    bool     quiet      = false;

    for (int i = 1; i < argc; ++i) {
        if      (strcmp(argv[i], "-g") == 0 && i+1 < argc) lna_gain   = (uint32_t)atoi(argv[++i]);
        else if (strcmp(argv[i], "-v") == 0 && i+1 < argc) vga_gain   = (uint32_t)atoi(argv[++i]);
        else if (strcmp(argv[i], "-a") == 0)               amp_enable = true;
        else if (strcmp(argv[i], "-q") == 0)               quiet      = true;
        else if (strcmp(argv[i], "-h") == 0) { print_usage(argv[0]); return 0; }
        else {
            std::fprintf(stderr, "Unknown option: %s\n", argv[i]);
            print_usage(argv[0]);
            return 1;
        }
    }

    signal(SIGINT,  signal_handler);
    signal(SIGTERM, signal_handler);

    std::printf("WinDECT v0.1 — US DECT 6.0 Wideband Scanner + Voice Decoder\n");
    std::printf("  LNA: %u dB  VGA: %u dB  Amp: %s\n",
                lna_gain, vga_gain, amp_enable ? "ON" : "off");
    std::printf("  Wideband: %.3f MHz @ %.3f Msps  |  Narrowband: %.3f Msps per channel\n\n",
                WidebandMonitor::WIDEBAND_CENTER_HZ / 1e6,
                WidebandMonitor::WIDEBAND_SAMPLE_RATE / 1e6,
                SAMPLE_RATE / 1e6);

    // Audio stays open for the entire session
    AudioOut audio;
    if (audio.open(8000))
        std::printf("Audio: 8 kHz / 16-bit mono / WinMM\n\n");
    else
        std::printf("Audio: UNAVAILABLE — voice decoded but not played\n\n");

    HackrfSource hackrf;

    // Apply gain settings (called whenever we retune)
    auto apply_gains = [&]() -> bool {
        return hackrf.set_lna_gain(lna_gain)
            && hackrf.set_vga_gain(vga_gain)
            && hackrf.set_amp_enable(amp_enable);
    };

    // Close and reopen the HackRF to get a fresh USB state.
    // DeDECTive uses the same pattern — it's the only reliable way to
    // reconfigure the device without getting LIBUSB_ERROR_BUSY.
    auto reopen_hackrf = [&]() -> bool {
        hackrf.stop();
        hackrf.close();
        if (!hackrf.open()) {
            std::fprintf(stderr, "HackRF reopen failed: %s\n", hackrf.last_error());
            return false;
        }
        return true;
    };

    std::printf("Opening HackRF... ");
    std::fflush(stdout);
    if (!hackrf.open()) {
        std::fprintf(stderr, "FAILED: %s\n", hackrf.last_error());
        return 1;
    }
    std::printf("OK\n\n");

    // ── State machine ─────────────────────────────────────────────────────────
    //
    //  SCANNING  WidebandMonitor decodes all 10 channels at 18.432 Msps.
    //            render_frame() shows a live activity table.
    //            → LOCKED the instant any channel reports a voice packet.
    //
    //  LOCKED    HackRF retuned to the voice channel at 4.608 Msps.
    //            NarrowCtx feeds IQ directly into the narrowband decoder.
    //            PCM is played through WinMM AudioOut.
    //            → SCANNING after VOICE_END_MS ms of silence.

    static constexpr int64_t VOICE_END_MS = 2000; // 2 s silence = call ended

    enum class State { SCANNING, LOCKED };
    State   state     = State::SCANNING;
    int     locked_ch = -1;
    int64_t lock_start = 0;

    // ── Start in wideband / SCANNING mode ────────────────────────────────────
    auto monitor = std::make_unique<WidebandMonitor>(&audio);

    if (!hackrf.set_sample_rate(WidebandMonitor::WIDEBAND_SAMPLE_RATE) ||
        !apply_gains() ||
        !hackrf.set_freq(WidebandMonitor::WIDEBAND_CENTER_HZ)) {
        std::fprintf(stderr, "HackRF wideband setup failed: %s\n", hackrf.last_error());
        return 1;
    }
    if (!hackrf.start([&monitor](const std::complex<float>* s, size_t n) {
            monitor->ingest(s, n);
        })) {
        std::fprintf(stderr, "HackRF start failed: %s\n", hackrf.last_error());
        return 1;
    }
    std::printf("Scanning all %zu channels...  Ctrl-C to stop.\n\n",
                WidebandMonitor::NUM_CHANNELS);

    std::unique_ptr<NarrowCtx> narrow;

    while (g_running) {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));

        // ── SCANNING ─────────────────────────────────────────────────────────
        if (state == State::SCANNING) {
            const int vc = monitor->locked_channel();
            if (vc < 0) {
                if (!quiet)
                    monitor->render_frame(-1, 0);
                continue;
            }

            // Voice detected — switch to narrowband on that channel
            locked_ch = vc;
            const DectChannel& ch = US_DECT_CHANNELS[locked_ch];

            std::printf("\x1b[H\x1b[J"); // clear screen
            std::printf("*** VOICE on ch%d  %.3f MHz — switching to narrowband...\n",
                        ch.number, ch.freq_hz / 1e6);
            std::fflush(stdout);

            // Stop streaming FIRST (inside reopen), then destroy monitor —
            // the callback captures monitor by reference and must not fire
            // after monitor is destroyed.
            if (!reopen_hackrf()) break;
            monitor.reset();

            bool ok = hackrf.set_sample_rate(SAMPLE_RATE)
                   && apply_gains()
                   && hackrf.set_freq(ch.freq_hz);

            if (ok) {
                narrow = std::make_unique<NarrowCtx>(&audio);
                // Seed last_voice_ms so we get a grace period before the
                // silence timer starts (decoder needs a few frames to sync).
                narrow->last_voice_ms.store(mono_ms(), std::memory_order_relaxed);

                ok = hackrf.start([&narrow](const std::complex<float>* s, size_t n) {
                    narrow->ingest(s, n);
                });
            }

            if (ok) {
                lock_start = mono_ms();
                state      = State::LOCKED;
                std::printf("LOCKED — following ch%d  %.3f MHz"
                            "  (Ctrl-C to stop)\n\n",
                            ch.number, ch.freq_hz / 1e6);
                std::fflush(stdout);
            } else {
                // Retune failed — go straight back to wideband
                std::fprintf(stderr, "Narrowband retune failed: %s — resuming scan\n",
                             hackrf.last_error());
                narrow.reset();
                locked_ch = -1;

                monitor = std::make_unique<WidebandMonitor>(&audio);
                if (!reopen_hackrf() ||
                    !hackrf.set_sample_rate(WidebandMonitor::WIDEBAND_SAMPLE_RATE) ||
                    !apply_gains() ||
                    !hackrf.set_freq(WidebandMonitor::WIDEBAND_CENTER_HZ) ||
                    !hackrf.start([&monitor](const std::complex<float>* s, size_t n) {
                        monitor->ingest(s, n);
                    })) {
                    std::fprintf(stderr, "Wideband restart failed: %s\n",
                                 hackrf.last_error());
                    break;
                }
                // state remains SCANNING
            }

        // ── LOCKED ───────────────────────────────────────────────────────────
        } else {
            const int64_t now       = mono_ms();
            const int64_t silent_ms = now
                - narrow->last_voice_ms.load(std::memory_order_relaxed);
            const int64_t dur_sec   = (now - lock_start) / 1000;

            if (!quiet) {
                std::printf("\rch%d  %.3f MHz  |  duration %lld:%02lld"
                            "  |  silence %lld ms        ",
                            US_DECT_CHANNELS[locked_ch].number,
                            US_DECT_CHANNELS[locked_ch].freq_hz / 1e6,
                            dur_sec / 60, dur_sec % 60,
                            silent_ms);
                std::fflush(stdout);
            }

            if (silent_ms < VOICE_END_MS)
                continue;

            // Call ended — switch back to wideband
            std::printf("\n--- Call ended on ch%d  (duration %lld:%02lld)"
                        "  Resuming scan...\n\n",
                        US_DECT_CHANNELS[locked_ch].number,
                        dur_sec / 60, dur_sec % 60);
            std::fflush(stdout);

            // Stop streaming FIRST, then destroy narrow context
            if (!reopen_hackrf()) break;
            narrow.reset();
            locked_ch = -1;

            monitor = std::make_unique<WidebandMonitor>(&audio);
            if (!hackrf.set_sample_rate(WidebandMonitor::WIDEBAND_SAMPLE_RATE) ||
                !apply_gains() ||
                !hackrf.set_freq(WidebandMonitor::WIDEBAND_CENTER_HZ) ||
                !hackrf.start([&monitor](const std::complex<float>* s, size_t n) {
                    monitor->ingest(s, n);
                })) {
                std::fprintf(stderr, "Wideband restart failed: %s\n",
                             hackrf.last_error());
                break;
            }

            state = State::SCANNING;
            std::printf("Scanning all %zu channels...  Ctrl-C to stop.\n\n",
                        WidebandMonitor::NUM_CHANNELS);
            std::fflush(stdout);
        }
    }

    hackrf.stop();
    std::printf("\nDone.\n");
    return 0;
}
