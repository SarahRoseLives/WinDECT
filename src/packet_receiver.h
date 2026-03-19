#pragma once
#include <cstdint>
#include <cstddef>
#include <functional>
#include <array>

namespace windect {

// Maximum concurrent DECT parts tracked simultaneously
inline constexpr int MAX_PARTS = 8;

// DECT RFP sync word (32-bit preamble + sync field).
// PP uses the bitwise complement.
inline constexpr uint32_t RFP_SYNC_FIELD = 0xAAAAE98A;

// Payload bits after the sync field captured for one packet.
// = 64 (A-field) + 320 (B-field) + 4 (X-CRC) = 388 bits
inline constexpr int P32_D_FIELD_BITS = 388;

// Inter-frame and inter-slot timing (in input samples at 4,608,000 Hz)
// DECT slot = 480 symbols × 4 samp/sym = 1920 samples
// DECT frame = 24 slots × 1920 = 46,080 samples
inline constexpr uint64_t INTER_SLOT_TIME  = 480 * 4;
inline constexpr uint64_t INTER_FRAME_TIME = INTER_SLOT_TIME * 24;
inline constexpr uint64_t TIME_TOL         = 10; // ±10 samples tolerance

enum class PartType { RFP, PP };

struct ReceivedPacket {
    uint8_t  bits[P32_D_FIELD_BITS]; // raw payload bits (1 bit per byte)
    int      rx_id;                  // internal part index [0, MAX_PARTS)
    uint32_t rx_seq;                 // frame sequence counter (mod 32)
    PartType type;
};

// Callback invoked for each received packet
using PacketCallback = std::function<void(const ReceivedPacket&)>;

// PacketReceiver
// ==============
// Ported from gr-dect2 packet_receiver_impl.cc (Pavel Yazev, GPLv3).
//
// Consumes a stream of float phase-difference values (one per IQ sample, at
// 4 samples per DECT symbol) and emits ReceivedPackets via the callback.
//
// Processing:
//   1. Maintains 4 interleaved 32-bit shift registers (one per sub-sample phase).
//   2. Each new bit is compared against RFP_SYNC_FIELD and ~RFP_SYNC_FIELD.
//   3. When sync detected, searches a short window of buffered samples for the
//      best sampling point (maximum energy), then extracts 388 payload bits at
//      the chosen phase, 1 bit every 4 samples.
//   4. Tracks up to MAX_PARTS concurrent devices by timing.
//   5. Reports inactive parts via lost_callback.

class PacketReceiver {
public:
    explicit PacketReceiver(PacketCallback on_packet,
                            std::function<void(int rx_id)> on_lost = nullptr);

    // Feed one phase-difference sample. Call for every sample from PhaseDiff.
    void process_sample(float phase_val) noexcept;

    void reset() noexcept;

private:
    // ── sync detection ────────────────────────────────────────────────────
    enum class SyncState { WaitBegin, WaitEnd, PostWait };

    static constexpr int SMPL_BUF_LEN = 32 * 4; // 128

    uint32_t  rx_bits_buf_[4];     // interleaved shift registers
    int       rx_bits_idx_;

    float     smpl_buf_[SMPL_BUF_LEN];
    int       smpl_buf_idx_;

    SyncState sync_state_;
    PartType  cur_part_type_;

    int       begin_pos_;
    int       end_pos_;
    int       smpl_cnt_;           // counts 0–3 for 4:1 decimation
    int       out_bit_cnt_;        // how many payload bits collected so far

    uint64_t  inc_smpl_cnt_;       // running input sample counter

    // ── part tracking ─────────────────────────────────────────────────────
    uint32_t  part_activity_;      // bitmask of active parts
    uint64_t  part_time_[MAX_PARTS];
    uint32_t  part_seq_[MAX_PARTS];
    int       cur_part_rx_id_;

    // ── packet assembly ───────────────────────────────────────────────────
    ReceivedPacket cur_packet_;

    // ── callbacks ─────────────────────────────────────────────────────────
    PacketCallback            on_packet_;
    std::function<void(int)>  on_lost_;

    // ── helpers ───────────────────────────────────────────────────────────
    int  find_best_smpl_point() noexcept;
    int  register_part() noexcept;
    int  check_part_activity() noexcept;
};

} // namespace windect
