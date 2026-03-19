#pragma once
#include "packet_receiver.h"
#include <cstdint>
#include <functional>
#include <string>

namespace windect {

// Part information reported to the application layer
struct PartInfo {
    int      rx_id;
    PartType type;
    uint8_t  part_id[5];      // DECT RFPI / IPUI (5 bytes)
    bool     voice_present;
    bool     part_id_valid;

    uint64_t packets_ok;      // A-field CRC passes
    uint64_t packets_bad_crc; // A-field CRC failures

    std::string part_id_hex() const;
    std::string type_str()    const { return (type == PartType::RFP) ? "RFP" : "PP"; }
};

// Callback invoked whenever the parts list changes (new part / voice change)
using PartCallback = std::function<void(const PartInfo parts[], int count)>;

// PacketDecoder
// =============
// Ported from gr-dect2 packet_decoder_impl.cc (Pavel Yazev, GPLv3).
//
// Validates A-field CRC (RCRC-16), extracts Part ID and voice presence,
// tracks pairing of RFP/PP, and descrambles the B-field for voice output.
// For Phase 1 (scanning) we focus on detection — A-field decode, CRC validation,
// and part identification. B-field voice output is prepared but not yet routed.

class PacketDecoder {
public:
    explicit PacketDecoder(PartCallback on_update = nullptr);

    // Feed a received packet from PacketReceiver
    void process_packet(const ReceivedPacket& pkt) noexcept;

    // Mark a part as lost (call when PacketReceiver fires on_lost)
    void notify_lost(int rx_id) noexcept;

    // Current snapshot of detected parts (for polling)
    int  active_count() const noexcept;
    bool get_part(int rx_id, PartInfo& out) const noexcept;

private:
    // ── A-field CRC (RCRC-16, CCITT variant) ──────────────────────────────
    static uint16_t calc_rcrc(const uint8_t* data, unsigned len) noexcept;

    // ── B-field X-CRC (5-bit, extracted from 84 test bits) ────────────────
    static uint8_t  calc_xcrc(const uint8_t* b_field) noexcept;

    // ── descramble B-field ────────────────────────────────────────────────
    static void descramble(const uint8_t* b_field_in,
                           uint8_t*       nibbles_out, // 80 nibbles (4-bit values)
                           uint8_t        frame_number) noexcept;

    // ── A-field decode ────────────────────────────────────────────────────
    // Returns true if CRC passes.
    bool decode_afield(const uint8_t* bits, int rx_id) noexcept;

    // ── part bookkeeping ──────────────────────────────────────────────────
    struct PartState {
        bool     active         = false;
        bool     voice_present  = false;
        bool     part_id_rcvd   = false;
        bool     qt_rcvd        = false;
        bool     log_dirty      = false;
        bool     rfp_fn_cor     = false;
        uint8_t  frame_number   = 0;
        uint64_t rx_seq         = 0;
        uint8_t  part_id[5]     = {};
        PartType type           = PartType::RFP;
        uint64_t packet_cnt     = 0;
        uint64_t bad_crc_cnt    = 0;
        int      pair_rx_id     = -1; // -1 = no pair
    };

    PartState parts_[MAX_PARTS];

    static bool ids_equal(const uint8_t* a, const uint8_t* b) noexcept;
    void try_pair(int rx_id) noexcept;
    void publish_update() noexcept;

    PartCallback on_update_;
};

} // namespace windect
