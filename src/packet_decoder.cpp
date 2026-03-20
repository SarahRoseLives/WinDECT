#include "packet_decoder.h"
#include <cstring>
#include <sstream>
#include <iomanip>

namespace windect {

// ── Scramble table (with corrections by Jakub Hruska) ───────────────────────
// Ported verbatim from gr-dect2 packet_decoder_impl.cc
static const uint8_t scrt[8][31] = {
    {0x3B,0xCD,0x21,0x5D,0x88,0x65,0xBD,0x44,0xEF,0x34,0x85,0x76,0x21,0x96,0xF5,
     0x13,0xBC,0xD2,0x15,0xD8,0x86,0x5B,0xD4,0x4E,0xF3,0x48,0x57,0x62,0x19,0x6F,0x51},
    {0x32,0xDE,0xA2,0x77,0x9A,0x42,0xBB,0x10,0xCB,0x7A,0x89,0xDE,0x69,0x0A,0xEC,
     0x43,0x2D,0xEA,0x27,0x79,0xA4,0x2B,0xB1,0x0C,0xB7,0xA8,0x9D,0xE6,0x90,0xAE,0xC4},
    {0x2D,0xEA,0x27,0x79,0xA4,0x2B,0xB1,0x0C,0xB7,0xA8,0x9D,0xE6,0x90,0xAE,0xC4,
     0x32,0xDE,0xA2,0x77,0x9A,0x42,0xBB,0x10,0xCB,0x7A,0x89,0xDE,0x69,0x0A,0xEC,0x43},
    {0x27,0x79,0xA4,0x2B,0xB1,0x0C,0xB7,0xA8,0x9D,0xE6,0x90,0xAE,0xC4,0x32,0xDE,
     0xA2,0x77,0x9A,0x42,0xBB,0x10,0xCB,0x7A,0x89,0xDE,0x69,0x0A,0xEC,0x43,0x2D,0xEA},
    {0x19,0x6F,0x51,0x3B,0xCD,0x21,0x5D,0x88,0x65,0xBD,0x44,0xEF,0x34,0x85,0x76,
     0x21,0x96,0xF5,0x13,0xBC,0xD2,0x15,0xD8,0x86,0x5B,0xD4,0x4E,0xF3,0x48,0x57,0x62},
    {0x13,0xBC,0xD2,0x15,0xD8,0x86,0x5B,0xD4,0x4E,0xF3,0x48,0x57,0x62,0x19,0x6F,
     0x51,0x3B,0xCD,0x21,0x5D,0x88,0x65,0xBD,0x44,0xEF,0x34,0x85,0x76,0x21,0x96,0xF5},
    {0x0C,0xB7,0xA8,0x9D,0xE6,0x90,0xAE,0xC4,0x32,0xDE,0xA2,0x77,0x9A,0x42,0xBB,
     0x10,0xCB,0x7A,0x89,0xDE,0x69,0x0A,0xEC,0x43,0x2D,0xEA,0x27,0x79,0xA4,0x2B,0xB1},
    {0x79,0xA4,0x2B,0xB1,0x0C,0xB7,0xA8,0x9D,0xE6,0x90,0xAE,0xC4,0x32,0xDE,0xA2,
     0x77,0x9A,0x42,0xBB,0x10,0xCB,0x7A,0x89,0xDE,0x69,0x0A,0xEC,0x43,0x2D,0xEA,0x27},
};

// RCRC-16 table (CCITT HDLC variant used in DECT A-field)
static const uint16_t crc_table[16] = {
    0x0000, 0x0589, 0x0b12, 0x0e9b, 0x1624, 0x13ad, 0x1d36, 0x18bf,
    0x2c48, 0x29c1, 0x275a, 0x22d3, 0x3a6c, 0x3fe5, 0x317e, 0x34f7
};

// ── PartInfo helpers ─────────────────────────────────────────────────────────

std::string PartInfo::part_id_hex() const {
    std::ostringstream ss;
    for (int i = 0; i < 5; ++i)
        ss << std::hex << std::setfill('0') << std::setw(2) << (unsigned)part_id[i];
    return ss.str();
}

// ── PacketDecoder ────────────────────────────────────────────────────────────

PacketDecoder::PacketDecoder(PartCallback on_update, VoiceCallback on_voice)
    : on_update_(std::move(on_update))
    , on_voice_(std::move(on_voice))
{
    memset(parts_, 0, sizeof(parts_));
    for (auto& p : parts_) {
        p.pair_rx_id = -1;
        g72x_init_state(&p.g721_state);
    }
}

// ── CRC helpers ──────────────────────────────────────────────────────────────

uint16_t PacketDecoder::calc_rcrc(const uint8_t* data, unsigned len) noexcept {
    uint16_t crc = 0;
    while (len--) {
        unsigned idx = (crc >> 12) ^ (*data >> 4);
        crc = crc_table[idx & 0x0F] ^ (crc << 4);
        idx = (crc >> 12) ^ (*data >> 0);
        crc = crc_table[idx & 0x0F] ^ (crc << 4);
        ++data;
    }
    return crc ^ 0x0001;
}

uint8_t PacketDecoder::calc_xcrc(const uint8_t* b_field) noexcept {
    // Extract test bits (80 bits, 1 per 5th bit with interleaving) then
    // compute a 5-bit generator polynomial CRC. Ported from gr-dect2.
    uint8_t rbits[10] = {};
    uint32_t rbit_cnt = 0, rbyte_cnt = 0;
    uint8_t rbyte = 0;

    for (int i = 0; i <= (83 - 4); ++i) {
        uint32_t bi = (uint32_t)i + 48 * (1 + (i >> 4));
        uint32_t nb = bi >> 3;
        uint32_t bw = b_field[nb];

        rbyte <<= 1;
        rbyte |= (bw >> (7 - (bi - (nb << 3)))) & 1;

        if (++rbit_cnt == 8) {
            rbits[rbyte_cnt++] = rbyte;
            rbit_cnt = 0;
        }
    }

    const uint8_t gp = 0x10;
    uint8_t crc = rbits[0];
    int i = 0;
    while (i < 10) {
        uint8_t next = (i < 9) ? rbits[i + 1] : 0;
        ++i;
        for (int j = 0; j < 8; ) {
            while (!(crc & 0x80)) {
                crc <<= 1; crc |= !!(next & 0x80); next <<= 1;
                if (++j > 7) goto next_byte;
            }
            if (j > 7) break;
            crc <<= 1; crc |= !!(next & 0x80); next <<= 1;
            ++j;
            crc ^= gp;
        }
        next_byte:;
    }
    return crc >> 4;
}

void PacketDecoder::descramble(const uint8_t* b_field_in,
                               uint8_t*       nibbles_out,
                               uint8_t        frame_number) noexcept {
    uint8_t offset = frame_number % 8;
    for (int i = 0; i < 40; ++i) {
        uint8_t descrt = b_field_in[i] ^ scrt[offset][i % 31];
        nibbles_out[2*i]   = (descrt >> 4) & 0xF;
        nibbles_out[2*i+1] =  descrt       & 0xF;
    }
}

// ── A-field decode ────────────────────────────────────────────────────────────
// The packet bits stream (388 bits) starts with the D-field (no sync).
// Layout: bits[0..63] = A-field (packed MSB first), bits[64..383] = B-field,
// bits[384..387] = X-CRC.
//
// We unpack the 64 A-field bits into 8 bytes, then validate RCRC.

bool PacketDecoder::decode_afield(const uint8_t* bits, int rx_id) noexcept {
    PartState& ps = parts_[rx_id];

    // Pack 64 bits → 8 bytes
    uint8_t af[8] = {};
    for (int i = 0; i < 64; ++i)
        af[i >> 3] = (af[i >> 3] << 1) | (bits[i] & 1);

    // Validate RCRC-16 (bytes 0–5 protected by bytes 6–7)
    uint16_t rcrc_recv = ((uint16_t)af[6] << 8) | af[7];
    uint16_t rcrc_calc = calc_rcrc(af, 6);
    if (rcrc_calc != rcrc_recv) {
        ++ps.bad_crc_cnt;
        return false;
    }

    uint8_t header = af[0];
    uint8_t ta     = (header >> 5) & 0x07; // TA field (message type)

    switch (ta) {
    case 3: // Identification — contains RFPI (base) or IPUI (portable)
        for (int i = 0; i < 5; ++i) ps.part_id[i] = af[1 + i];
        ps.part_id_rcvd = true;
        break;
    case 4: // Qt — multiframe sync / system info (frame 8 of 16)
        ps.frame_number = 8;
        if (!ps.qt_rcvd) {
            ps.qt_rcvd   = true;
            ps.log_dirty = true;
            // Any G.721 state accumulated before Qt sync is from garbage data;
            // reset so the first real voice frame starts from a known state.
            g72x_init_state(&ps.g721_state);
        }
        break;
    default:
        break;
    }

    // Voice presence: A-field bits[1..3] == 0 means voice
    bool voice = ((header >> 1) & 7) == 0;
    if (voice != ps.voice_present) {
        ps.voice_present = voice;
        ps.log_dirty     = true;
    }

    return true;
}

// ── helpers ──────────────────────────────────────────────────────────────────

bool PacketDecoder::ids_equal(const uint8_t* a, const uint8_t* b) noexcept {
    for (int i = 0; i < 5; ++i) if (a[i] != b[i]) return false;
    return true;
}

void PacketDecoder::try_pair(int rx_id) noexcept {
    PartState& me = parts_[rx_id];
    if (me.pair_rx_id >= 0 || !me.part_id_rcvd || me.type != PartType::PP) return;

    for (int i = 0; i < MAX_PARTS; ++i) {
        if (i == rx_id) continue;
        PartState& other = parts_[i];
        if (!other.active) continue;
        if (!other.part_id_rcvd) continue;
        if (ids_equal(me.part_id, other.part_id)) {
            me.pair_rx_id    = i;
            other.pair_rx_id = rx_id;
        }
    }
}

void PacketDecoder::publish_update() noexcept {
    if (!on_update_) return;
    PartInfo info[MAX_PARTS];
    int count = 0;
    for (int i = 0; i < MAX_PARTS; ++i) {
        const PartState& ps = parts_[i];
        if (!ps.active) continue;
        PartInfo& pi        = info[count++];
        pi.rx_id            = i;
        pi.type             = ps.type;
        pi.voice_present    = ps.voice_present;
        pi.part_id_valid    = ps.part_id_rcvd;
        pi.packets_ok       = ps.packet_cnt;
        pi.packets_bad_crc  = ps.bad_crc_cnt;
        pi.voice_frames_ok  = ps.voice_frames_ok;
        pi.voice_xcrc_fail  = ps.voice_xcrc_fail;
        memcpy(pi.part_id, ps.part_id, 5);
    }
    on_update_(info, count);
}

// ── B-field voice decode ──────────────────────────────────────────────────────
// Packs the B-field bits, validates X-CRC, descrambles, decodes G.721 ADPCM,
// and emits 80 int16_t PCM samples via the voice callback.
//
// On X-CRC failure, zero nibbles are decoded (keeping the adaptive predictor
// state smooth) so the output fades to near-silence rather than hard-clicking.

void PacketDecoder::decode_bfield(const uint8_t* bits, int rx_id) noexcept {
    PartState& ps = parts_[rx_id];
    if (!on_voice_) return;
    if (!ps.active || !ps.voice_present) return;
    if (!ps.qt_rcvd) return;

    // Pack 320 B-field bits (1 bit per byte) → 40 bytes, MSB-first
    uint8_t b_field[40] = {};
    for (int i = 0; i < 320; ++i)
        b_field[i >> 3] = (b_field[i >> 3] << 1) | (bits[64 + i] & 1);

    // Extract X-CRC from bits 384–387
    uint8_t x_received = 0;
    x_received |= (bits[384] & 1) << 3;
    x_received |= (bits[385] & 1) << 2;
    x_received |= (bits[386] & 1) << 1;
    x_received |= (bits[387] & 1);

    // For paired PPs, use the RFP's frame_number (the authoritative source)
    uint8_t fn = ps.frame_number;
    if (ps.type == PartType::PP && ps.pair_rx_id >= 0)
        fn = parts_[ps.pair_rx_id].frame_number;

    int16_t pcm[80];
    if (calc_xcrc(b_field) != x_received) {
        ++ps.voice_xcrc_fail;
        // Feed zero nibbles through the decoder so the predictor state
        // converges gracefully to silence instead of producing a hard pop.
        for (int i = 0; i < 80; ++i)
            pcm[i] = static_cast<int16_t>(
                g721_decoder(0, AUDIO_ENCODING_LINEAR, &ps.g721_state));
    } else {
        uint8_t nibbles[80];
        descramble(b_field, nibbles, fn);
        for (int i = 0; i < 80; ++i)
            pcm[i] = static_cast<int16_t>(
                g721_decoder(nibbles[i], AUDIO_ENCODING_LINEAR, &ps.g721_state));
        ++ps.voice_frames_ok;
    }

    on_voice_(rx_id, pcm, 80);
}

// ── Public interface ──────────────────────────────────────────────────────────

void PacketDecoder::process_packet(const ReceivedPacket& pkt) noexcept {
    int rx_id = pkt.rx_id;
    if (rx_id < 0 || rx_id >= MAX_PARTS) return;

    PartState& ps = parts_[rx_id];

    if (ps.active) {
        uint64_t seq_diff = (pkt.rx_seq - ps.rx_seq) & 0x1F;

        // Gap filling: for each missed frame, feed zero nibbles through the
        // decoder to produce comfort noise and keep the predictor state
        // smooth across the gap (matches DeDECTive / gr-dect2 behaviour).
        if (seq_diff > 1 && seq_diff <= 8 &&
            ps.qt_rcvd && ps.voice_present && on_voice_) {
            for (uint64_t gap = 0; gap < seq_diff - 1; ++gap) {
                int16_t pcm[80];
                for (int i = 0; i < 80; ++i)
                    pcm[i] = static_cast<int16_t>(
                        g721_decoder(0, AUDIO_ENCODING_LINEAR, &ps.g721_state));
                on_voice_(rx_id, pcm, 80);
            }
        }

        if (pkt.type == PartType::RFP) {
            ps.frame_number = (ps.frame_number + (uint8_t)seq_diff) & 0x0F;
            // Propagate frame number to paired PP
            if (ps.pair_rx_id >= 0)
                parts_[ps.pair_rx_id].frame_number = ps.frame_number;
        } else { // PP
            // Once paired, PP frame_number is driven by the RFP branch above.
            // Only self-advance while still unpaired.
            if (ps.pair_rx_id < 0)
                ps.frame_number = (ps.frame_number + (uint8_t)seq_diff) & 0x0F;
        }

        ps.rx_seq = pkt.rx_seq;
        ++ps.packet_cnt;
    } else {
        // New part — initialise all state
        ps.active        = true;
        ps.type          = pkt.type;
        ps.frame_number  = 0;
        ps.rx_seq        = pkt.rx_seq;
        ps.voice_present = false;
        ps.packet_cnt    = 0;
        ps.bad_crc_cnt   = 0;
        ps.part_id_rcvd  = false;
        ps.qt_rcvd       = false;
        ps.log_dirty     = true;
        ps.pair_rx_id    = -1;
        memset(ps.part_id, 0, 5);
        g72x_init_state(&ps.g721_state);
    }

    // Decode A-field (updates voice_present, part_id, frame_number, qt_rcvd)
    bool a_ok = decode_afield(pkt.bits, rx_id);

    // Decode B-field voice if A-field was valid
    if (a_ok)
        decode_bfield(pkt.bits, rx_id);

    // Try to pair PP with its RFP
    try_pair(rx_id);

    // Propagate Qt sync to paired PP and reset its G.721 state
    if (pkt.type == PartType::RFP && ps.qt_rcvd && ps.pair_rx_id >= 0) {
        PartState& pp = parts_[ps.pair_rx_id];
        if (!pp.qt_rcvd) {
            pp.qt_rcvd   = true;
            pp.log_dirty = true;
            g72x_init_state(&pp.g721_state);
        }
    }

    // Publish on: new part, id first received, voice change, or periodic stats
    bool periodic = (ps.packet_cnt % 100 == 0) && ps.packet_cnt > 0;
    if (ps.log_dirty || periodic) {
        publish_update();
        ps.log_dirty = false;
    }
}

void PacketDecoder::notify_lost(int rx_id) noexcept {
    if (rx_id < 0 || rx_id >= MAX_PARTS) return;
    PartState& ps = parts_[rx_id];
    bool had_id   = ps.part_id_rcvd;

    if (ps.pair_rx_id >= 0) {
        PartState& pair = parts_[ps.pair_rx_id];
        if (ps.type == PartType::RFP) pair.voice_present = false;
        pair.pair_rx_id = -1;
    }

    memset(&ps, 0, sizeof(ps));
    ps.pair_rx_id = -1;
    g72x_init_state(&ps.g721_state);

    if (had_id) publish_update();
}

int PacketDecoder::active_count() const noexcept {
    int n = 0;
    for (auto& p : parts_) if (p.active) ++n;
    return n;
}

bool PacketDecoder::get_part(int rx_id, PartInfo& out) const noexcept {
    if (rx_id < 0 || rx_id >= MAX_PARTS) return false;
    const PartState& ps = parts_[rx_id];
    if (!ps.active) return false;
    out.rx_id         = rx_id;
    out.type          = ps.type;
    out.voice_present = ps.voice_present;
    out.part_id_valid = ps.part_id_rcvd;
    memcpy(out.part_id, ps.part_id, 5);
    return true;
}

} // namespace windect
