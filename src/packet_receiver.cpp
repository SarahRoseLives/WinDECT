#include "packet_receiver.h"
#include <cstring>
#include <cmath>
#include <algorithm>

namespace windect {

PacketReceiver::PacketReceiver(PacketCallback on_packet,
                               std::function<void(int)> on_lost)
    : rx_bits_idx_(0)
    , smpl_buf_idx_(0)
    , sync_state_(SyncState::WaitBegin)
    , cur_part_type_(PartType::RFP)
    , begin_pos_(0)
    , end_pos_(0)
    , smpl_cnt_(0)
    , out_bit_cnt_(0)
    , inc_smpl_cnt_(0)
    , part_activity_(0)
    , cur_part_rx_id_(-1)
    , on_packet_(std::move(on_packet))
    , on_lost_(std::move(on_lost))
{
    memset(rx_bits_buf_, 0, sizeof(rx_bits_buf_));
    memset(smpl_buf_,    0, sizeof(smpl_buf_));
    memset(part_time_,   0, sizeof(part_time_));
    memset(part_seq_,    0, sizeof(part_seq_));
    memset(&cur_packet_, 0, sizeof(cur_packet_));
}

void PacketReceiver::reset() noexcept {
    memset(rx_bits_buf_, 0, sizeof(rx_bits_buf_));
    memset(smpl_buf_,    0, sizeof(smpl_buf_));
    memset(part_time_,   0, sizeof(part_time_));
    memset(part_seq_,    0, sizeof(part_seq_));
    memset(&cur_packet_, 0, sizeof(cur_packet_));
    rx_bits_idx_   = 0;
    smpl_buf_idx_  = 0;
    sync_state_    = SyncState::WaitBegin;
    smpl_cnt_      = 0;
    out_bit_cnt_   = 0;
    inc_smpl_cnt_  = 0;
    part_activity_ = 0;
    cur_part_rx_id_ = -1;
}

// Find the sub-sample offset (0–3) with the highest cumulative energy
// in the buffered samples, to align the bit decision point optimally.
int PacketReceiver::find_best_smpl_point() noexcept {
    if (begin_pos_ == end_pos_) return 0; // already at unique position

    float    max_val   = 0.0f;
    int      max_index = begin_pos_;
    int      pos       = begin_pos_;

    while (true) {
        // Accumulate absolute values of 32 samples spaced 4 apart
        float acc   = 0.0f;
        int   index = pos;
        for (int j = 0; j < 32; ++j) {
            acc  += std::fabs(smpl_buf_[index]);
            index = (index - 4) & (SMPL_BUF_LEN - 1);
        }
        if (acc > max_val) {
            max_val   = acc;
            max_index = pos;
        }
        if (pos == end_pos_) break;
        pos = (pos + 1) & (SMPL_BUF_LEN - 1);
    }
    return (end_pos_ - max_index) & (SMPL_BUF_LEN - 1);
}

// Assign an incoming burst to an existing part (by timing) or allocate a new one.
int PacketReceiver::register_part() noexcept {
    if (part_activity_) {
        uint32_t part_mask = 1;
        for (int j = 0; j < MAX_PARTS; ++j, part_mask <<= 1) {
            if (!(part_activity_ & part_mask)) continue;

            uint64_t delta = (inc_smpl_cnt_ - part_time_[j]) % INTER_FRAME_TIME;
            uint32_t seq   = 0;

            if (delta < TIME_TOL) {
                seq = (uint32_t)((inc_smpl_cnt_ - part_time_[j]) / INTER_FRAME_TIME);
            } else if (INTER_FRAME_TIME - delta <= TIME_TOL) {
                seq = 1 + (uint32_t)((inc_smpl_cnt_ - part_time_[j]) / INTER_FRAME_TIME);
            } else {
                continue; // wrong timing slot
            }

            part_time_[j]  = inc_smpl_cnt_;
            part_seq_[j]   = (part_seq_[j] + seq) & 0x1F;
            return j;
        }

        // No existing part matched — allocate new
        part_mask = 1;
        for (int j = 0; j < MAX_PARTS; ++j, part_mask <<= 1) {
            if (!(part_activity_ & part_mask)) {
                part_activity_ |= part_mask;
                part_time_[j]   = inc_smpl_cnt_;
                part_seq_[j]    = 0;
                return j;
            }
        }
        return -1; // full
    } else {
        // First part ever
        part_activity_ = 1;
        part_time_[0]  = inc_smpl_cnt_;
        part_seq_[0]   = 0;
        return 0;
    }
}

// Return the rx_id of any part that has not been heard for >4 frames, else -1.
int PacketReceiver::check_part_activity() noexcept {
    if (!part_activity_) return -1;
    uint32_t part_mask = 1;
    for (int j = 0; j < MAX_PARTS; ++j, part_mask <<= 1) {
        if ((part_activity_ & part_mask) &&
            (inc_smpl_cnt_ - part_time_[j] > 4 * INTER_FRAME_TIME)) {
            part_activity_ &= ~part_mask;
            return j;
        }
    }
    return -1;
}

void PacketReceiver::process_sample(float phase_val) noexcept {
    // Bit decision: positive phase → 0, negative → 1
    uint32_t rx_bit = (phase_val >= 0.0f) ? 0u : 1u;

    // Push bit into the current interleaved shift register
    rx_bits_buf_[rx_bits_idx_] = (rx_bits_buf_[rx_bits_idx_] << 1) | rx_bit;

    // Save sample in circular buffer for best-sample-point search
    smpl_buf_[smpl_buf_idx_] = phase_val;

    switch (sync_state_) {
    case SyncState::WaitBegin: {
        // Check RFP sync (and PP sync = bitwise complement)
        bool rfp_sync = (rx_bits_buf_[rx_bits_idx_] == RFP_SYNC_FIELD);
        bool pp_sync  = (rx_bits_buf_[rx_bits_idx_] == ~RFP_SYNC_FIELD);

        if (rfp_sync || pp_sync) {
            cur_part_type_ = rfp_sync ? PartType::RFP : PartType::PP;
            begin_pos_     = smpl_buf_idx_;
            sync_state_    = SyncState::WaitEnd;
        }
        break;
    }

    case SyncState::WaitEnd: {
        uint32_t expected = (cur_part_type_ == PartType::RFP)
                            ? RFP_SYNC_FIELD : ~RFP_SYNC_FIELD;
        bool still_sync = (rx_bits_buf_[rx_bits_idx_] == expected);

        if (!still_sync) {
            end_pos_ = (smpl_buf_idx_ - 1) & (SMPL_BUF_LEN - 1);

            // Find optimal decimation offset
            smpl_cnt_ = (1 + find_best_smpl_point()) & 3;

            cur_part_rx_id_ = register_part();
            if (cur_part_rx_id_ < 0) {
                sync_state_ = SyncState::WaitBegin;
                break;
            }

            out_bit_cnt_            = 0;
            cur_packet_.rx_id       = cur_part_rx_id_;
            cur_packet_.rx_seq      = part_seq_[cur_part_rx_id_];
            cur_packet_.type        = cur_part_type_;
            sync_state_             = SyncState::PostWait;
        }
        break;
    }

    case SyncState::PostWait:
        // Collect one bit every 4 samples (4:1 decimation)
        if (smpl_cnt_ == 0) {
            cur_packet_.bits[out_bit_cnt_] = static_cast<uint8_t>(rx_bit);
            ++out_bit_cnt_;

            if (out_bit_cnt_ == P32_D_FIELD_BITS) {
                if (on_packet_) on_packet_(cur_packet_);
                sync_state_ = SyncState::WaitBegin;
            }
        }
        break;
    }

    // Advance counters
    smpl_buf_idx_  = (smpl_buf_idx_  + 1) & (SMPL_BUF_LEN - 1);
    rx_bits_idx_   = (rx_bits_idx_   + 1) & 3;
    smpl_cnt_      = (smpl_cnt_      + 1) & 3;
    ++inc_smpl_cnt_;

    // Check for inactive parts
    int lost = check_part_activity();
    if (lost >= 0 && on_lost_) on_lost_(lost);
}

} // namespace windect
