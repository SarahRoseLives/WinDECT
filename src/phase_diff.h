#pragma once
#include <complex>
#include <cmath>

namespace windect {

// GFSK phase-differential demodulator.
//
// Ported from gr-dect2 phase_diff_impl.cc (Pavel Yazev).
// Computes: ph_diff = s[n] * conj(s[n-3])
//           output  = atan2(imag(ph_diff), real(ph_diff))
//
// With 4 samples/symbol this spans 3/4 of a bit period, giving good
// phase sensitivity for DECT's BT=0.5 GFSK modulation.
//
// Input:  complex<float> IQ samples
// Output: float phase difference (radians)

class PhaseDiff {
public:
    PhaseDiff() : history_{}, head_(0) {}

    // Process one IQ sample; returns phase difference in radians.
    inline float process(std::complex<float> s) noexcept {
        history_[head_] = s;

        // Index of the sample 3 positions ago in the circular buffer
        int prev = (head_ + 4 - 3) & 3; // (head_ - 3) mod 4
        // gr-dect2 computes s[t-3] * conj(s[t])  (old × conj of current)
        // Our original had it reversed — that flips the phase sign and swaps RFP↔PP
        std::complex<float> ph_diff = history_[prev] * std::conj(s);

        head_ = (head_ + 1) & 3;
        return std::atan2(ph_diff.imag(), ph_diff.real());
    }

    void reset() noexcept {
        for (auto& h : history_) h = {};
        head_ = 0;
    }

private:
    std::complex<float> history_[4]; // circular history, length 4
    int                 head_;
};

} // namespace windect
