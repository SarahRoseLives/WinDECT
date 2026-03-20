#pragma once
#include <cstdint>

// G.726-32 (4-bit / 32 kbps) ADPCM Decoder
// Based on ITU-T G.726 reference implementation (Sun Microsystems, public domain).
//
// DECT voice uses G.726-32: 80 nibbles (4-bit codewords) per 10 ms frame
// at 8 000 Hz → 16-bit signed PCM out.

namespace g726 {

struct State {
    int32_t yl  = 34816; // slow quantizer step size (log)
    int32_t yu  = 544;   // fast quantizer step size
    int32_t dms = 0;     // short-term average magnitude
    int32_t dml = 0;     // long-term average magnitude
    int32_t ap  = 0;     // tone / transition detector
    int32_t a[2] = {};   // AR predictor coefficients  (a1, a2)
    int32_t b[6] = {};   // MA predictor coefficients  (b1..b6)
    int32_t pk[2] = {};  // previous two prediction-error signs
    int32_t dq[6] = {};  // quantised-difference history (float-encoded)
    int32_t sr[2] = {};  // reconstructed-signal history (float-encoded)
    int32_t td  = 0;     // tone detector flag
};

// Decode one 4-bit G.726-32 codeword (0-15) → 16-bit PCM sample.
int16_t decode32(uint8_t code, State& s) noexcept;

// Reset decoder to power-on defaults.
inline void reset(State& s) noexcept { s = State{}; }

} // namespace g726
