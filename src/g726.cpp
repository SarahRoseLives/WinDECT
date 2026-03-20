// G.726-32 ADPCM Decoder
// Ported from ITU-T G.726 reference implementation (Sun Microsystems, BSD/public domain).
// Only the decoder path is implemented (no encoder).

#include "g726.h"
#include <algorithm>
#include <cstring>

namespace g726 {

// ── 4-bit (32 kbps) look-up tables ──────────────────────────────────────────

// Inverse quantiser log-magnitude (DQLN)
static const int32_t dqlntab[16] = {
    -2048,  4, 135, 213, 273, 323, 373, 425,
      425, 373, 323, 273, 213, 135,   4, -2048
};

// Quantiser step-size log multiplier (WL)
static const int32_t wltab[16] = {
    -60,  -30,   58,  172,  334,  538, 1198, 3042,
   3042, 1198,  538,  334,  172,   58,  -30,  -60
};

// Signal transition indicator (FI) — feeds dms/dml averages
static const int32_t fitab[16] = {
    0, 0, 0, 1, 1, 1, 3, 7,
    7, 3, 1, 1, 1, 0, 0, 0
};

// ── Utilities ────────────────────────────────────────────────────────────────

// Return smallest i such that val < 2^i  (equivalent to bit-length of val).
// Matches ITU  quan(val, power2, 15).
static int32_t blog2(int32_t val) noexcept {
    for (int i = 0; i < 15; ++i)
        if (val < (1 << i)) return i;
    return 15;
}

// Floating-point multiply used by the adaptive predictor.
// an  : predictor coefficient >> 2  (signed Q9-ish)
// srn : signal history in internal 10-bit float format
static int32_t fmult(int32_t an, int32_t srn) noexcept {
    int32_t anmag  = (an > 0) ? an : ((-an) & 0x1FFF);
    int32_t anexp  = blog2(anmag) - 6;
    int32_t anmant = (anmag == 0) ? 32
                   : (anexp >= 0) ? (anmag >> anexp) & 0x3F
                                  : (anmag << -anexp) & 0x3F;
    int32_t wanexp = anexp + ((srn >> 6) & 0xF) - 13;
    int32_t wanmant = ((anmant * (srn & 0x3F)) + 0x30) >> 4;
    int32_t retval = (wanexp >= 0) ? ((wanmant << wanexp) & 0x7FFF)
                                   : (wanmant >> (-wanexp));
    return ((an ^ srn) < 0) ? -retval : retval;
}

// Convert dq (biased sign-magnitude: positive OR mag-0x8000 for negative)
// to internal float format used in dq[] history.
static int32_t dq_float(int32_t dq) noexcept {
    // mag lives in bits [14:0] regardless of sign
    int32_t sign = (dq < 0) ? 1 : 0;
    int32_t mag  = dq & 0x7FFF;
    if (mag == 0) return sign ? -0x400 : 0;
    int32_t exp  = blog2(mag);
    int32_t mant = ((mag << 6) >> exp) & 0x3F;
    int32_t r    = (exp << 6) | mant;
    return sign ? (r - 0x400) : r;
}

// Convert sr (true 2's-complement PCM) to internal float format for sr[] history.
static int32_t sr_float(int32_t sr) noexcept {
    if (sr == 0)      return 0;
    if (sr == -32768) return -0x400;
    int32_t sign = (sr < 0) ? 1 : 0;
    int32_t mag  = sign ? -sr : sr;
    int32_t exp  = blog2(mag);
    int32_t mant = ((mag << 6) >> exp) & 0x3F;
    int32_t r    = (exp << 6) | mant;
    return sign ? (r - 0x400) : r;
}

// ── Adaptive predictor ───────────────────────────────────────────────────────

static int32_t predictor_zero(const State& s) noexcept {
    int32_t r = 0;
    for (int i = 0; i < 6; ++i) r += fmult(s.b[i] >> 2, s.dq[i]);
    return r;
}

static int32_t predictor_pole(const State& s) noexcept {
    return fmult(s.a[1] >> 2, s.sr[1]) + fmult(s.a[0] >> 2, s.sr[0]);
}

// ── Step size ────────────────────────────────────────────────────────────────

static int32_t step_size(const State& s) noexcept {
    if (s.ap >= 256) return s.yu;
    int32_t y = s.yl >> 6;
    int32_t d = s.yu - y;
    return (d < 0) ? y - (((-d) * s.ap) >> 8)
                   : y + (( d  * s.ap) >> 8);
}

// ── Inverse quantiser ────────────────────────────────────────────────────────
// Returns biased sign-magnitude: non-negative for sign=0, (mag-0x8000) for sign=1.

static int32_t reconstruct(int32_t sign, int32_t dqln, int32_t y) noexcept {
    int32_t dql = dqln + (y >> 2);
    if (dql < 0) return sign ? -0x8000 : 0;
    int32_t dex = (dql >> 7) & 15;
    int32_t dqt = 128 + (dql & 127);
    int32_t dq  = (dqt << 7) >> (14 - dex);
    return sign ? (dq - 0x8000) : dq;
}

// ── State update (adaptation) ─────────────────────────────────────────────────

static void update(int32_t y, int32_t wi, int32_t fi,
                   int32_t dq, int32_t sr, int32_t dqsez, State& s) noexcept
{
    int32_t mag = dq & 0x7FFF; // magnitude from biased sign-magnitude

    // Tone detector: compare magnitude against adaptive threshold
    {
        int32_t ylint  = s.yl >> 15;
        int32_t ylfrac = (s.yl >> 10) & 31;
        int32_t thr1   = (32 + ylfrac) << ylint;
        int32_t thr2   = (ylint > 9) ? (31 << 10) : thr1;
        int32_t dqthr  = (thr2 + (thr2 >> 1)) >> 1;
        if (!s.td) s.td = (mag <= dqthr) ? 0 : 1;
    }

    // Fast step size
    s.yu = std::clamp(y + ((wi - y) >> 5), 544, 5120);
    // Slow step size
    s.yl += s.yu + ((-s.yl) >> 6);

    int32_t pk0 = (dqsez < 0) ? 1 : 0; // sign of prediction error

    if (s.td == 1) {
        // Tonal signal: zero predictors
        for (int i = 0; i < 2; ++i) s.a[i] = 0;
        for (int i = 0; i < 6; ++i) s.b[i] = 0;
    } else {
        int32_t pks1 = pk0 ^ s.pk[0];

        // Update a[1] (second-order AR coefficient)
        int32_t fa1  = pks1 ? -s.a[0] : s.a[0];
        int32_t a2p;
        if      (fa1 < -8191) a2p = s.a[1] - 128;
        else if (fa1 >  8191) a2p = s.a[1] + 128;
        else                  a2p = s.a[1] + (fa1 >> 6);
        a2p += (pk0 ^ s.pk[1]) ? -128 : 128;
        s.a[1] = std::clamp(a2p, -12288, 12288);

        // Update a[0] (first-order AR coefficient)
        if (pks1) s.a[0] -= (s.a[0] >> 8) + 192;
        else      s.a[0] -= (s.a[0] >> 8) - 192;
        {
            int32_t a1ul = 15360 - s.a[1];
            s.a[0] = std::clamp(s.a[0], -a1ul, a1ul);
        }

        // Update b[] (MA coefficients)
        for (int i = 0; i < 6; ++i) {
            if (mag) {
                if ((dq ^ s.dq[i]) >= 0) s.b[i] += (128 - (s.b[i] >> 8));
                else                      s.b[i] -= (128 + (s.b[i] >> 8));
            }
        }
    }

    // Shift histories
    for (int i = 5; i > 0; --i) s.dq[i] = s.dq[i - 1];
    s.dq[0] = dq_float(dq);

    s.sr[1] = s.sr[0];
    s.sr[0] = sr_float(sr);

    s.pk[1] = s.pk[0];
    s.pk[0] = pk0;

    // Tone detector adaptation
    if (s.td == 0) s.ap -= (s.ap >> 4);
    else           s.ap += 0x200 - (s.ap >> 4);
    if (s.ap > 256) s.ap = 256;

    // Signal-level estimates (used internally; not needed for decode output)
    s.dms += (fi - s.dms) >> 5;
    s.dml += (((s.dms << 2) - s.dml) >> 7);
}

// ── Public API ────────────────────────────────────────────────────────────────

int16_t decode32(uint8_t I, State& s) noexcept {
    I &= 0x0F; // guard: only 4 bits used

    int32_t y    = step_size(s);
    int32_t sign = (I >> 3) & 1;
    int32_t dq   = reconstruct(sign, dqlntab[I], y);

    // Signal estimate: zero-predictor + pole-predictor, each scaled >>1
    int32_t sezi = predictor_zero(s);
    int32_t sez  = sezi >> 1;
    int32_t se   = (sezi + predictor_pole(s)) >> 1;

    // Reconstructed signal: se + dq  (dq is biased sign-magnitude)
    int32_t sr = (dq < 0) ? se - (dq & 0x7FFF) : se + dq;
    sr = std::clamp(sr, -32768, 32767);

    // Prediction error (for state update)
    int32_t dqsez = dq + sez;

    update(y, wltab[I], fitab[I], dq, sr, dqsez, s);

    return static_cast<int16_t>(sr);
}

} // namespace g726
