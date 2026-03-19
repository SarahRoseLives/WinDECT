#pragma once
#include <cstdint>
#include <array>
#include <string>

namespace windect {

// DECT symbol rate (GFSK, 1.152 Mbps)
inline constexpr uint32_t DECT_SYMBOL_RATE   = 1'152'000;

// We sample at exactly 4× the symbol rate so every 4 IQ samples = 1 bit.
// No fractional resampler needed.
inline constexpr uint32_t SAMPLE_RATE        = 4 * DECT_SYMBOL_RATE; // 4,608,000 Hz

// DECT channel spacing (1.728 MHz per channel)
inline constexpr double   DECT_CHAN_SPACING  = 1'728'000.0;

// US DECT 6.0 — FCC Part 15, Subpart D
// Ten channels, 1.728 MHz spacing, 1921.536–1937.088 MHz
// (matches gr-dect2 fork channel list for US band)
struct DectChannel {
    int         number;
    uint64_t    freq_hz;
    const char *label;
};

inline constexpr std::array<DectChannel, 10> US_DECT_CHANNELS = {{
    { 0,  1'921'536'000ULL, "US-0  1921.536 MHz" },
    { 1,  1'923'264'000ULL, "US-1  1923.264 MHz" },
    { 2,  1'924'992'000ULL, "US-2  1924.992 MHz" },
    { 3,  1'926'720'000ULL, "US-3  1926.720 MHz" },
    { 4,  1'928'448'000ULL, "US-4  1928.448 MHz" },
    { 5,  1'930'176'000ULL, "US-5  1930.176 MHz" },
    { 6,  1'931'904'000ULL, "US-6  1931.904 MHz" },
    { 7,  1'933'632'000ULL, "US-7  1933.632 MHz" },
    { 8,  1'935'360'000ULL, "US-8  1935.360 MHz" },
    { 9,  1'937'088'000ULL, "US-9  1937.088 MHz" },
}};

} // namespace windect
