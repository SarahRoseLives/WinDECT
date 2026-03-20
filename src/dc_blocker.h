#pragma once
#include <complex>

namespace windect {

// IIR DC-removal filter for complex IQ samples.
// H(z) = (1 - z^-1) / (1 - alpha * z^-1)
// Narrow notch at DC; passes everything else with minimal distortion.
class DCBlocker {
public:
    explicit DCBlocker(float alpha = 0.9999f) : alpha_(alpha) {}

    std::complex<float> process(std::complex<float> x) noexcept {
        auto y = x - x_prev_ + alpha_ * y_prev_;
        x_prev_ = x;
        y_prev_ = y;
        return y;
    }

    void reset() noexcept { x_prev_ = {}; y_prev_ = {}; }

private:
    float               alpha_;
    std::complex<float> x_prev_{};
    std::complex<float> y_prev_{};
};

} // namespace windect
