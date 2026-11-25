#pragma once

// Helpers shared by the dbte-core tests.

#include <algorithm>
#include <cstdint>
#include <format>
#include <numbers>
#include <random>
#include <string>

#include "dbte/dbte.hpp"

namespace dbte::test {

inline constexpr float kPi = std::numbers::pi_v<float>;

/// Deterministic random numbers that are the same with every compiler. std::mt19937 is
/// fully specified by the standard; the standard distributions are not, so they are not
/// used here.
class Rng {
public:
    explicit Rng(std::uint32_t seed) : engine_(seed) {}

    /// Uniform in [0, 1).
    float uniform() {
        return static_cast<float>(static_cast<std::uint32_t>(engine_()) >> 8) *
               (1.0f / 16777216.0f);
    }
    float uniform(float lo, float hi) { return lo + (hi - lo) * uniform(); }
    Vec3 vec3(float lo, float hi) { return {uniform(lo, hi), uniform(lo, hi), uniform(lo, hi)}; }

    Vec3 unit_vector() {
        for (;;) {
            const Vec3 v = vec3(-1.0f, 1.0f);
            const float len = length(v);
            if (len > 0.1f && len <= 1.0f) {
                return v / len;
            }
        }
    }
    Quat rotation() {
        // Separate statements: the order of evaluation of function arguments is
        // unspecified, and the sequence must not depend on the compiler.
        const Vec3 axis = unit_vector();
        const float angle = uniform(-kPi, kPi);
        return quat_from_axis_angle(axis, angle);
    }

    /// A random right-handed basis: rotation * symmetric positive definite stretch.
    Mat3 basis(float min_stretch = 0.4f, float max_stretch = 2.5f) {
        const Mat3 r = to_matrix(rotation());
        const Mat3 q = to_matrix(rotation());
        const Mat3 d =
            Mat3::diagonal({uniform(min_stretch, max_stretch), uniform(min_stretch, max_stretch),
                            uniform(min_stretch, max_stretch)});
        return r * (q * d * transpose(q));
    }

private:
    std::mt19937 engine_;
};

[[nodiscard]] inline float max_abs_diff(Vec3 a, Vec3 b) {
    const Vec3 d = abs(a - b);
    return std::max({d.x, d.y, d.z});
}

[[nodiscard]] inline float max_abs_diff(const Mat3& a, const Mat3& b) {
    return std::max({max_abs_diff(a.i, b.i), max_abs_diff(a.j, b.j), max_abs_diff(a.k, b.k)});
}

[[nodiscard]] inline float max_abs(const Mat3& m) { return max_abs_diff(m, Mat3::zero()); }

[[nodiscard]] inline std::string str(Vec3 v) {
    return std::format("({:.7g}, {:.7g}, {:.7g})", v.x, v.y, v.z);
}

[[nodiscard]] inline std::string str(const Mat3& m) {
    return std::format("[i={} j={} k={}]", str(m.i), str(m.j), str(m.k));
}

/// Central finite differences of f at p with step h.
template <typename F>
[[nodiscard]] Mat3 numeric_jacobian(F&& f, Vec3 p, float h) {
    const auto column_at = [&](Vec3 e) { return (f(p + e * h) - f(p - e * h)) / (2.0f * h); };
    return {column_at({1.0f, 0.0f, 0.0f}), column_at({0.0f, 1.0f, 0.0f}),
            column_at({0.0f, 0.0f, 1.0f})};
}

/// A random rigid frame near the origin.
[[nodiscard]] inline Affine3 random_frame(Rng& rng) {
    return {to_matrix(rng.rotation()), rng.vec3(-3.0f, 3.0f)};
}

}  // namespace dbte::test
