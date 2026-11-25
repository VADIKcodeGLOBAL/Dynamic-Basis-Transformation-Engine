#include "dbte/basis.hpp"

#include <cmath>

namespace dbte {

bool is_orthonormal(const Mat3& m, float tolerance) noexcept {
    const auto within = [tolerance](float v, float target) {
        return std::abs(v - target) <= tolerance;
    };
    return within(dot(m.i, m.i), 1.0f) && within(dot(m.j, m.j), 1.0f) &&
           within(dot(m.k, m.k), 1.0f) && within(dot(m.i, m.j), 0.0f) &&
           within(dot(m.j, m.k), 0.0f) && within(dot(m.k, m.i), 0.0f);
}

bool is_rotation(const Mat3& m, float tolerance) noexcept {
    return is_orthonormal(m, tolerance) && determinant(m) > 0.0f;
}

Orientation orientation(const Mat3& m, float epsilon) noexcept {
    const float det = determinant(m);
    if (det > epsilon) {
        return Orientation::RightHanded;
    }
    if (det < -epsilon) {
        return Orientation::LeftHanded;
    }
    return Orientation::Degenerate;
}

PolarDecomposition polar_decompose(const Mat3& m) noexcept {
    const float scale = frobenius_norm(m);
    if (!(scale > 0.0f) || !(std::abs(determinant(m)) > 1e-9f * scale * scale * scale)) {
        return {Mat3::identity(), m};
    }
    // Higham (1986): X <- (gamma X + X^-T / gamma) / 2, with the Frobenius-norm scaling
    // gamma = sqrt(|X^-1| / |X|). Converges quadratically to the orthogonal polar factor.
    Mat3 x = m;
    for (int iteration = 0; iteration < 32; ++iteration) {
        const Mat3 inv_t = cofactor(x) * (1.0f / determinant(x));
        const float gamma = std::sqrt(frobenius_norm(inv_t) / frobenius_norm(x));
        const Mat3 next = (x * gamma + inv_t * (1.0f / gamma)) * 0.5f;
        const float change = frobenius_norm(next - x);
        x = next;
        if (change <= 1e-6f) {
            break;
        }
    }
    const Mat3 s = transpose(x) * m;
    return {x, (s + transpose(s)) * 0.5f};
}

Mat3 nearest_rotation(const Mat3& m) noexcept { return polar_decompose(m).rotation; }

Mat3 interpolate_bases(const Mat3& a, const Mat3& b, float t) noexcept {
    if (!(determinant(a) > 0.0f) || !(determinant(b) > 0.0f)) {
        return a * (1.0f - t) + b * t;
    }
    const PolarDecomposition pa = polar_decompose(a);
    const PolarDecomposition pb = polar_decompose(b);
    const Mat3 rotation =
        to_matrix(slerp(quat_from_matrix(pa.rotation), quat_from_matrix(pb.rotation), t));
    const Mat3 stretch = pa.stretch * (1.0f - t) + pb.stretch * t;
    return rotation * stretch;
}

}  // namespace dbte
