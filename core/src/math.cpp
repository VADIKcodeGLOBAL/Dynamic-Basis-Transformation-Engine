#include "dbte/math.hpp"

#include <cmath>

namespace dbte {

Quat quat_from_matrix(const Mat3& m) noexcept {
    // Element (row, col) is written m<row><col>.
    const float m00 = m.i.x, m10 = m.i.y, m20 = m.i.z;
    const float m01 = m.j.x, m11 = m.j.y, m21 = m.j.z;
    const float m02 = m.k.x, m12 = m.k.y, m22 = m.k.z;
    const float tr = m00 + m11 + m22;
    Quat q;
    if (tr > 0.0f) {
        const float s = std::sqrt(tr + 1.0f) * 2.0f;
        q = {(m21 - m12) / s, (m02 - m20) / s, (m10 - m01) / s, 0.25f * s};
    } else if (m00 > m11 && m00 > m22) {
        const float s = std::sqrt(1.0f + m00 - m11 - m22) * 2.0f;
        q = {0.25f * s, (m01 + m10) / s, (m02 + m20) / s, (m21 - m12) / s};
    } else if (m11 > m22) {
        const float s = std::sqrt(1.0f + m11 - m00 - m22) * 2.0f;
        q = {(m01 + m10) / s, 0.25f * s, (m12 + m21) / s, (m02 - m20) / s};
    } else {
        const float s = std::sqrt(1.0f + m22 - m00 - m11) * 2.0f;
        q = {(m02 + m20) / s, (m12 + m21) / s, 0.25f * s, (m10 - m01) / s};
    }
    return normalized(q);
}

Quat slerp(Quat a, Quat b, float t) noexcept {
    float d = dot(a, b);
    if (d < 0.0f) {  // q and -q are the same rotation: take the shorter arc.
        b = -b;
        d = -d;
    }
    if (d > 0.9995f) {  // Nearly parallel: normalized lerp is accurate and stable.
        return normalized(a * (1.0f - t) + b * t);
    }
    const float theta = std::acos(d);
    const float s = std::sin(theta);
    return a * (std::sin((1.0f - t) * theta) / s) + b * (std::sin(t * theta) / s);
}

}  // namespace dbte
