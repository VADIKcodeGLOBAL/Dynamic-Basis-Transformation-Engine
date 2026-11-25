#pragma once

// Small linear algebra for dbte-core: Vec3, Mat3, Quat and Affine3.
//
// Conventions (see docs/ARCHITECTURE.md, section 8): right-handed coordinates, column
// vectors, 32-bit floats. A Mat3 stores its three columns; the columns are the basis
// vectors i, j, k, so M * v = v.x * i + v.y * j + v.z * k.

#include <algorithm>
#include <cmath>

namespace dbte {

// ---------------------------------------------------------------------------------------
// Scalars

/// Clamps v to [lo, hi]. Unlike std::clamp it is defined for lo > hi (returns hi).
[[nodiscard]] constexpr float clamp(float v, float lo, float hi) noexcept {
    return std::min(std::max(v, lo), hi);
}

/// Hermite smoothstep on [0, 1]: 0 below 0, 1 above 1, t^2 (3 - 2t) in between.
[[nodiscard]] constexpr float smoothstep01(float t) noexcept {
    const float c = clamp(t, 0.0f, 1.0f);
    return c * c * (3.0f - 2.0f * c);
}

/// Derivative of smoothstep01 with respect to t. It is zero outside (0, 1), so the
/// function is continuously differentiable everywhere.
[[nodiscard]] constexpr float smoothstep01_derivative(float t) noexcept {
    const float c = clamp(t, 0.0f, 1.0f);
    return 6.0f * c * (1.0f - c);
}

/// sin(x) / x, with the limit 1 at x = 0.
[[nodiscard]] inline float sinc(float x) noexcept {
    return std::abs(x) < 1e-4f ? 1.0f - x * x * (1.0f / 6.0f) : std::sin(x) / x;
}

// ---------------------------------------------------------------------------------------
// Vec3

struct Vec3 {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;

    constexpr bool operator==(const Vec3&) const = default;

    constexpr Vec3& operator+=(Vec3 o) noexcept {
        x += o.x;
        y += o.y;
        z += o.z;
        return *this;
    }
    constexpr Vec3& operator-=(Vec3 o) noexcept {
        x -= o.x;
        y -= o.y;
        z -= o.z;
        return *this;
    }
    constexpr Vec3& operator*=(float s) noexcept {
        x *= s;
        y *= s;
        z *= s;
        return *this;
    }
};

[[nodiscard]] constexpr Vec3 operator+(Vec3 a, Vec3 b) noexcept {
    return {a.x + b.x, a.y + b.y, a.z + b.z};
}
[[nodiscard]] constexpr Vec3 operator-(Vec3 a, Vec3 b) noexcept {
    return {a.x - b.x, a.y - b.y, a.z - b.z};
}
[[nodiscard]] constexpr Vec3 operator-(Vec3 a) noexcept { return {-a.x, -a.y, -a.z}; }
[[nodiscard]] constexpr Vec3 operator*(Vec3 a, float s) noexcept {
    return {a.x * s, a.y * s, a.z * s};
}
[[nodiscard]] constexpr Vec3 operator*(float s, Vec3 a) noexcept { return a * s; }
[[nodiscard]] constexpr Vec3 operator/(Vec3 a, float s) noexcept {
    return {a.x / s, a.y / s, a.z / s};
}

[[nodiscard]] constexpr float dot(Vec3 a, Vec3 b) noexcept {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}
[[nodiscard]] constexpr Vec3 cross(Vec3 a, Vec3 b) noexcept {
    return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
/// Component-wise product.
[[nodiscard]] constexpr Vec3 hadamard(Vec3 a, Vec3 b) noexcept {
    return {a.x * b.x, a.y * b.y, a.z * b.z};
}
[[nodiscard]] constexpr float length_squared(Vec3 a) noexcept { return dot(a, a); }
[[nodiscard]] inline float length(Vec3 a) noexcept { return std::sqrt(dot(a, a)); }

/// a / |a|, or `fallback` when a is (almost) zero.
[[nodiscard]] inline Vec3 normalized_or(Vec3 a, Vec3 fallback) noexcept {
    const float len = length(a);
    return len > 1e-30f ? a / len : fallback;
}
[[nodiscard]] inline Vec3 normalized(Vec3 a) noexcept { return normalized_or(a, Vec3{}); }

[[nodiscard]] inline Vec3 abs(Vec3 a) noexcept {
    return {std::abs(a.x), std::abs(a.y), std::abs(a.z)};
}
[[nodiscard]] constexpr Vec3 max(Vec3 a, Vec3 b) noexcept {
    return {std::max(a.x, b.x), std::max(a.y, b.y), std::max(a.z, b.z)};
}
[[nodiscard]] constexpr Vec3 min(Vec3 a, Vec3 b) noexcept {
    return {std::min(a.x, b.x), std::min(a.y, b.y), std::min(a.z, b.z)};
}
/// -1, 0 or +1 per component.
[[nodiscard]] constexpr Vec3 sign(Vec3 a) noexcept {
    const auto s = [](float v) { return v > 0.0f ? 1.0f : (v < 0.0f ? -1.0f : 0.0f); };
    return {s(a.x), s(a.y), s(a.z)};
}
/// Component by index: 0 = x, 1 = y, 2 = z.
[[nodiscard]] constexpr float component(Vec3 a, int index) noexcept {
    return index == 0 ? a.x : (index == 1 ? a.y : a.z);
}

// ---------------------------------------------------------------------------------------
// Mat3

/// 3x3 matrix stored as three columns. Default-constructed it is the identity.
struct Mat3 {
    Vec3 i{1.0f, 0.0f, 0.0f};  ///< Column 0: the first basis vector.
    Vec3 j{0.0f, 1.0f, 0.0f};  ///< Column 1: the second basis vector.
    Vec3 k{0.0f, 0.0f, 1.0f};  ///< Column 2: the third basis vector.

    constexpr bool operator==(const Mat3&) const = default;

    [[nodiscard]] static constexpr Mat3 identity() noexcept { return {}; }
    [[nodiscard]] static constexpr Mat3 zero() noexcept { return {Vec3{}, Vec3{}, Vec3{}}; }
    [[nodiscard]] static constexpr Mat3 diagonal(Vec3 d) noexcept {
        return {{d.x, 0.0f, 0.0f}, {0.0f, d.y, 0.0f}, {0.0f, 0.0f, d.z}};
    }
};

/// Column by index: 0 = i, 1 = j, 2 = k.
[[nodiscard]] constexpr Vec3 column(const Mat3& m, int index) noexcept {
    return index == 0 ? m.i : (index == 1 ? m.j : m.k);
}
/// Element at (row, col).
[[nodiscard]] constexpr float element(const Mat3& m, int row, int col) noexcept {
    return component(column(m, col), row);
}

[[nodiscard]] constexpr Vec3 operator*(const Mat3& m, Vec3 v) noexcept {
    return m.i * v.x + m.j * v.y + m.k * v.z;
}
[[nodiscard]] constexpr Mat3 operator*(const Mat3& a, const Mat3& b) noexcept {
    return {a * b.i, a * b.j, a * b.k};
}
[[nodiscard]] constexpr Mat3 operator+(const Mat3& a, const Mat3& b) noexcept {
    return {a.i + b.i, a.j + b.j, a.k + b.k};
}
[[nodiscard]] constexpr Mat3 operator-(const Mat3& a, const Mat3& b) noexcept {
    return {a.i - b.i, a.j - b.j, a.k - b.k};
}
[[nodiscard]] constexpr Mat3 operator*(const Mat3& m, float s) noexcept {
    return {m.i * s, m.j * s, m.k * s};
}
[[nodiscard]] constexpr Mat3 operator*(float s, const Mat3& m) noexcept { return m * s; }

[[nodiscard]] constexpr Mat3 transpose(const Mat3& m) noexcept {
    return {{m.i.x, m.j.x, m.k.x}, {m.i.y, m.j.y, m.k.y}, {m.i.z, m.j.z, m.k.z}};
}
[[nodiscard]] constexpr float determinant(const Mat3& m) noexcept {
    return dot(m.i, cross(m.j, m.k));
}
[[nodiscard]] constexpr float trace(const Mat3& m) noexcept { return m.i.x + m.j.y + m.k.z; }

/// Cofactor matrix cof(M) = det(M) M^-T. It maps normals: for any tangent t and normal n
/// of a surface, cof(M) n is perpendicular to M t. Needs no division, so it also works
/// for singular M.
[[nodiscard]] constexpr Mat3 cofactor(const Mat3& m) noexcept {
    return {cross(m.j, m.k), cross(m.k, m.i), cross(m.i, m.j)};
}

/// Inverse of M. The caller makes sure det(M) is not zero.
[[nodiscard]] constexpr Mat3 inverse(const Mat3& m) noexcept {
    return transpose(cofactor(m)) * (1.0f / determinant(m));
}

/// Outer product u v^T.
[[nodiscard]] constexpr Mat3 outer(Vec3 u, Vec3 v) noexcept { return {u * v.x, u * v.y, u * v.z}; }

[[nodiscard]] inline float frobenius_norm(const Mat3& m) noexcept {
    return std::sqrt(length_squared(m.i) + length_squared(m.j) + length_squared(m.k));
}

// ---------------------------------------------------------------------------------------
// Quat

/// Rotation quaternion (x, y, z, w) with w the scalar part. Default: no rotation.
struct Quat {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    float w = 1.0f;

    constexpr bool operator==(const Quat&) const = default;
};

[[nodiscard]] constexpr Quat operator*(Quat a, Quat b) noexcept {
    return {a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
            a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
            a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
            a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z};
}
[[nodiscard]] constexpr Quat operator*(Quat q, float s) noexcept {
    return {q.x * s, q.y * s, q.z * s, q.w * s};
}
[[nodiscard]] constexpr Quat operator+(Quat a, Quat b) noexcept {
    return {a.x + b.x, a.y + b.y, a.z + b.z, a.w + b.w};
}
[[nodiscard]] constexpr Quat operator-(Quat q) noexcept { return {-q.x, -q.y, -q.z, -q.w}; }
[[nodiscard]] constexpr float dot(Quat a, Quat b) noexcept {
    return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
}
[[nodiscard]] constexpr Quat conjugate(Quat q) noexcept { return {-q.x, -q.y, -q.z, q.w}; }
[[nodiscard]] inline float length(Quat q) noexcept { return std::sqrt(dot(q, q)); }
[[nodiscard]] inline Quat normalized(Quat q) noexcept {
    const float len = length(q);
    return len > 1e-30f ? q * (1.0f / len) : Quat{};
}

/// Rotates v by the unit quaternion q.
[[nodiscard]] constexpr Vec3 rotate(Quat q, Vec3 v) noexcept {
    const Vec3 u{q.x, q.y, q.z};
    const Vec3 t = cross(u, v) * 2.0f;
    return v + t * q.w + cross(u, t);
}

/// Rotation matrix of the unit quaternion q.
[[nodiscard]] constexpr Mat3 to_matrix(Quat q) noexcept {
    const float xx = q.x * q.x, yy = q.y * q.y, zz = q.z * q.z;
    const float xy = q.x * q.y, xz = q.x * q.z, yz = q.y * q.z;
    const float wx = q.w * q.x, wy = q.w * q.y, wz = q.w * q.z;
    return {{1.0f - 2.0f * (yy + zz), 2.0f * (xy + wz), 2.0f * (xz - wy)},
            {2.0f * (xy - wz), 1.0f - 2.0f * (xx + zz), 2.0f * (yz + wx)},
            {2.0f * (xz + wy), 2.0f * (yz - wx), 1.0f - 2.0f * (xx + yy)}};
}

/// Unit quaternion for a rotation of `angle` radians about the unit vector `axis`.
[[nodiscard]] inline Quat quat_from_axis_angle(Vec3 axis, float angle) noexcept {
    const float s = std::sin(0.5f * angle);
    return {axis.x * s, axis.y * s, axis.z * s, std::cos(0.5f * angle)};
}

/// Unit quaternion of a rotation matrix (det +1). Shepperd's method.
[[nodiscard]] Quat quat_from_matrix(const Mat3& m) noexcept;

/// Spherical linear interpolation along the shorter arc.
[[nodiscard]] Quat slerp(Quat a, Quat b, float t) noexcept;

// ---------------------------------------------------------------------------------------
// Affine3

/// Affine map p -> linear * p + translation. Deformer frames use it with a rotation as
/// the linear part: it maps the deformer's local coordinates to logical space.
struct Affine3 {
    Mat3 linear{};
    Vec3 translation{};

    constexpr bool operator==(const Affine3&) const = default;
};

[[nodiscard]] constexpr Vec3 transform_point(const Affine3& a, Vec3 p) noexcept {
    return a.linear * p + a.translation;
}
[[nodiscard]] constexpr Vec3 transform_vector(const Affine3& a, Vec3 v) noexcept {
    return a.linear * v;
}
/// Composition: (a * b)(p) = a(b(p)).
[[nodiscard]] constexpr Affine3 operator*(const Affine3& a, const Affine3& b) noexcept {
    return {a.linear * b.linear, a.linear * b.translation + a.translation};
}
/// Inverse of an affine map with an invertible linear part.
[[nodiscard]] constexpr Affine3 inverse(const Affine3& a) noexcept {
    const Mat3 inv = inverse(a.linear);
    return {inv, -(inv * a.translation)};
}
/// Inverse of a rigid map (the linear part is a rotation): cheaper and exact.
[[nodiscard]] constexpr Affine3 inverse_rigid(const Affine3& a) noexcept {
    const Mat3 inv = transpose(a.linear);
    return {inv, -(inv * a.translation)};
}

}  // namespace dbte
