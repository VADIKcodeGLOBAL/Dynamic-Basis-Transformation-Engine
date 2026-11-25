#pragma once

// Tools for bases: orthonormality and orientation checks, the polar decomposition and
// interpolation of bases over time that never flattens space (det stays positive).

#include "dbte/math.hpp"

namespace dbte {

/// Builds the basis matrix whose columns are i, j, k.
[[nodiscard]] constexpr Mat3 basis_from_vectors(Vec3 i, Vec3 j, Vec3 k) noexcept {
    return {i, j, k};
}

/// True when the columns are unit length and mutually perpendicular within `tolerance`.
[[nodiscard]] bool is_orthonormal(const Mat3& m, float tolerance = 1e-4f) noexcept;

/// True when m is a rotation: orthonormal and right-handed.
[[nodiscard]] bool is_rotation(const Mat3& m, float tolerance = 1e-4f) noexcept;

enum class Orientation {
    RightHanded,  ///< det > epsilon: the basis keeps the orientation of space.
    LeftHanded,   ///< det < -epsilon: the basis mirrors space.
    Degenerate,   ///< |det| <= epsilon: the basis flattens space.
};

[[nodiscard]] Orientation orientation(const Mat3& m, float epsilon = 1e-6f) noexcept;

/// M = rotation * stretch, with `stretch` symmetric positive definite.
struct PolarDecomposition {
    Mat3 rotation{};
    Mat3 stretch{};
};

/// Polar decomposition by Higham's scaled Newton iteration. For det(M) > 0 the rotation
/// is proper (det +1). For a degenerate M (det close to 0) it returns {identity, M}.
[[nodiscard]] PolarDecomposition polar_decompose(const Mat3& m) noexcept;

/// The rotation closest to M (the rotation factor of the polar decomposition).
[[nodiscard]] Mat3 nearest_rotation(const Mat3& m) noexcept;

/// Interpolates between two right-handed bases: the rotation factors with slerp, the
/// stretch factors linearly (Shoemake and Duff, 1992). The determinant stays positive
/// for every t in [0, 1], so space never flattens on the way. t = 0 gives a, t = 1
/// gives b. If either basis is not right-handed, no such path exists and the function
/// falls back to linear interpolation.
[[nodiscard]] Mat3 interpolate_bases(const Mat3& a, const Mat3& b, float t) noexcept;

}  // namespace dbte
