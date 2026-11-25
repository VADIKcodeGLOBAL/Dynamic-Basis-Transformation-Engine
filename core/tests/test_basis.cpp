#include <doctest/doctest.h>

#include "test_util.hpp"

using namespace dbte;
using dbte::test::kPi;
using dbte::test::max_abs_diff;
using dbte::test::Rng;
using dbte::test::str;

TEST_CASE("orthonormality and orientation") {
    CHECK(is_orthonormal(Mat3::identity()));
    CHECK(is_rotation(to_matrix(quat_from_axis_angle({0.0f, 0.0f, 1.0f}, 1.0f))));
    const Mat3 shear =
        basis_from_vectors({1.0f, 0.0f, 0.0f}, {0.6f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f});
    CHECK_FALSE(is_orthonormal(shear));
    CHECK(orientation(shear) == Orientation::RightHanded);
    const Mat3 mirror = Mat3::diagonal({1.0f, 1.0f, -1.0f});
    CHECK(is_orthonormal(mirror));
    CHECK_FALSE(is_rotation(mirror));
    CHECK(orientation(mirror) == Orientation::LeftHanded);
    CHECK(orientation(Mat3::diagonal({1.0f, 1.0f, 0.0f})) == Orientation::Degenerate);
}

TEST_CASE("polar decomposition: rotation times symmetric positive definite stretch") {
    Rng rng(10);
    for (int n = 0; n < 500; ++n) {
        const Mat3 stretch = rng.basis(0.2f, 4.0f);
        const float s0 = rng.uniform(-1.0f, 1.0f);
        const float s1 = rng.uniform(-1.0f, 1.0f);
        const Mat3 m = stretch * Mat3{{1.0f, 0.0f, 0.0f}, {s0, 1.0f, 0.0f}, {0.0f, s1, 1.0f}};
        INFO("m = ", str(m));
        const PolarDecomposition pd = polar_decompose(m);
        CHECK(is_rotation(pd.rotation, 1e-4f));
        CHECK(max_abs_diff(pd.stretch, transpose(pd.stretch)) < 1e-4f);
        CHECK(max_abs_diff(pd.rotation * pd.stretch, m) < 1e-4f * frobenius_norm(m));
        // Positive definite: x^T S x > 0 for any x.
        for (int s = 0; s < 4; ++s) {
            const Vec3 x = rng.unit_vector();
            CHECK(dot(x, pd.stretch * x) > 0.0f);
        }
    }
}

TEST_CASE("polar decomposition of a rotation is the rotation itself") {
    Rng rng(11);
    for (int n = 0; n < 100; ++n) {
        const Mat3 r = to_matrix(rng.rotation());
        const PolarDecomposition pd = polar_decompose(r);
        CHECK(max_abs_diff(pd.rotation, r) < 1e-5f);
        CHECK(max_abs_diff(pd.stretch, Mat3::identity()) < 1e-5f);
    }
    // Degenerate input: identity rotation and the input as the stretch.
    const Mat3 flat = Mat3::diagonal({1.0f, 2.0f, 0.0f});
    CHECK(polar_decompose(flat).rotation == Mat3::identity());
    CHECK(max_abs_diff(nearest_rotation(Mat3::diagonal({2.0f, 3.0f, 4.0f})), Mat3::identity()) <
          1e-6f);
}

TEST_CASE("interpolated bases keep det > 0 and hit both ends") {
    Rng rng(12);
    for (int n = 0; n < 300; ++n) {
        const Mat3 a = rng.basis();
        const Mat3 b = rng.basis();
        INFO("a = ", str(a), " b = ", str(b));
        CHECK(max_abs_diff(interpolate_bases(a, b, 0.0f), a) < 1e-4f * frobenius_norm(a));
        CHECK(max_abs_diff(interpolate_bases(a, b, 1.0f), b) < 1e-4f * frobenius_norm(b));
        for (int s = 0; s <= 50; ++s) {
            const float t = static_cast<float>(s) / 50.0f;
            CHECK(determinant(interpolate_bases(a, b, t)) > 0.0f);
        }
    }
}

TEST_CASE("linear interpolation can flatten space, polar interpolation does not") {
    // Half a turn about z: blending the matrices linearly passes through det = 0.
    const Mat3 a = Mat3::identity();
    const Mat3 b = to_matrix(quat_from_axis_angle({0.0f, 0.0f, 1.0f}, kPi));
    const Mat3 linear = a * 0.5f + b * 0.5f;
    CHECK(std::abs(determinant(linear)) < 1e-6f);
    const Mat3 polar = interpolate_bases(a, b, 0.5f);
    CHECK(determinant(polar) == doctest::Approx(1.0f).epsilon(1e-5));
    CHECK(is_rotation(polar));
}

TEST_CASE("interpolation of bases with different orientation falls back to lerp") {
    const Mat3 a = Mat3::identity();
    const Mat3 b = Mat3::diagonal({1.0f, 1.0f, -1.0f});
    CHECK(max_abs_diff(interpolate_bases(a, b, 0.25f), a * 0.75f + b * 0.25f) < 1e-6f);
}
