#include <doctest/doctest.h>

#include <cmath>

#include "test_util.hpp"

using namespace dbte;
using dbte::test::kPi;
using dbte::test::max_abs_diff;
using dbte::test::Rng;
using dbte::test::str;

TEST_CASE("Vec3 arithmetic and products") {
    const Vec3 a{1.0f, 2.0f, 3.0f};
    const Vec3 b{-2.0f, 0.5f, 4.0f};
    CHECK(a + b == Vec3{-1.0f, 2.5f, 7.0f});
    CHECK(a - b == Vec3{3.0f, 1.5f, -1.0f});
    CHECK(a * 2.0f == Vec3{2.0f, 4.0f, 6.0f});
    CHECK(dot(a, b) == doctest::Approx(-2.0f + 1.0f + 12.0f));
    // Right-handed: x cross y = z.
    CHECK(cross({1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}) == Vec3{0.0f, 0.0f, 1.0f});
    CHECK(dot(cross(a, b), a) == doctest::Approx(0.0f));
    CHECK(length(normalized(b)) == doctest::Approx(1.0f));
    CHECK(normalized_or(Vec3{}, Vec3{0.0f, 1.0f, 0.0f}) == Vec3{0.0f, 1.0f, 0.0f});
    CHECK(sign(Vec3{-3.0f, 0.0f, 2.0f}) == Vec3{-1.0f, 0.0f, 1.0f});
}

TEST_CASE("Mat3 columns are the basis vectors") {
    const Mat3 m{{1.0f, 2.0f, 3.0f}, {4.0f, 5.0f, 6.0f}, {7.0f, 8.0f, 10.0f}};
    // M v = x i + y j + z k, the README formula.
    CHECK(m * Vec3{1.0f, 0.0f, 0.0f} == m.i);
    CHECK(m * Vec3{2.0f, 1.0f, 0.0f} == m.i * 2.0f + m.j);
    CHECK(element(m, 1, 0) == 2.0f);  // row 1 of column i
    CHECK(element(m, 0, 2) == 7.0f);
    CHECK(transpose(transpose(m)) == m);
    CHECK(determinant(m) == doctest::Approx(-3.0f));
    CHECK(Mat3{} == Mat3::identity());
    CHECK(trace(Mat3::diagonal({1.0f, 2.0f, 3.0f})) == 6.0f);
}

TEST_CASE("Mat3 inverse and cofactor") {
    Rng rng(1);
    for (int n = 0; n < 200; ++n) {
        const Mat3 m = rng.basis();
        INFO("m = ", str(m));
        CHECK(max_abs_diff(inverse(m) * m, Mat3::identity()) < 1e-4f);
        CHECK(max_abs_diff(m * inverse(m), Mat3::identity()) < 1e-4f);
        // cof(M)^T M = det(M) I, which is why cofactors map normals correctly.
        CHECK(max_abs_diff(transpose(cofactor(m)) * m, Mat3::identity() * determinant(m)) <
              1e-4f * std::max(1.0f, std::abs(determinant(m))));
        CHECK(determinant(transpose(m)) == doctest::Approx(determinant(m)).epsilon(1e-4));
    }
    // The cofactor matrix exists for singular matrices too.
    const Mat3 flat{{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 0.0f}};
    CHECK(cofactor(flat) == Mat3{{0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 1.0f}});
}

TEST_CASE("outer product") {
    const Mat3 m = outer({1.0f, 2.0f, 3.0f}, {4.0f, 5.0f, 6.0f});
    CHECK(element(m, 1, 2) == 2.0f * 6.0f);
    CHECK(m * Vec3{1.0f, 0.0f, 0.0f} == Vec3{4.0f, 8.0f, 12.0f});
}

TEST_CASE("Quat rotations agree with their matrices") {
    Rng rng(2);
    const Quat z90 = quat_from_axis_angle({0.0f, 0.0f, 1.0f}, 0.5f * kPi);
    CHECK(max_abs_diff(rotate(z90, {1.0f, 0.0f, 0.0f}), {0.0f, 1.0f, 0.0f}) < 1e-6f);
    for (int n = 0; n < 200; ++n) {
        const Quat a = rng.rotation();
        const Quat b = rng.rotation();
        const Vec3 v = rng.vec3(-5.0f, 5.0f);
        CHECK(max_abs_diff(rotate(a, v), to_matrix(a) * v) < 1e-5f);
        CHECK(max_abs_diff(to_matrix(a * b), to_matrix(a) * to_matrix(b)) < 1e-5f);
        CHECK(max_abs_diff(to_matrix(conjugate(a)), transpose(to_matrix(a))) < 1e-5f);
        // Round trip through a matrix gives the same rotation (q or -q).
        const Quat back = quat_from_matrix(to_matrix(a));
        CHECK(std::abs(dot(back, a)) == doctest::Approx(1.0f).epsilon(1e-5));
    }
}

TEST_CASE("slerp") {
    const Quat a{};
    const Quat b = quat_from_axis_angle({0.0f, 1.0f, 0.0f}, 0.5f * kPi);
    CHECK(std::abs(dot(slerp(a, b, 0.0f), a)) == doctest::Approx(1.0f));
    CHECK(std::abs(dot(slerp(a, b, 1.0f), b)) == doctest::Approx(1.0f));
    const Quat mid = slerp(a, b, 0.5f);
    const Quat expected = quat_from_axis_angle({0.0f, 1.0f, 0.0f}, 0.25f * kPi);
    CHECK(std::abs(dot(mid, expected)) == doctest::Approx(1.0f));
    CHECK(length(mid) == doctest::Approx(1.0f));
    // Takes the shorter arc even when the second quaternion has the opposite sign.
    CHECK(std::abs(dot(slerp(a, -b, 0.5f), expected)) == doctest::Approx(1.0f));
}

TEST_CASE("Affine3 composition and inverse") {
    Rng rng(3);
    for (int n = 0; n < 100; ++n) {
        const Affine3 a{rng.basis(), rng.vec3(-5.0f, 5.0f)};
        const Affine3 b{rng.basis(), rng.vec3(-5.0f, 5.0f)};
        const Vec3 p = rng.vec3(-5.0f, 5.0f);
        CHECK(max_abs_diff(transform_point(a * b, p), transform_point(a, transform_point(b, p))) <
              1e-3f);
        CHECK(max_abs_diff(transform_point(inverse(a), transform_point(a, p)), p) < 1e-3f);
        const Affine3 rigid{to_matrix(rng.rotation()), rng.vec3(-5.0f, 5.0f)};
        CHECK(max_abs_diff(transform_point(inverse_rigid(rigid), transform_point(rigid, p)), p) <
              1e-4f);
    }
}

TEST_CASE("scalar helpers") {
    CHECK(clamp(5.0f, 0.0f, 1.0f) == 1.0f);
    CHECK(clamp(-5.0f, 0.0f, 1.0f) == 0.0f);
    CHECK(smoothstep01(0.5f) == doctest::Approx(0.5f));
    CHECK(smoothstep01(-1.0f) == 0.0f);
    CHECK(smoothstep01(2.0f) == 1.0f);
    CHECK(smoothstep01_derivative(0.0f) == 0.0f);
    CHECK(smoothstep01_derivative(1.0f) == 0.0f);
    CHECK(smoothstep01_derivative(0.5f) == doctest::Approx(1.5f));
    CHECK(sinc(0.0f) == 1.0f);
    CHECK(sinc(1e-5f) == doctest::Approx(1.0f));
    CHECK(sinc(0.5f * kPi) == doctest::Approx(2.0f / kPi));
}
