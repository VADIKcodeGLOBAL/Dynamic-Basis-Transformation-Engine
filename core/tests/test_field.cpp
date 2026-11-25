#include <doctest/doctest.h>

#include <algorithm>
#include <cmath>

#include "test_util.hpp"

using namespace dbte;
using dbte::test::kPi;
using dbte::test::max_abs;
using dbte::test::max_abs_diff;
using dbte::test::Rng;
using dbte::test::str;

namespace {

/// A chain with one deformer of every kind, set up so that no link folds for points in
/// [-6, 6]^3.
DeformationField demo_chain() {
    AffineVolume sphere;
    sphere.centre = {1.0f, 0.0f, 0.0f};
    sphere.basis = basis_from_vectors({1.0f, 0.0f, 0.0f}, {0.15f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.1f});
    sphere.inner = 1.0f;
    sphere.outer = 5.0f;

    Bend bend;
    bend.frame.translation = {0.0f, -1.0f, 0.0f};
    bend.curvature = 0.08f;
    bend.length = 6.0f;

    Twist twist;
    twist.frame.linear = to_matrix(quat_from_axis_angle({1.0f, 0.0f, 0.0f}, 0.5f * kPi));
    twist.length = 8.0f;
    twist.angle = 0.6f;

    Taper taper;
    taper.frame.translation = {0.0f, -4.0f, 0.0f};
    taper.length = 8.0f;
    taper.scale_x = 0.8f;
    taper.scale_z = 1.2f;

    CorridorCompression corridor;
    corridor.frame.translation = {0.0f, -6.0f, 0.0f};
    corridor.max_distance = 30.0f;
    corridor.player = 6.0f;

    AffineVolume box;
    box.shape = VolumeShape::Box;
    box.centre = {-1.0f, 1.0f, 0.5f};
    box.orientation = quat_from_axis_angle({0.0f, 1.0f, 0.0f}, 0.4f);
    box.half_extents = {1.0f, 0.5f, 1.0f};
    box.basis = basis_from_vectors({1.0f, 0.1f, 0.0f}, {0.0f, 0.9f, 0.0f}, {0.0f, 0.0f, 1.0f});
    box.inner = 0.0f;
    box.outer = 4.0f;

    return DeformationField({sphere, bend, twist, taper, corridor, box});
}

}  // namespace

TEST_CASE("an empty field is the identity") {
    const DeformationField field;
    const Vec3 p{1.0f, 2.0f, 3.0f};
    CHECK(field.map(p) == p);
    CHECK(field.jacobian(p) == Mat3::identity());
    CHECK(field.inverse(p).point == p);
    CHECK(field.inverse(p).converged);
}

TEST_CASE("chain: Phi is the composition and J the product of the links") {
    const DeformationField field = demo_chain();
    Rng rng(200);
    for (int n = 0; n < 500; ++n) {
        const Vec3 p = rng.vec3(-6.0f, 6.0f);
        INFO("p = ", str(p));
        Vec3 q = p;
        Mat3 j = Mat3::identity();
        for (const Deformer& d : field.deformers()) {
            const Evaluation e = evaluate(d, q);
            q = e.point;
            j = e.jacobian * j;
        }
        const Evaluation e = field.evaluate(p);
        CHECK(max_abs_diff(e.point, q) < 1e-6f);
        CHECK(max_abs_diff(e.jacobian, j) < 1e-6f);
        CHECK(determinant(e.jacobian) > 0.0f);
    }
}

TEST_CASE("chain: inverse brings mapped points back") {
    const DeformationField field = demo_chain();
    Rng rng(201);
    int newton_points = 0;
    for (int n = 0; n < 2000; ++n) {
        const Vec3 p = rng.vec3(-6.0f, 6.0f);
        INFO("p = ", str(p));
        const InverseResult back = field.inverse(field.map(p));
        CHECK(back.converged);
        CHECK(max_abs_diff(back.point, p) < 1e-3f);
        newton_points += back.iterations > 0 ? 1 : 0;
    }
    // Most sampled points lie in the soft edge of a volume, which needs Newton's method.
    CHECK(newton_points > 100);
}

TEST_CASE("chain: mapped normals stay perpendicular to mapped tangents") {
    const DeformationField field = demo_chain();
    Rng rng(202);
    for (int n = 0; n < 500; ++n) {
        const Vec3 p = rng.vec3(-6.0f, 6.0f);
        const Vec3 normal = rng.unit_vector();
        const Vec3 tangent = normalized(cross(normal, rng.unit_vector()));
        const Vec3 mapped_normal = field.map_normal(p, normal);
        const Vec3 mapped_tangent = field.map_tangent(p, tangent);
        CHECK(length(mapped_normal) == doctest::Approx(1.0f));
        CHECK(std::abs(dot(mapped_normal, mapped_tangent)) <= 1e-4f * length(mapped_tangent));
    }
}

TEST_CASE("chain: analytic Jacobian matches finite differences") {
    // Smooth links only, so there are no kinks to step over.
    GlobalBasis global;
    global.basis = basis_from_vectors({1.0f, 0.0f, 0.2f}, {0.4f, 1.0f, 0.0f}, {0.0f, 0.2f, 1.0f});
    CorridorCompression corridor;
    corridor.max_distance = 6.0f;
    corridor.player = 2.0f;
    corridor.frame.linear = to_matrix(quat_from_axis_angle({0.0f, 0.0f, 1.0f}, 0.3f));
    const DeformationField field({global, corridor});
    Rng rng(203);
    for (int n = 0; n < 500; ++n) {
        const Vec3 p = rng.vec3(-5.0f, 5.0f);
        const Mat3 j = field.jacobian(p);
        const Mat3 fd = test::numeric_jacobian([&](Vec3 x) { return field.map(x); }, p, 2e-3f);
        INFO("p = ", str(p), " J = ", str(j), " fd = ", str(fd));
        CHECK(max_abs_diff(j, fd) <= 2e-3f * std::max(1.0f, max_abs(j)));
    }
}

TEST_CASE("layers select which deformers apply") {
    GlobalBasis a;
    a.basis = Mat3::diagonal({2.0f, 1.0f, 1.0f});
    a.layers = 0b01u;
    GlobalBasis b;
    b.basis = Mat3::diagonal({1.0f, 3.0f, 1.0f});
    b.layers = 0b10u;
    const DeformationField field({a, b});
    const Vec3 p{1.0f, 1.0f, 1.0f};
    CHECK(field.map(p, 0b01u) == Vec3{2.0f, 1.0f, 1.0f});
    CHECK(field.map(p, 0b10u) == Vec3{1.0f, 3.0f, 1.0f});
    CHECK(field.map(p, 0b11u) == Vec3{2.0f, 3.0f, 1.0f});
    CHECK(field.map(p, 0b100u) == p);
    CHECK(max_abs_diff(field.inverse(Vec3{2.0f, 3.0f, 1.0f}, 0b11u).point, p) < 1e-6f);
}

TEST_CASE("frame follows the rotation of space") {
    // A pure rotation: the frame is the rotation itself.
    GlobalBasis rotation;
    rotation.basis = to_matrix(quat_from_axis_angle({0.0f, 1.0f, 0.0f}, 0.7f));
    CHECK(max_abs_diff(DeformationField({rotation}).frame({1.0f, 2.0f, 3.0f}), rotation.basis) <
          1e-5f);

    // A shear has a rotation part too, but stays close to the identity for small shears.
    GlobalBasis shear;
    shear.basis = basis_from_vectors({1.0f, 0.0f, 0.0f}, {0.1f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f});
    CHECK(is_rotation(DeformationField({shear}).frame({0.0f, 0.0f, 0.0f})));

    // After a 90 degree bend, "forward" (y) has turned into "up" (z) and "up" into -y.
    Bend bend;
    bend.curvature = 0.25f;
    bend.length = 2.0f * kPi;
    const Mat3 f = DeformationField({bend}).frame({0.0f, 8.0f, 0.5f});
    CHECK(is_rotation(f));
    CHECK(max_abs_diff(f * Vec3{0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}) < 1e-4f);
    CHECK(max_abs_diff(f * Vec3{0.0f, 0.0f, 1.0f}, {0.0f, -1.0f, 0.0f}) < 1e-4f);
}
