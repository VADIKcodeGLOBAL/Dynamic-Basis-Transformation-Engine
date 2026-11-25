#include <doctest/doctest.h>

#include <algorithm>
#include <cmath>

#include "test_util.hpp"

using namespace dbte;
using dbte::test::kPi;
using dbte::test::max_abs;
using dbte::test::max_abs_diff;
using dbte::test::random_frame;
using dbte::test::Rng;
using dbte::test::str;

namespace {

// Finite-difference step at |p| <= 10. Further out the step grows with |p| so that float
// rounding in Phi(p +- h) stays small against h.
constexpr float kStep = 2e-3f;

float step_at(Vec3 p) { return kStep * std::max(1.0f, 0.1f * length(p)); }

/// Properties every deformer must have at every sampled point p:
/// det J > 0, analytic J equals finite differences (away from kinks, where J is only
/// continuous), the inverse brings Phi(p) back to p, and mapped normals stay
/// perpendicular to mapped tangents.
template <typename D, typename Sample, typename NearKink>
void check_properties(const D& d, Rng& rng, int points, Sample&& sample, NearKink&& near_kink,
                      float inverse_tolerance = 1e-3f) {
    for (int n = 0; n < points; ++n) {
        const Vec3 p = sample(rng);
        INFO("p = ", str(p));
        const Evaluation e = evaluate(d, p);
        INFO("J = ", str(e.jacobian));
        CHECK(determinant(e.jacobian) > 0.0f);

        // Skip finite differences across a kink, where J is only continuous.
        const float h = step_at(p);
        if (!near_kink(p, 4.0f * h)) {
            const Mat3 fd = test::numeric_jacobian([&](Vec3 x) { return map(d, x); }, p, h);
            INFO("finite differences = ", str(fd));
            CHECK(max_abs_diff(fd, e.jacobian) <= 2e-3f * std::max(1.0f, max_abs(e.jacobian)));
        }

        const InverseResult back = inverse(d, e.point);
        INFO("inverse = ", str(back.point));
        CHECK(back.converged);
        CHECK(max_abs_diff(back.point, p) <= inverse_tolerance * std::max(1.0f, length(p)));

        const Vec3 normal = rng.unit_vector();
        const Vec3 tangent = normalized(cross(normal, rng.unit_vector()));
        const Vec3 mapped_normal = normalized(cofactor(e.jacobian) * normal);
        const Vec3 mapped_tangent = e.jacobian * tangent;
        CHECK(std::abs(dot(mapped_normal, mapped_tangent)) <= 1e-4f * length(mapped_tangent));
    }
}

/// Local coordinates of p in a rigid frame.
Vec3 to_local(const Affine3& frame, Vec3 p) { return transform_point(inverse_rigid(frame), p); }

bool near_segment_ends(const Affine3& frame, float length_y, Vec3 p, float margin) {
    const float y = to_local(frame, p).y;
    return std::abs(y) < margin || std::abs(y - length_y) < margin;
}

/// A random point given in the local frame.
Vec3 local_sample(Rng& rng, const Affine3& frame, Vec3 lo, Vec3 hi) {
    const Vec3 local{rng.uniform(lo.x, hi.x), rng.uniform(lo.y, hi.y), rng.uniform(lo.z, hi.z)};
    return transform_point(frame, local);
}

Mat3 small_perturbation(Rng& rng, float amount) {
    return Mat3::identity() +
           Mat3{rng.vec3(-amount, amount), rng.vec3(-amount, amount), rng.vec3(-amount, amount)};
}

}  // namespace

// --- Global basis ------------------------------------------------------------------------

TEST_CASE("global basis is the README formula") {
    // The shear preset of the prototype: j leans along x.
    GlobalBasis d;
    d.origin = {1.0f, 0.0f, 0.0f};
    d.basis = basis_from_vectors({1.0f, 0.0f, 0.0f}, {0.6f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f});
    // Phi(p) = o + x i + y j + z k with (x, y, z) = p - o.
    CHECK(max_abs_diff(map(d, {1.0f, 2.0f, 0.0f}), {2.2f, 2.0f, 0.0f}) < 1e-6f);
    CHECK(map(d, d.origin) == d.origin);
    CHECK(jacobian(d, {5.0f, 5.0f, 5.0f}) == d.basis);
    d.weight = 0.5f;
    CHECK(max_abs_diff(map(d, {1.0f, 2.0f, 0.0f}), {1.6f, 2.0f, 0.0f}) < 1e-6f);
}

TEST_CASE("global basis properties") {
    Rng rng(100);
    for (int n = 0; n < 20; ++n) {
        GlobalBasis d;
        d.origin = rng.vec3(-3.0f, 3.0f);
        d.basis = rng.basis();
        check_properties(
            d, rng, 100, [](Rng& r) { return r.vec3(-10.0f, 10.0f); },
            [](Vec3, float) { return false; });
    }
}

// --- Affine volume -----------------------------------------------------------------------

TEST_CASE("affine volume: full effect inside, identity outside") {
    AffineVolume d;
    d.centre = {2.0f, 1.0f, -1.0f};
    d.basis = basis_from_vectors({1.0f, 0.0f, 0.0f}, {0.3f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.2f});
    d.inner = 2.0f;
    d.outer = 5.0f;
    const Vec3 inside = d.centre + Vec3{0.5f, 1.0f, -0.5f};
    CHECK(max_abs_diff(map(d, inside), d.centre + d.basis * (inside - d.centre)) < 1e-5f);
    CHECK(max_abs_diff(jacobian(d, inside), d.basis) < 1e-6f);
    const Vec3 outside = d.centre + Vec3{4.0f, 3.0f, 0.0f};
    CHECK(map(d, outside) == outside);
    CHECK(jacobian(d, outside) == Mat3::identity());
}

TEST_CASE("affine volume: no jump at the edges") {
    Rng rng(101);
    AffineVolume d;
    d.basis = basis_from_vectors({1.0f, 0.0f, 0.0f}, {0.5f, 1.0f, 0.0f}, {0.0f, 0.2f, 1.0f});
    d.inner = 1.0f;
    d.outer = 6.0f;
    for (int n = 0; n < 200; ++n) {
        const Vec3 dir = rng.unit_vector();
        for (const float r : {d.inner, d.outer}) {
            const Evaluation in = evaluate(d, dir * (r - 1e-4f));
            const Evaluation out = evaluate(d, dir * (r + 1e-4f));
            CHECK(max_abs_diff(in.point, out.point) < 1e-3f);
            CHECK(max_abs_diff(in.jacobian, out.jacobian) < 1e-3f);
        }
    }
}

TEST_CASE("affine sphere volume properties") {
    Rng rng(102);
    for (int n = 0; n < 20; ++n) {
        AffineVolume d;
        d.centre = rng.vec3(-2.0f, 2.0f);
        d.basis = small_perturbation(rng, 0.07f);
        d.inner = rng.uniform(0.5f, 2.0f);
        d.outer = d.inner + rng.uniform(4.0f, 5.0f);
        d.weight = rng.uniform(0.3f, 1.0f);
        REQUIRE(validate(d).ok());
        check_properties(
            d, rng, 100,
            [&](Rng& r) {
                const Vec3 dir = r.unit_vector();
                return d.centre + dir * r.uniform(0.0f, d.outer + 1.0f);
            },
            [&](Vec3 p, float margin) {
                const float dist = volume_distance(d, p);
                return std::abs(dist - d.inner) < margin || std::abs(dist - d.outer) < margin;
            });
    }
}

TEST_CASE("affine box volume properties") {
    Rng rng(103);
    for (int n = 0; n < 20; ++n) {
        AffineVolume d;
        d.shape = VolumeShape::Box;
        d.centre = rng.vec3(-2.0f, 2.0f);
        d.orientation = rng.rotation();
        d.half_extents = rng.vec3(0.5f, 2.0f);
        d.basis = small_perturbation(rng, 0.05f);
        d.inner = rng.uniform(0.0f, 1.0f);
        d.outer = d.inner + rng.uniform(3.0f, 4.0f);
        REQUIRE(validate(d).ok());
        const Vec3 reach =
            d.half_extents + Vec3{d.outer, d.outer, d.outer} + Vec3{1.0f, 1.0f, 1.0f};
        check_properties(
            d, rng, 100,
            [&](Rng& r) {
                return d.centre + rotate(d.orientation, hadamard(r.vec3(-1.0f, 1.0f), reach));
            },
            [&](Vec3 p, float margin) {
                const float dist = volume_distance(d, p);
                const Vec3 local = abs(rotate(conjugate(d.orientation), p - d.centre));
                const Vec3 face = abs(local - d.half_extents);
                return std::abs(dist - d.inner) < margin || std::abs(dist - d.outer) < margin ||
                       std::min({face.x, face.y, face.z}) < margin;
            });
    }
}

// --- Bend --------------------------------------------------------------------------------

TEST_CASE("bend by 90 degrees turns the corridor upwards") {
    Bend d;
    const float radius = 4.0f;
    d.curvature = 1.0f / radius;
    d.length = 0.5f * kPi * radius;  // A quarter circle.
    // Before the bend nothing changes.
    CHECK(map(d, {0.3f, -2.0f, 0.5f}) == Vec3{0.3f, -2.0f, 0.5f});
    // The end of the centre line lands a quarter turn later, at (0, r, r).
    CHECK(max_abs_diff(map(d, {0.0f, d.length, 0.0f}), {0.0f, radius, radius}) < 1e-5f);
    // Beyond the bend the corridor continues straight along +z.
    CHECK(max_abs_diff(map(d, {0.0f, d.length + 3.0f, 0.0f}), {0.0f, radius, radius + 3.0f}) <
          1e-5f);
    // A floor point 1 m below the centre line is on the outside of the bend.
    CHECK(max_abs_diff(map(d, {0.0f, d.length, -1.0f}), {0.0f, radius + 1.0f, radius}) < 1e-5f);
    // The local basis at the end: forward (y) has turned into up (z).
    const Mat3 j = jacobian(d, {0.0f, d.length + 1.0f, 0.0f});
    CHECK(max_abs_diff(j.j, {0.0f, 0.0f, 1.0f}) < 1e-6f);
    CHECK(max_abs_diff(j.k, {0.0f, -1.0f, 0.0f}) < 1e-6f);
}

TEST_CASE("bend with zero curvature or zero weight is the identity") {
    Bend d;
    d.length = 5.0f;
    const Vec3 p{1.0f, 2.0f, 3.0f};
    CHECK(map(d, p) == p);
    d.curvature = 0.3f;
    d.weight = 0.0f;
    CHECK(map(d, p) == p);
    CHECK(jacobian(d, p) == Mat3::identity());
}

TEST_CASE("bend is stable for tiny curvature") {
    Bend d;
    d.curvature = 1e-7f;
    d.length = 10.0f;
    const Vec3 p{0.5f, 4.0f, -1.0f};
    CHECK(max_abs_diff(map(d, p), p) < 1e-5f);
    const InverseResult back = inverse(d, map(d, p));
    CHECK(back.converged);
    CHECK(max_abs_diff(back.point, p) < 1e-5f);
}

TEST_CASE("bend properties") {
    Rng rng(104);
    for (int n = 0; n < 20; ++n) {
        Bend d;
        d.frame = random_frame(rng);
        d.curvature = rng.uniform(-0.4f, 0.4f);
        d.length = std::min(rng.uniform(1.0f, 6.0f), 0.5f * kPi / std::abs(d.curvature));
        d.half_extents = {2.0f, 4.0f, 1.5f};
        d.weight = rng.uniform(0.5f, 1.0f);
        REQUIRE(validate(d).ok());
        check_properties(
            d, rng, 100,
            [&](Rng& r) {
                return local_sample(r, d.frame, {-2.0f, -3.0f, -1.5f},
                                    {2.0f, d.length + 3.0f, 1.5f});
            },
            [&](Vec3 p, float margin) { return near_segment_ends(d.frame, d.length, p, margin); });
    }
}

// --- Twist and taper ---------------------------------------------------------------------

TEST_CASE("twist rotates cross-sections and preserves volume") {
    Twist d;
    d.length = 4.0f;
    d.angle = 0.5f * kPi;
    CHECK(map(d, {1.0f, -1.0f, 0.0f}) == Vec3{1.0f, -1.0f, 0.0f});
    // A quarter turn counter-clockwise about +y takes +x to -z.
    CHECK(max_abs_diff(map(d, {1.0f, 4.0f, 0.0f}), {0.0f, 4.0f, -1.0f}) < 1e-6f);
    CHECK(max_abs_diff(map(d, {1.0f, 9.0f, 0.0f}), {0.0f, 9.0f, -1.0f}) < 1e-6f);
    CHECK(determinant(jacobian(d, {0.7f, 2.0f, -0.4f})) == doctest::Approx(1.0f));
}

TEST_CASE("twist properties") {
    Rng rng(105);
    for (int n = 0; n < 20; ++n) {
        Twist d;
        d.frame = random_frame(rng);
        d.length = rng.uniform(1.0f, 6.0f);
        d.angle = rng.uniform(-kPi, kPi);
        check_properties(
            d, rng, 100,
            [&](Rng& r) {
                return local_sample(r, d.frame, {-2.0f, -2.0f, -2.0f},
                                    {2.0f, d.length + 2.0f, 2.0f});
            },
            [&](Vec3 p, float margin) { return near_segment_ends(d.frame, d.length, p, margin); });
    }
}

TEST_CASE("taper scales cross-sections") {
    Taper d;
    d.length = 2.0f;
    d.scale_x = 0.5f;
    d.scale_z = 2.0f;
    CHECK(map(d, {1.0f, -1.0f, 1.0f}) == Vec3{1.0f, -1.0f, 1.0f});
    CHECK(max_abs_diff(map(d, {1.0f, 2.0f, 1.0f}), {0.5f, 2.0f, 2.0f}) < 1e-6f);
    CHECK(max_abs_diff(map(d, {1.0f, 5.0f, 1.0f}), {0.5f, 5.0f, 2.0f}) < 1e-6f);
}

TEST_CASE("taper properties") {
    Rng rng(106);
    for (int n = 0; n < 20; ++n) {
        Taper d;
        d.frame = random_frame(rng);
        d.length = rng.uniform(1.0f, 6.0f);
        d.scale_x = rng.uniform(0.3f, 2.0f);
        d.scale_z = rng.uniform(0.3f, 2.0f);
        REQUIRE(validate(d).ok());
        check_properties(
            d, rng, 100,
            [&](Rng& r) {
                return local_sample(r, d.frame, {-2.0f, -2.0f, -2.0f},
                                    {2.0f, d.length + 2.0f, 2.0f});
            },
            [&](Vec3 p, float margin) { return near_segment_ends(d.frame, d.length, p, margin); });
    }
}

// --- Corridor compression ------------------------------------------------------------------

TEST_CASE("corridor compression keeps the entrance and limits the visible distance") {
    CorridorCompression d;
    d.max_distance = 10.0f;
    d.player = 50.0f;
    // The entrance (local origin) stays where it is.
    CHECK(max_abs_diff(map(d, {0.0f, 0.0f, 0.0f}), {0.0f, 0.0f, 0.0f}) < 1e-5f);
    // Around the player space is unchanged to first order.
    CHECK(jacobian(d, {0.0f, d.player, 0.0f}).j.y == doctest::Approx(1.0f));
    // A door 30 m ahead appears just under max_distance ahead of the player, and one
    // 1 km ahead no further than that.
    const float player_y = map(d, {0.0f, d.player, 0.0f}).y;
    const float door_y = map(d, {0.0f, d.player + 30.0f, 0.0f}).y;
    CHECK(door_y - player_y < d.max_distance);
    CHECK(door_y - player_y > 0.99f * d.max_distance);
    CHECK(map(d, {0.0f, 1000.0f, 0.0f}).y - player_y <= d.max_distance * (1.0f + 1e-6f));
    // Space is compressed only along the corridor.
    CHECK(map(d, {3.0f, 20.0f, -2.0f}).x == 3.0f);
    CHECK(map(d, {3.0f, 20.0f, -2.0f}).z == -2.0f);
}

TEST_CASE("corridor compression properties") {
    Rng rng(107);
    for (int n = 0; n < 20; ++n) {
        CorridorCompression d;
        d.frame = random_frame(rng);
        d.max_distance = rng.uniform(5.0f, 20.0f);
        d.player = rng.uniform(-5.0f, 50.0f);
        d.weight = rng.uniform(0.3f, 1.0f);
        const float a = d.max_distance / d.weight;
        // Within 3a of the player; much further, float precision runs out because those
        // points are squeezed into a sliver.
        check_properties(
            d, rng, 100,
            [&](Rng& r) {
                return local_sample(r, d.frame, {-3.0f, d.player - 3.0f * a, -3.0f},
                                    {3.0f, d.player + 3.0f * a, 3.0f});
            },
            [](Vec3, float) { return false; });
    }
}

TEST_CASE("corridor compression with zero weight is the identity") {
    CorridorCompression d;
    d.weight = 0.0f;
    const Vec3 p{1.0f, 70.0f, 2.0f};
    CHECK(map(d, p) == p);
    CHECK(inverse(d, p).point == p);
}

TEST_CASE("corridor inverse reports points with no preimage") {
    CorridorCompression d;
    d.max_distance = 10.0f;
    // With the player at the entrance all of space maps into |y| < max_distance.
    CHECK_FALSE(inverse(d, {0.0f, 25.0f, 0.0f}).converged);
}

// --- Variant dispatch ----------------------------------------------------------------------

TEST_CASE("Deformer variant dispatches to the right overload") {
    Bend bend;
    bend.curvature = 0.2f;
    bend.length = 3.0f;
    bend.layers = 0b100u;
    const Deformer d = bend;
    const Vec3 p{0.1f, 2.0f, 0.3f};
    CHECK(evaluate(d, p).point == evaluate(bend, p).point);
    CHECK(inverse(d, p).point == inverse(bend, p).point);
    CHECK(layers(d) == 0b100u);
}
