#include <doctest/doctest.h>

#include <cmath>

#include "test_util.hpp"

using namespace dbte;
using dbte::test::kPi;

namespace {

AffineVolume sheared_sphere(float inner, float outer) {
    AffineVolume d;
    d.basis = basis_from_vectors({1.0f, 0.0f, 0.0f}, {0.6f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f});
    d.inner = inner;
    d.outer = outer;
    return d;
}

}  // namespace

TEST_CASE("a strong shear with a thin edge folds, a wide edge does not") {
    // det J = 1 - 0.3 d s'(d) at 45 degrees in the xy plane; with the edge from 4 to 6 its
    // minimum is 1 - 0.3 * 3.787 = -0.136 (see docs/math.md, affine volume).
    const ValidationReport thin = validate(sheared_sphere(4.0f, 6.0f));
    CHECK(thin.has(Problem::FoldOver));
    CHECK(thin.min_determinant == doctest::Approx(-0.136f).epsilon(0.05));
    CHECK(volume_distance(sheared_sphere(4.0f, 6.0f), thin.worst_point) > 4.0f);
    CHECK(volume_distance(sheared_sphere(4.0f, 6.0f), thin.worst_point) < 6.0f);

    // With the edge from 1 to 6 the minimum is 1 - 0.3 * 1.162 = 0.651.
    const ValidationReport wide = validate(sheared_sphere(1.0f, 6.0f));
    CHECK(wide.ok());
    CHECK(wide.min_determinant == doctest::Approx(0.651f).epsilon(0.02));
}

TEST_CASE("the validator's minimum matches dense sampling") {
    const AffineVolume d = sheared_sphere(2.0f, 5.0f);
    const ValidationReport report = validate(d);
    float dense = 1.0f;
    for (int a = 0; a < 720; ++a) {
        for (int r = 0; r <= 300; ++r) {
            const float angle = static_cast<float>(a) * (kPi / 360.0f);
            const float radius = d.inner + (d.outer - d.inner) * static_cast<float>(r) / 300.0f;
            const Vec3 p{radius * std::cos(angle), radius * std::sin(angle), 0.0f};
            dense = std::min(dense, determinant(jacobian(d, p)));
        }
    }
    CHECK(report.min_determinant == doctest::Approx(dense).epsilon(0.01));
}

TEST_CASE("inside a volume the basis itself must keep its orientation") {
    AffineVolume d;
    d.basis = Mat3::diagonal({1.0f, 1.0f, -1.0f});
    CHECK(validate(d).has(Problem::FoldOver));
    d.inner = 3.0f;
    d.outer = 2.0f;
    CHECK(validate(d).has(Problem::InvalidRange));
}

TEST_CASE("box volume: range and orientation checks") {
    AffineVolume d;
    d.shape = VolumeShape::Box;
    d.half_extents = {1.0f, 1.0f, 1.0f};
    d.inner = 0.0f;
    d.outer = 3.0f;
    CHECK(validate(d).ok());
    d.orientation = {0.0f, 0.0f, 0.0f, 2.0f};
    CHECK(validate(d).has(Problem::NonRigidFrame));
    d.orientation = {};
    d.half_extents = {1.0f, -1.0f, 1.0f};
    CHECK(validate(d).has(Problem::InvalidRange));
}

TEST_CASE("bend: the region of interest must stay inside the bend radius") {
    Bend d;
    d.curvature = 0.25f;  // radius 4
    d.length = 2.0f;
    d.half_extents = {1.0f, 3.0f, 3.0f};
    const ValidationReport ok = validate(d);
    CHECK(ok.ok());
    CHECK(ok.min_determinant == doctest::Approx(0.25f));
    d.half_extents.z = 5.0f;
    const ValidationReport folds = validate(d);
    CHECK(folds.has(Problem::FoldOver));
    CHECK(folds.worst_point.z == doctest::Approx(5.0f));
    // Negative curvature bends the other way, so the worst side flips too.
    d.curvature = -0.25f;
    CHECK(validate(d).worst_point.z == doctest::Approx(-5.0f));
}

TEST_CASE("bend: angle and frame checks") {
    Bend d;
    d.curvature = 1.0f;
    d.length = 4.0f;  // 229 degrees
    CHECK(validate(d).has(Problem::BendAngleTooLarge));
    d.length = 3.0f;
    CHECK_FALSE(validate(d).has(Problem::BendAngleTooLarge));
    d.frame.linear = Mat3::diagonal({1.0f, 2.0f, 1.0f});
    CHECK(validate(d).has(Problem::NonRigidFrame));
    d.frame.linear = Mat3::diagonal({1.0f, 1.0f, -1.0f});
    CHECK(validate(d).has(Problem::NonRigidFrame));
}

TEST_CASE("twist never folds, taper folds when a scale reaches zero") {
    Twist twist;
    twist.length = 1.0f;
    twist.angle = 10.0f;
    CHECK(validate(twist).ok());
    twist.length = 0.0f;
    CHECK(validate(twist).has(Problem::InvalidRange));

    Taper taper;
    taper.length = 2.0f;
    taper.scale_x = 0.5f;
    taper.scale_z = 0.5f;
    const ValidationReport ok = validate(taper);
    CHECK(ok.ok());
    CHECK(ok.min_determinant == doctest::Approx(0.25f));
    taper.scale_x = -0.5f;
    CHECK(validate(taper).has(Problem::FoldOver));
    taper.weight = 0.5f;  // Blends the scale to 0.25: valid again.
    CHECK(validate(taper).ok());
}

TEST_CASE("corridor compression never folds") {
    CorridorCompression d;
    d.max_distance = 2.0f;
    d.half_extents = {1.0f, 500.0f, 1.0f};
    const ValidationReport report = validate(d);
    CHECK(report.ok());
    CHECK(report.min_determinant >= 0.0f);
    d.max_distance = 0.0f;
    CHECK(validate(d).has(Problem::InvalidRange));
}

TEST_CASE("global basis: det of the blended basis") {
    GlobalBasis d;
    d.basis = to_matrix(quat_from_axis_angle({0.0f, 0.0f, 1.0f}, kPi));
    CHECK(validate(d).ok());
    d.weight = 0.5f;  // Linear blend of I and a half turn flattens space.
    CHECK(validate(d).has(Problem::FoldOver));
}

TEST_CASE("validate_region finds folds of the whole chain") {
    Bend bend;
    bend.curvature = 0.5f;  // radius 2
    bend.length = 2.0f;
    const DeformationField field({bend});
    const ValidationReport inside =
        validate_region(field, {-1.0f, -1.0f, -1.0f}, {1.0f, 3.0f, 1.0f});
    CHECK(inside.ok());
    CHECK(inside.min_determinant == doctest::Approx(0.5f).epsilon(0.01));
    const ValidationReport beyond =
        validate_region(field, {-1.0f, -1.0f, -1.0f}, {1.0f, 3.0f, 3.0f});
    CHECK(beyond.has(Problem::FoldOver));
    CHECK(beyond.worst_point.z > 2.0f);
}

TEST_CASE("every problem has a description") {
    for (const Problem p : {Problem::FoldOver, Problem::NonRigidFrame, Problem::InvalidRange,
                            Problem::BendAngleTooLarge}) {
        CHECK_FALSE(describe(p).empty());
    }
}
