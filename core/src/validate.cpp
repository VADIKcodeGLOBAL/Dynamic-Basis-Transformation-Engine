#include "dbte/validate.hpp"

#include <algorithm>
#include <cmath>
#include <initializer_list>
#include <limits>
#include <numbers>
#include <variant>

#include "dbte/basis.hpp"

namespace dbte {
namespace {

void add(ValidationReport& report, Problem problem) {
    if (!report.has(problem)) {
        report.problems.push_back(problem);
    }
}

void consider(ValidationReport& report, float det, Vec3 point) noexcept {
    if (det < report.min_determinant) {
        report.min_determinant = det;
        report.worst_point = point;
    }
}

void check_fold(ValidationReport& report, const ValidationOptions& options) {
    if (!(report.min_determinant > options.epsilon)) {
        add(report, Problem::FoldOver);
    }
}

void check_frame(ValidationReport& report, const Affine3& frame) {
    if (!is_rotation(frame.linear, 1e-3f)) {
        add(report, Problem::NonRigidFrame);
    }
}

/// Samples det_at over the box centre + axes * [-half, half] on an n^3 grid, skipping
/// points that `accept` rejects, then refines the smallest value by a local search.
template <typename DetFn, typename AcceptFn>
void sample_box(ValidationReport& report, Vec3 centre, const Mat3& axes, Vec3 half,
                int samples_per_axis, DetFn&& det_at, AcceptFn&& accept) {
    const int n = std::max(samples_per_axis, 2);
    const float steps = static_cast<float>(n - 1);
    const auto world = [&](Vec3 local) { return centre + axes * local; };
    const auto inside_box = [&](Vec3 local) {
        const Vec3 a = abs(local);
        return a.x <= half.x && a.y <= half.y && a.z <= half.z;
    };

    float best = std::numeric_limits<float>::infinity();
    Vec3 best_local{};
    for (int ix = 0; ix < n; ++ix) {
        for (int iy = 0; iy < n; ++iy) {
            for (int iz = 0; iz < n; ++iz) {
                const Vec3 t{static_cast<float>(ix) / steps, static_cast<float>(iy) / steps,
                             static_cast<float>(iz) / steps};
                const Vec3 local = hadamard(t * 2.0f - Vec3{1.0f, 1.0f, 1.0f}, half);
                const Vec3 p = world(local);
                if (!accept(p)) {
                    continue;
                }
                const float det = det_at(p);
                if (det < best) {
                    best = det;
                    best_local = local;
                }
            }
        }
    }
    if (!(best < std::numeric_limits<float>::infinity())) {
        return;  // Nothing to sample.
    }

    // Coordinate search around the best sample with a shrinking step.
    Vec3 step = half / steps;
    for (int round = 0; round < 40; ++round) {
        bool moved = false;
        for (int axis = 0; axis < 3; ++axis) {
            for (const float direction : {-1.0f, 1.0f}) {
                const Vec3 offset{axis == 0 ? step.x : 0.0f, axis == 1 ? step.y : 0.0f,
                                  axis == 2 ? step.z : 0.0f};
                const Vec3 local = best_local + offset * direction;
                if (!inside_box(local) || !accept(world(local))) {
                    continue;
                }
                const float det = det_at(world(local));
                if (det < best) {
                    best = det;
                    best_local = local;
                    moved = true;
                }
            }
        }
        if (!moved) {
            step *= 0.5f;
        }
    }
    consider(report, best, world(best_local));
}

ValidationReport check(const GlobalBasis& d, const ValidationOptions& options) {
    ValidationReport report;
    const Mat3 j = Mat3::identity() * (1.0f - d.weight) + d.basis * d.weight;
    consider(report, determinant(j), d.origin);
    check_fold(report, options);
    return report;
}

ValidationReport check(const AffineVolume& d, const ValidationOptions& options) {
    ValidationReport report;
    const bool box = d.shape == VolumeShape::Box;
    if (d.inner < 0.0f || !(d.outer > d.inner) ||
        (box && (d.half_extents.x < 0.0f || d.half_extents.y < 0.0f || d.half_extents.z < 0.0f))) {
        add(report, Problem::InvalidRange);
    }
    if (box && !(std::abs(length(d.orientation) - 1.0f) <= 1e-3f)) {
        add(report, Problem::NonRigidFrame);
    }

    // Inside the volume the mapping is affine, so det J is constant there.
    const Mat3 inner_map = Mat3::identity() + (d.basis - Mat3::identity()) * d.weight;
    consider(report, determinant(inner_map), d.centre);

    // The soft edge can fold even when the inside does not: sample it.
    if (d.outer > d.inner) {
        const Vec3 reach{d.outer, d.outer, d.outer};
        sample_box(
            report, d.centre, box ? to_matrix(d.orientation) : Mat3::identity(),
            box ? d.half_extents + reach : reach, options.samples_per_axis,
            [&](Vec3 p) { return determinant(evaluate(d, p).jacobian); },
            [&](Vec3 p) {
                const float dist = volume_distance(d, p);
                return dist > d.inner && dist < d.outer;
            });
    }
    check_fold(report, options);
    return report;
}

ValidationReport check(const Bend& d, const ValidationOptions& options) {
    ValidationReport report;
    check_frame(report, d.frame);
    if (!(d.length > 0.0f)) {
        add(report, Problem::InvalidRange);
    }
    const float k = d.weight * d.curvature;
    if (std::abs(k * d.length) > std::numbers::pi_v<float> + 1e-5f) {
        add(report, Problem::BendAngleTooLarge);
    }
    // In the bent segment det J = 1 - k z (local z); the worst point of the region of
    // interest lies on the side towards the centre of the bend.
    const float z = std::copysign(std::abs(d.half_extents.z), k);
    consider(report, 1.0f - k * z,
             transform_point(d.frame, {0.0f, 0.5f * std::max(d.length, 0.0f), z}));
    check_fold(report, options);
    return report;
}

ValidationReport check(const Twist& d, const ValidationOptions& /*options*/) {
    ValidationReport report;
    check_frame(report, d.frame);
    if (!(d.length > 0.0f)) {
        add(report, Problem::InvalidRange);
    }
    consider(report, 1.0f, d.frame.translation);  // A twist preserves volume: det J = 1.
    return report;
}

ValidationReport check(const Taper& d, const ValidationOptions& options) {
    ValidationReport report;
    check_frame(report, d.frame);
    if (!(d.length > 0.0f)) {
        add(report, Problem::InvalidRange);
    }
    // det J = fx(y) fz(y) with f = 1 + a smoothstep(y / length); sample along the segment.
    const float ax = d.weight * (d.scale_x - 1.0f);
    const float az = d.weight * (d.scale_z - 1.0f);
    constexpr int kSteps = 64;
    for (int s = 0; s <= kSteps; ++s) {
        const float t = static_cast<float>(s) / static_cast<float>(kSteps);
        const float h = smoothstep01(t);
        consider(report, (1.0f + ax * h) * (1.0f + az * h),
                 transform_point(d.frame, {0.0f, t * d.length, 0.0f}));
    }
    check_fold(report, options);
    return report;
}

ValidationReport check(const CorridorCompression& d, const ValidationOptions& /*options*/) {
    ValidationReport report;
    check_frame(report, d.frame);
    if (d.weight > 0.0f && !(d.max_distance > 0.0f)) {
        add(report, Problem::InvalidRange);
    }
    // det J = sech^2(...) > 0 everywhere: the compression is strictly monotonic and never
    // folds. Report the smallest value over the region of interest for information.
    if (d.weight > 0.0f && d.max_distance > 0.0f) {
        const float a = d.max_distance / d.weight;
        const float ey = std::abs(d.half_extents.y);
        const float y = std::abs(ey - d.player) >= std::abs(-ey - d.player) ? ey : -ey;
        const float c = std::cosh((y - d.player) / a);
        consider(report, 1.0f / (c * c), transform_point(d.frame, {0.0f, y, 0.0f}));
    }
    return report;
}

}  // namespace

std::string_view describe(Problem problem) noexcept {
    switch (problem) {
        case Problem::FoldOver:
            return "Space folds over itself (det J <= 0). Widen the soft edge, weaken the "
                   "deformation or shrink the region.";
        case Problem::NonRigidFrame:
            return "The frame or orientation must be a rotation: orthonormal axes, no scale "
                   "and no mirroring.";
        case Problem::InvalidRange:
            return "A length, distance or edge width is out of range: lengths and distances "
                   "must be positive and outer must be larger than inner.";
        case Problem::BendAngleTooLarge:
            return "The bend angle is above 180 degrees. Use a smaller curvature or a shorter "
                   "bent segment.";
    }
    return "Unknown problem.";
}

bool ValidationReport::has(Problem problem) const noexcept {
    return std::find(problems.begin(), problems.end(), problem) != problems.end();
}

ValidationReport validate(const Deformer& deformer, const ValidationOptions& options) {
    return std::visit([&](const auto& d) { return check(d, options); }, deformer);
}

ValidationReport validate_region(const DeformationField& field, Vec3 box_min, Vec3 box_max,
                                 LayerMask mask, const ValidationOptions& options) {
    ValidationReport report;
    sample_box(
        report, (box_min + box_max) * 0.5f, Mat3::identity(), abs(box_max - box_min) * 0.5f,
        options.samples_per_axis, [&](Vec3 p) { return determinant(field.jacobian(p, mask)); },
        [](Vec3 /*p*/) { return true; });
    check_fold(report, options);
    return report;
}

}  // namespace dbte
