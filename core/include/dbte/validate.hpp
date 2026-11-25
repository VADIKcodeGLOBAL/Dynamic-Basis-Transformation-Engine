#pragma once

// Validator: finds where a deformation turns space inside out (det J <= epsilon) and
// checks parameter ranges. Meant for editor tools and tests, not for per-frame use.

#include <cstdint>
#include <string_view>
#include <vector>

#include "dbte/deformers.hpp"
#include "dbte/field.hpp"

namespace dbte {

enum class Problem : std::uint8_t {
    FoldOver,           ///< det J drops to epsilon or below: space folds over itself.
    NonRigidFrame,      ///< A frame or orientation is not a rotation.
    InvalidRange,       ///< A length, distance or edge width is out of range.
    BendAngleTooLarge,  ///< The bend angle |curvature * length| is above pi.
};

/// A short English explanation of the problem and how to fix it.
[[nodiscard]] std::string_view describe(Problem problem) noexcept;

struct ValidationOptions {
    float epsilon = 1e-3f;      ///< det J at or below this counts as a fold.
    int samples_per_axis = 24;  ///< Grid resolution of sampled checks.
};

struct ValidationReport {
    float min_determinant = 1.0f;  ///< Smallest det J found.
    Vec3 worst_point{};            ///< Logical-space point where it was found.
    std::vector<Problem> problems;

    [[nodiscard]] bool ok() const noexcept { return problems.empty(); }
    [[nodiscard]] bool has(Problem problem) const noexcept;
};

/// Checks one deformer. Uses the closed-form minimum of det J where there is one; for an
/// affine volume it samples the soft edge on a grid and refines the worst sample. For
/// a bend the check covers the deformer's half_extents (its region of interest).
[[nodiscard]] ValidationReport validate(const Deformer& deformer,
                                        const ValidationOptions& options = {});

/// Samples det J of the whole chain over the box [box_min, box_max].
[[nodiscard]] ValidationReport validate_region(const DeformationField& field, Vec3 box_min,
                                               Vec3 box_max, LayerMask mask = kAllLayers,
                                               const ValidationOptions& options = {});

}  // namespace dbte
