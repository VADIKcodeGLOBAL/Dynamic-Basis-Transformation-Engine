#include "dbte/field.hpp"

#include "dbte/basis.hpp"

namespace dbte {

Evaluation DeformationField::evaluate(Vec3 p, LayerMask mask) const noexcept {
    Evaluation acc{p, Mat3::identity()};
    for (const Deformer& d : deformers_) {
        if ((layers(d) & mask) == 0u) {
            continue;
        }
        const Evaluation e = dbte::evaluate(d, acc.point);
        acc.point = e.point;
        acc.jacobian = e.jacobian * acc.jacobian;
    }
    return acc;
}

Vec3 DeformationField::map(Vec3 p, LayerMask mask) const noexcept {
    return evaluate(p, mask).point;
}

Mat3 DeformationField::jacobian(Vec3 p, LayerMask mask) const noexcept {
    return evaluate(p, mask).jacobian;
}

Vec3 DeformationField::map_tangent(Vec3 p, Vec3 tangent, LayerMask mask) const noexcept {
    return jacobian(p, mask) * tangent;
}

Vec3 DeformationField::map_normal(Vec3 p, Vec3 normal, LayerMask mask) const noexcept {
    return normalized_or(cofactor(jacobian(p, mask)) * normal, normal);
}

InverseResult DeformationField::inverse(Vec3 q, LayerMask mask,
                                        const InverseOptions& options) const noexcept {
    InverseResult result{q, true, 0};
    for (auto it = deformers_.rbegin(); it != deformers_.rend(); ++it) {
        if ((layers(*it) & mask) == 0u) {
            continue;
        }
        const InverseResult step = dbte::inverse(*it, result.point, options);
        result.point = step.point;
        result.converged = result.converged && step.converged;
        result.iterations += step.iterations;
    }
    return result;
}

Mat3 DeformationField::frame(Vec3 p, LayerMask mask) const noexcept {
    return nearest_rotation(jacobian(p, mask));
}

}  // namespace dbte
