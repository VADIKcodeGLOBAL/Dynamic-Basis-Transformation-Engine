#pragma once

// DeformationField: a chain of deformers applied in order, Phi = Phi_n o ... o Phi_1.

#include <utility>
#include <vector>

#include "dbte/deformers.hpp"

namespace dbte {

class DeformationField {
public:
    DeformationField() = default;
    explicit DeformationField(std::vector<Deformer> chain) noexcept
        : deformers_(std::move(chain)) {}

    /// The deformers, applied in index order.
    [[nodiscard]] std::vector<Deformer>& deformers() noexcept { return deformers_; }
    [[nodiscard]] const std::vector<Deformer>& deformers() const noexcept { return deformers_; }

    /// Maps a logical-space point to visible space and returns the Jacobian of the whole
    /// chain at p: J = J_n(p_{n-1}) ... J_1(p_0). Only deformers that share a layer with
    /// `mask` take part.
    [[nodiscard]] Evaluation evaluate(Vec3 p, LayerMask mask = kAllLayers) const noexcept;

    [[nodiscard]] Vec3 map(Vec3 p, LayerMask mask = kAllLayers) const noexcept;
    [[nodiscard]] Mat3 jacobian(Vec3 p, LayerMask mask = kAllLayers) const noexcept;

    /// Maps a tangent vector at p: J t.
    [[nodiscard]] Vec3 map_tangent(Vec3 p, Vec3 tangent,
                                   LayerMask mask = kAllLayers) const noexcept;

    /// Maps a normal at p with the cofactor matrix and normalizes it. The result stays
    /// perpendicular to every mapped tangent. Returns the input normal if the mapped one
    /// degenerates to zero length.
    [[nodiscard]] Vec3 map_normal(Vec3 p, Vec3 normal, LayerMask mask = kAllLayers) const noexcept;

    /// Brings a visible-space point back to logical space by inverting the deformers in
    /// reverse order: in closed form where one exists, with Newton's method otherwise.
    [[nodiscard]] InverseResult inverse(Vec3 q, LayerMask mask = kAllLayers,
                                        const InverseOptions& options = {}) const noexcept;

    /// Rotation part of the local basis J(p), from the polar decomposition. A camera at p
    /// can take its orientation from it so that "up" turns with the deformed space.
    [[nodiscard]] Mat3 frame(Vec3 p, LayerMask mask = kAllLayers) const noexcept;

private:
    std::vector<Deformer> deformers_;
};

}  // namespace dbte
