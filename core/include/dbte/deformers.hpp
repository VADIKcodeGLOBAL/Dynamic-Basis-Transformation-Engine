#pragma once

// Deformers: closed-form mappings from logical space L to visible space W, each with an
// analytic Jacobian and an inverse. The formulas are specified in docs/math.md; the
// shaders implement the same formulas, so change both together.

#include <cstdint>
#include <variant>

#include "dbte/math.hpp"

namespace dbte {

/// Bit mask of layers. A deformer applies to geometry whose layer mask shares a bit with
/// the deformer's. 24 bits, so masks survive the trip to the GPU as exact floats.
using LayerMask = std::uint32_t;
inline constexpr LayerMask kAllLayers = 0x00FF'FFFFu;

/// One basis for the whole space: Phi(p) = origin + B' (p - origin), B' = I + w (B - I).
struct GlobalBasis {
    Vec3 origin{};
    Mat3 basis{};  ///< Columns are the basis vectors i, j, k.
    float weight = 1.0f;
    LayerMask layers = kAllLayers;

    constexpr bool operator==(const GlobalBasis&) const = default;
};

enum class VolumeShape : std::uint8_t { Sphere, Box };

/// Affine deformation inside a sphere or box with a soft edge:
/// Phi(p) = p + w(p) (B - I) (p - centre), where w = weight inside the volume and falls
/// smoothly to 0 between the distances `inner` and `outer`.
struct AffineVolume {
    Vec3 centre{};
    Mat3 basis{};  ///< Basis inside the volume, in logical-space coordinates.
    VolumeShape shape = VolumeShape::Sphere;
    Vec3 half_extents{};  ///< Box: half size of the full-effect box in its own frame.
    Quat orientation{};   ///< Box: rotation of the box frame. Must be a unit quaternion.
    /// Sphere: distance from the centre. Box: distance from the box surface (0 inside).
    float inner = 1.0f;  ///< Full effect up to this distance.
    float outer = 2.0f;  ///< No effect beyond this distance.
    float weight = 1.0f;
    LayerMask layers = kAllLayers;

    constexpr bool operator==(const AffineVolume&) const = default;
};

/// Bend after Barr (1984). Local frame: y runs along the bent segment, which starts at the
/// origin and has length `length`; the segment curves towards +z (for positive
/// curvature) around an axis parallel to x. Before the segment space is unchanged;
/// beyond it space follows rigidly.
struct Bend {
    Affine3 frame{};         ///< Local to logical space. The linear part must be a rotation.
    float curvature = 0.0f;  ///< 1 / radius. Negative values bend towards -z.
    float length = 1.0f;     ///< Length of the bent segment along local y.
    Vec3 half_extents{};     ///< Region of interest around the origin, for the validator.
    float weight = 1.0f;     ///< Scales the curvature: 0 is straight, 1 is fully bent.
    LayerMask layers = kAllLayers;

    constexpr bool operator==(const Bend&) const = default;
};

/// Twist after Barr (1984): cross-sections rotate about local y by an angle that grows
/// smoothly from 0 at y = 0 to `angle` at y = length, and stays constant beyond.
struct Twist {
    Affine3 frame{};
    float length = 1.0f;
    float angle = 0.0f;  ///< Radians, counter-clockwise about +y.
    Vec3 half_extents{};
    float weight = 1.0f;  ///< Scales the angle.
    LayerMask layers = kAllLayers;

    constexpr bool operator==(const Twist&) const = default;
};

/// Taper after Barr (1984): cross-sections scale in local x and z by factors that change
/// smoothly from 1 at y = 0 to scale_x and scale_z at y = length.
struct Taper {
    Affine3 frame{};
    float length = 1.0f;
    float scale_x = 1.0f;
    float scale_z = 1.0f;
    Vec3 half_extents{};
    float weight = 1.0f;  ///< Blends the scale factors towards 1.
    LayerMask layers = kAllLayers;

    constexpr bool operator==(const Taper&) const = default;
};

/// Compression of distance along local y relative to the player: around the player space
/// is almost unchanged, while far points never appear further than `max_distance`. The
/// origin (the corridor entrance) stays in place:
/// Y = a (tanh((y - player) / a) + tanh(player / a)), a = max_distance / weight.
struct CorridorCompression {
    Affine3 frame{};
    float max_distance = 10.0f;
    float player = 0.0f;  ///< The player's local y coordinate, updated every frame.
    Vec3 half_extents{};
    float weight = 1.0f;  ///< 0 turns the compression off.
    LayerMask layers = kAllLayers;

    constexpr bool operator==(const CorridorCompression&) const = default;
};

using Deformer = std::variant<GlobalBasis, AffineVolume, Bend, Twist, Taper, CorridorCompression>;

/// A mapped point together with the Jacobian of the mapping at the original point.
struct Evaluation {
    Vec3 point{};
    Mat3 jacobian{};
};

struct InverseOptions {
    /// Newton iterations per deformer, used only where no closed form exists (the soft
    /// edge of an affine volume).
    int max_iterations = 8;
    /// Accepted residual |Phi(p) - q|, relative to max(1, |q|).
    float tolerance = 1e-5f;
};

struct InverseResult {
    Vec3 point{};
    bool converged = false;
    int iterations = 0;  ///< Newton iterations used, 0 for closed-form inverses.
};

[[nodiscard]] Evaluation evaluate(const GlobalBasis& d, Vec3 p) noexcept;
[[nodiscard]] Evaluation evaluate(const AffineVolume& d, Vec3 p) noexcept;
[[nodiscard]] Evaluation evaluate(const Bend& d, Vec3 p) noexcept;
[[nodiscard]] Evaluation evaluate(const Twist& d, Vec3 p) noexcept;
[[nodiscard]] Evaluation evaluate(const Taper& d, Vec3 p) noexcept;
[[nodiscard]] Evaluation evaluate(const CorridorCompression& d, Vec3 p) noexcept;
[[nodiscard]] Evaluation evaluate(const Deformer& d, Vec3 p) noexcept;

[[nodiscard]] InverseResult inverse(const GlobalBasis& d, Vec3 q,
                                    const InverseOptions& options = {}) noexcept;
[[nodiscard]] InverseResult inverse(const AffineVolume& d, Vec3 q,
                                    const InverseOptions& options = {}) noexcept;
[[nodiscard]] InverseResult inverse(const Bend& d, Vec3 q,
                                    const InverseOptions& options = {}) noexcept;
[[nodiscard]] InverseResult inverse(const Twist& d, Vec3 q,
                                    const InverseOptions& options = {}) noexcept;
[[nodiscard]] InverseResult inverse(const Taper& d, Vec3 q,
                                    const InverseOptions& options = {}) noexcept;
[[nodiscard]] InverseResult inverse(const CorridorCompression& d, Vec3 q,
                                    const InverseOptions& options = {}) noexcept;
[[nodiscard]] InverseResult inverse(const Deformer& d, Vec3 q,
                                    const InverseOptions& options = {}) noexcept;

/// The deformer's layer mask.
[[nodiscard]] LayerMask layers(const Deformer& d) noexcept;

/// Distance used by the soft edge of an affine volume: from the centre for a sphere, from
/// the box surface for a box (0 inside the box).
[[nodiscard]] float volume_distance(const AffineVolume& d, Vec3 p) noexcept;

/// Mapped point only.
template <typename D>
[[nodiscard]] Vec3 map(const D& d, Vec3 p) noexcept {
    return evaluate(d, p).point;
}

/// Jacobian only.
template <typename D>
[[nodiscard]] Mat3 jacobian(const D& d, Vec3 p) noexcept {
    return evaluate(d, p).jacobian;
}

}  // namespace dbte
