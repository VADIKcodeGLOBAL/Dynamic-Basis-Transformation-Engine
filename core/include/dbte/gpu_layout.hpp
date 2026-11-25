#pragma once

// CPU-GPU data contract, layout version 1 (docs/ARCHITECTURE.md, section 6).
//
// Every deformer is packed into one block of 24 floats, which is 6 RGBA32F texels of the
// dbte_data texture. Shaders read the same blocks and evaluate the same formulas, so a
// change here needs a new layout version and matching shader changes.
//
//   texel | x           y           z           w
//   ------+-----------------------------------------------
//   0     | type        layers      weight      param 0
//   1     | origin.x    origin.y    origin.z    param 1
//   2     | matrix.i.x  matrix.i.y  matrix.i.z  param 2
//   3     | matrix.j.x  matrix.j.y  matrix.j.z  param 3
//   4     | matrix.k.x  matrix.k.y  matrix.k.z  param 4
//   5     | extents.x   extents.y   extents.z   param 5
//
// type and layers are integers stored as floats (exact below 2^24). For an affine volume
// "matrix" is its basis; for the other deformers except the global basis it is the
// rotation of the local frame. The per-type parameters are listed at DeformerType.

#include <cstddef>
#include <optional>
#include <span>
#include <vector>

#include "dbte/deformers.hpp"
#include "dbte/field.hpp"

namespace dbte::gpu {

inline constexpr int kLayoutVersion = 1;
inline constexpr std::size_t kTexelsPerDeformer = 6;
inline constexpr std::size_t kFloatsPerDeformer = 4 * kTexelsPerDeformer;
/// Deformers that one dbte_data texture holds in the demo.
inline constexpr std::size_t kMaxDeformers = 64;

/// Float offsets inside a block.
namespace offset {
inline constexpr std::size_t kType = 0;
inline constexpr std::size_t kLayers = 1;
inline constexpr std::size_t kWeight = 2;
inline constexpr std::size_t kParam0 = 3;
inline constexpr std::size_t kOrigin = 4;  // x, y, z
inline constexpr std::size_t kParam1 = 7;
inline constexpr std::size_t kColumnI = 8;  // x, y, z
inline constexpr std::size_t kParam2 = 11;
inline constexpr std::size_t kColumnJ = 12;  // x, y, z
inline constexpr std::size_t kParam3 = 15;
inline constexpr std::size_t kColumnK = 16;  // x, y, z
inline constexpr std::size_t kParam4 = 19;
inline constexpr std::size_t kExtents = 20;  // x, y, z
inline constexpr std::size_t kParam5 = 23;
}  // namespace offset

/// Value of the type field and the meaning of the per-type fields.
enum class DeformerType : int {
    None = 0,
    /// origin, matrix = basis.
    GlobalBasis = 1,
    /// origin = centre, matrix = basis, param 0 = inner, param 1 = outer,
    /// params 2-5 = orientation quaternion (x, y, z, w), extents = half extents.
    SphereVolume = 2,
    BoxVolume = 3,
    /// origin, matrix = frame rotation, extents = region of interest, and:
    /// param 0 = curvature, param 1 = length.
    Bend = 4,
    /// param 0 = length, param 1 = angle.
    Twist = 5,
    /// param 0 = length, param 1 = scale x, param 2 = scale z.
    Taper = 6,
    /// param 0 = max distance, param 1 = player.
    CorridorCompression = 7,
};

[[nodiscard]] DeformerType type_of(const Deformer& deformer) noexcept;

using Block = std::span<float, kFloatsPerDeformer>;
using ConstBlock = std::span<const float, kFloatsPerDeformer>;

/// Writes one deformer into a block. Unused fields are zero. Layer bits above 24 are
/// dropped.
void pack(const Deformer& deformer, Block out) noexcept;

/// Reads a block back. Returns nothing for an unknown type.
[[nodiscard]] std::optional<Deformer> unpack(ConstBlock in) noexcept;

/// Packs the first min(n, kMaxDeformers) deformers of the field, in order: the contents
/// of the dbte_data texture row. dbte_count is the returned size / kFloatsPerDeformer.
[[nodiscard]] std::vector<float> pack(const DeformationField& field);

}  // namespace dbte::gpu
