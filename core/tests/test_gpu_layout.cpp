#include <doctest/doctest.h>

#include <array>
#include <vector>

#include "test_util.hpp"

using namespace dbte;
using dbte::test::random_frame;
using dbte::test::Rng;

namespace {

namespace off = gpu::offset;

std::vector<Deformer> random_deformers(Rng& rng) {
    const auto random_layers = [&] { return static_cast<LayerMask>(rng.uniform() * 16777215.0f); };
    GlobalBasis global{rng.vec3(-5.0f, 5.0f), rng.basis(), rng.uniform(0.0f, 1.0f),
                       random_layers()};
    AffineVolume sphere;
    sphere.centre = rng.vec3(-5.0f, 5.0f);
    sphere.basis = rng.basis();
    sphere.inner = rng.uniform(0.0f, 2.0f);
    sphere.outer = rng.uniform(2.0f, 5.0f);
    sphere.weight = rng.uniform(0.0f, 1.0f);
    sphere.layers = random_layers();
    AffineVolume box = sphere;
    box.shape = VolumeShape::Box;
    box.orientation = rng.rotation();
    box.half_extents = rng.vec3(0.1f, 3.0f);
    const Bend bend{random_frame(rng),    rng.uniform(-1.0f, 1.0f), rng.uniform(0.1f, 5.0f),
                    rng.vec3(0.0f, 4.0f), rng.uniform(0.0f, 1.0f),  random_layers()};
    const Twist twist{random_frame(rng),    rng.uniform(0.1f, 5.0f), rng.uniform(-3.0f, 3.0f),
                      rng.vec3(0.0f, 4.0f), rng.uniform(0.0f, 1.0f), random_layers()};
    const Taper taper{random_frame(rng),       rng.uniform(0.1f, 5.0f), rng.uniform(0.1f, 3.0f),
                      rng.uniform(0.1f, 3.0f), rng.vec3(0.0f, 4.0f),    rng.uniform(0.0f, 1.0f),
                      random_layers()};
    const CorridorCompression corridor{random_frame(rng),         rng.uniform(1.0f, 30.0f),
                                       rng.uniform(-9.0f, 90.0f), rng.vec3(0.0f, 50.0f),
                                       rng.uniform(0.0f, 1.0f),   random_layers()};
    return {global, sphere, box, bend, twist, taper, corridor};
}

}  // namespace

TEST_CASE("pack and unpack give back the same parameters") {
    Rng rng(300);
    for (int n = 0; n < 100; ++n) {
        for (const Deformer& d : random_deformers(rng)) {
            std::array<float, gpu::kFloatsPerDeformer> block{};
            gpu::pack(d, block);
            const std::optional<Deformer> back = gpu::unpack(block);
            REQUIRE(back.has_value());
            CHECK(*back == d);
            CHECK(gpu::type_of(*back) == gpu::type_of(d));
        }
    }
}

TEST_CASE("block layout matches the documented contract") {
    AffineVolume d;
    d.shape = VolumeShape::Box;
    d.centre = {1.0f, 2.0f, 3.0f};
    d.basis = basis_from_vectors({4.0f, 5.0f, 6.0f}, {7.0f, 8.0f, 9.0f}, {10.0f, 11.0f, 12.0f});
    d.half_extents = {13.0f, 14.0f, 15.0f};
    d.orientation = {0.5f, 0.5f, 0.5f, 0.5f};
    d.inner = 16.0f;
    d.outer = 17.0f;
    d.weight = 0.75f;
    d.layers = 0b1011u;

    std::array<float, gpu::kFloatsPerDeformer> b{};
    gpu::pack(d, b);
    CHECK(b[off::kType] == 3.0f);  // BoxVolume
    CHECK(b[off::kLayers] == 11.0f);
    CHECK(b[off::kWeight] == 0.75f);
    CHECK(b[off::kParam0] == 16.0f);  // inner
    CHECK(b[off::kParam1] == 17.0f);  // outer
    CHECK(b[off::kOrigin + 0] == 1.0f);
    CHECK(b[off::kOrigin + 2] == 3.0f);
    CHECK(b[off::kColumnI + 0] == 4.0f);
    CHECK(b[off::kColumnJ + 1] == 8.0f);
    CHECK(b[off::kColumnK + 2] == 12.0f);
    CHECK(b[off::kExtents + 0] == 13.0f);
    CHECK(b[off::kExtents + 2] == 15.0f);
    CHECK(b[off::kParam2] == 0.5f);  // orientation x, y, z, w
    CHECK(b[off::kParam5] == 0.5f);
    // Texel t, channel c is float 4 t + c.
    CHECK(off::kColumnJ == std::size_t{4 * 3});
    CHECK(off::kParam5 == std::size_t{4 * 5 + 3});
    CHECK(gpu::kFloatsPerDeformer == std::size_t{24});
    CHECK(gpu::kLayoutVersion == 1);
}

TEST_CASE("frame-based deformers store the frame rotation as the matrix") {
    Bend d;
    d.frame.linear = to_matrix(quat_from_axis_angle({0.0f, 1.0f, 0.0f}, 0.3f));
    d.frame.translation = {1.0f, 2.0f, 3.0f};
    d.curvature = 0.25f;
    d.length = 6.0f;
    std::array<float, gpu::kFloatsPerDeformer> b{};
    gpu::pack(d, b);
    CHECK(b[off::kType] == 4.0f);
    CHECK(b[off::kOrigin + 1] == 2.0f);
    CHECK(b[off::kColumnI + 2] == d.frame.linear.i.z);
    CHECK(b[off::kParam0] == 0.25f);
    CHECK(b[off::kParam1] == 6.0f);
    CHECK(b[off::kParam2] == 0.0f);  // unused
}

TEST_CASE("layers keep 24 bits") {
    GlobalBasis d;
    d.layers = 0xFFFF'FFFFu;
    std::array<float, gpu::kFloatsPerDeformer> b{};
    gpu::pack(d, b);
    const std::optional<Deformer> back = gpu::unpack(b);
    REQUIRE(back.has_value());
    CHECK(layers(*back) == kAllLayers);
}

TEST_CASE("unpack rejects unknown types") {
    std::array<float, gpu::kFloatsPerDeformer> b{};
    CHECK_FALSE(gpu::unpack(b).has_value());  // type 0: none
    b[off::kType] = 99.0f;
    CHECK_FALSE(gpu::unpack(b).has_value());
    b[off::kType] = 2.5f;
    CHECK_FALSE(gpu::unpack(b).has_value());
    b[off::kType] = -1.0f;
    CHECK_FALSE(gpu::unpack(b).has_value());
}

TEST_CASE("packing a field writes the blocks in order and caps the count") {
    Rng rng(301);
    DeformationField field;
    for (int n = 0; n < 10; ++n) {
        for (const Deformer& d : random_deformers(rng)) {
            field.deformers().push_back(d);
        }
    }
    REQUIRE(field.deformers().size() == std::size_t{70});
    const std::vector<float> data = gpu::pack(field);
    CHECK(data.size() == gpu::kMaxDeformers * gpu::kFloatsPerDeformer);
    for (std::size_t n = 0; n < gpu::kMaxDeformers; ++n) {
        const std::span<const float, gpu::kFloatsPerDeformer> block(
            data.data() + n * gpu::kFloatsPerDeformer, gpu::kFloatsPerDeformer);
        const std::optional<Deformer> back = gpu::unpack(block);
        REQUIRE(back.has_value());
        CHECK(*back == field.deformers()[n]);
    }
}
