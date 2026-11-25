#include "dbte/gpu_layout.hpp"

#include <algorithm>
#include <cmath>
#include <initializer_list>
#include <variant>

namespace dbte::gpu {

using namespace offset;

namespace {

class Writer {
public:
    explicit Writer(Block out) noexcept : out_(out) { std::fill(out_.begin(), out_.end(), 0.0f); }

    void header(DeformerType type, LayerMask layers, float weight) noexcept {
        out_[kType] = static_cast<float>(static_cast<int>(type));
        out_[kLayers] = static_cast<float>(layers & kAllLayers);
        out_[kWeight] = weight;
    }
    void vec3(std::size_t at, Vec3 v) noexcept {
        out_[at] = v.x;
        out_[at + 1] = v.y;
        out_[at + 2] = v.z;
    }
    void matrix(const Mat3& m) noexcept {
        vec3(kColumnI, m.i);
        vec3(kColumnJ, m.j);
        vec3(kColumnK, m.k);
    }
    void frame(const Affine3& f) noexcept {
        vec3(kOrigin, f.translation);
        matrix(f.linear);
    }
    void params(std::initializer_list<float> values) noexcept {
        constexpr std::size_t kSlots[] = {kParam0, kParam1, kParam2, kParam3, kParam4, kParam5};
        std::size_t n = 0;
        for (const float v : values) {
            out_[kSlots[n++]] = v;
        }
    }

private:
    Block out_;
};

class Reader {
public:
    explicit Reader(ConstBlock in) noexcept : in_(in) {}

    [[nodiscard]] float at(std::size_t i) const noexcept { return in_[i]; }
    [[nodiscard]] Vec3 vec3(std::size_t at) const noexcept {
        return {in_[at], in_[at + 1], in_[at + 2]};
    }
    [[nodiscard]] Mat3 matrix() const noexcept {
        return {vec3(kColumnI), vec3(kColumnJ), vec3(kColumnK)};
    }
    [[nodiscard]] Affine3 frame() const noexcept { return {matrix(), vec3(kOrigin)}; }
    [[nodiscard]] float weight() const noexcept { return in_[kWeight]; }
    [[nodiscard]] LayerMask layers() const noexcept {
        const float v = in_[kLayers];
        if (!(v >= 0.0f) || v > static_cast<float>(kAllLayers)) {
            return 0u;
        }
        return static_cast<LayerMask>(v);
    }

private:
    ConstBlock in_;
};

void write(Writer& w, const GlobalBasis& d) noexcept {
    w.header(DeformerType::GlobalBasis, d.layers, d.weight);
    w.vec3(kOrigin, d.origin);
    w.matrix(d.basis);
}

void write(Writer& w, const AffineVolume& d) noexcept {
    w.header(d.shape == VolumeShape::Box ? DeformerType::BoxVolume : DeformerType::SphereVolume,
             d.layers, d.weight);
    w.vec3(kOrigin, d.centre);
    w.matrix(d.basis);
    w.vec3(kExtents, d.half_extents);
    w.params(
        {d.inner, d.outer, d.orientation.x, d.orientation.y, d.orientation.z, d.orientation.w});
}

void write(Writer& w, const Bend& d) noexcept {
    w.header(DeformerType::Bend, d.layers, d.weight);
    w.frame(d.frame);
    w.vec3(kExtents, d.half_extents);
    w.params({d.curvature, d.length});
}

void write(Writer& w, const Twist& d) noexcept {
    w.header(DeformerType::Twist, d.layers, d.weight);
    w.frame(d.frame);
    w.vec3(kExtents, d.half_extents);
    w.params({d.length, d.angle});
}

void write(Writer& w, const Taper& d) noexcept {
    w.header(DeformerType::Taper, d.layers, d.weight);
    w.frame(d.frame);
    w.vec3(kExtents, d.half_extents);
    w.params({d.length, d.scale_x, d.scale_z});
}

void write(Writer& w, const CorridorCompression& d) noexcept {
    w.header(DeformerType::CorridorCompression, d.layers, d.weight);
    w.frame(d.frame);
    w.vec3(kExtents, d.half_extents);
    w.params({d.max_distance, d.player});
}

}  // namespace

DeformerType type_of(const Deformer& deformer) noexcept {
    struct Visitor {
        DeformerType operator()(const GlobalBasis&) const noexcept {
            return DeformerType::GlobalBasis;
        }
        DeformerType operator()(const AffineVolume& d) const noexcept {
            return d.shape == VolumeShape::Box ? DeformerType::BoxVolume
                                               : DeformerType::SphereVolume;
        }
        DeformerType operator()(const Bend&) const noexcept { return DeformerType::Bend; }
        DeformerType operator()(const Twist&) const noexcept { return DeformerType::Twist; }
        DeformerType operator()(const Taper&) const noexcept { return DeformerType::Taper; }
        DeformerType operator()(const CorridorCompression&) const noexcept {
            return DeformerType::CorridorCompression;
        }
    };
    return std::visit(Visitor{}, deformer);
}

void pack(const Deformer& deformer, Block out) noexcept {
    Writer w(out);
    std::visit([&w](const auto& d) { write(w, d); }, deformer);
}

std::optional<Deformer> unpack(ConstBlock in) noexcept {
    const Reader r(in);
    const float type_value = r.at(kType);
    if (!(type_value >= 0.0f && type_value <= 64.0f) || std::floor(type_value) != type_value) {
        return std::nullopt;
    }
    const auto type = static_cast<DeformerType>(static_cast<int>(type_value));
    switch (type) {
        case DeformerType::GlobalBasis:
            return GlobalBasis{r.vec3(kOrigin), r.matrix(), r.weight(), r.layers()};
        case DeformerType::SphereVolume:
        case DeformerType::BoxVolume: {
            AffineVolume d;
            d.centre = r.vec3(kOrigin);
            d.basis = r.matrix();
            d.shape = type == DeformerType::BoxVolume ? VolumeShape::Box : VolumeShape::Sphere;
            d.half_extents = r.vec3(kExtents);
            d.orientation = {r.at(kParam2), r.at(kParam3), r.at(kParam4), r.at(kParam5)};
            d.inner = r.at(kParam0);
            d.outer = r.at(kParam1);
            d.weight = r.weight();
            d.layers = r.layers();
            return d;
        }
        case DeformerType::Bend:
            return Bend{r.frame(),        r.at(kParam0), r.at(kParam1),
                        r.vec3(kExtents), r.weight(),    r.layers()};
        case DeformerType::Twist:
            return Twist{r.frame(),        r.at(kParam0), r.at(kParam1),
                         r.vec3(kExtents), r.weight(),    r.layers()};
        case DeformerType::Taper:
            return Taper{r.frame(),        r.at(kParam0), r.at(kParam1), r.at(kParam2),
                         r.vec3(kExtents), r.weight(),    r.layers()};
        case DeformerType::CorridorCompression:
            return CorridorCompression{r.frame(),        r.at(kParam0), r.at(kParam1),
                                       r.vec3(kExtents), r.weight(),    r.layers()};
        case DeformerType::None:
            break;
    }
    return std::nullopt;
}

std::vector<float> pack(const DeformationField& field) {
    const std::size_t count = std::min(field.deformers().size(), kMaxDeformers);
    std::vector<float> data(count * kFloatsPerDeformer);
    for (std::size_t n = 0; n < count; ++n) {
        pack(field.deformers()[n],
             std::span<float>(data).subspan(n * kFloatsPerDeformer).first<kFloatsPerDeformer>());
    }
    return data;
}

}  // namespace dbte::gpu
