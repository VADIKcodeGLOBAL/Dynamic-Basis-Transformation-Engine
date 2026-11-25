#include "dbte/deformers.hpp"

#include <cmath>
#include <variant>

namespace dbte {
namespace {

constexpr float kTiny = 1e-30f;

/// Mapping and Jacobian in a deformer's local frame.
struct Local {
    Vec3 point;
    Mat3 jacobian;
};

/// Evaluates a deformer defined in a rigid local frame: Phi(p) = o + R phi(R^T (p - o)),
/// J(p) = R J_local R^T.
template <typename LocalFn>
Evaluation evaluate_in_frame(const Affine3& frame, Vec3 p, LocalFn&& local_fn) noexcept {
    const Mat3 rt = transpose(frame.linear);
    const Local local = local_fn(rt * (p - frame.translation));
    return {frame.translation + frame.linear * local.point, frame.linear * local.jacobian * rt};
}

template <typename LocalFn>
InverseResult inverse_in_frame(const Affine3& frame, Vec3 q, LocalFn&& local_fn) noexcept {
    InverseResult result = local_fn(transpose(frame.linear) * (q - frame.translation));
    result.point = frame.translation + frame.linear * result.point;
    return result;
}

/// tanh(x) and sech^2(x) without cancellation for large |x|.
struct TanhSech2 {
    float tanh;
    float sech2;
};

TanhSech2 tanh_sech2(float x) noexcept {
    const float e = std::exp(-2.0f * std::abs(x));
    const float inv = 1.0f / (1.0f + e);
    return {std::copysign((1.0f - e) * inv, x), 4.0f * e * inv * inv};
}

// --- Affine volume ----------------------------------------------------------------------

struct Falloff {
    float weight;   ///< w(p)
    Vec3 gradient;  ///< grad w(p)
};

Falloff volume_falloff(const AffineVolume& d, Vec3 p) noexcept {
    const Vec3 rel = p - d.centre;
    float dist = 0.0f;
    Vec3 grad_dist{};
    if (d.shape == VolumeShape::Sphere) {
        dist = length(rel);
        if (dist > 0.0f) {
            grad_dist = rel / dist;
        }
    } else {
        const Vec3 local = rotate(conjugate(d.orientation), rel);
        const Vec3 outside = max(abs(local) - d.half_extents, Vec3{});
        dist = length(outside);
        if (dist > 0.0f) {
            grad_dist = rotate(d.orientation, hadamard(outside, sign(local)) / dist);
        }
    }
    const float width = d.outer - d.inner;
    if (!(width > 0.0f)) {  // Hard edge. The validator reports it as an invalid range.
        return {dist <= d.inner ? d.weight : 0.0f, Vec3{}};
    }
    const float t = (dist - d.inner) / width;
    return {d.weight * (1.0f - smoothstep01(t)),
            grad_dist * (-d.weight * smoothstep01_derivative(t) / width)};
}

// --- Bend -----------------------------------------------------------------------------

Local bend_local(float k, float segment_length, Vec3 p) noexcept {
    const float len = std::max(segment_length, 0.0f);
    const float yc = clamp(p.y, 0.0f, len);
    const float theta = k * yc;
    const float s = std::sin(theta);
    const float c = std::cos(theta);
    const float half = 0.5f * theta;
    const float arc_y = yc * sinc(theta);                  // sin(theta) / k
    const float arc_z = yc * std::sin(half) * sinc(half);  // (1 - cos(theta)) / k
    const float dy = p.y - yc;
    const Vec3 point{p.x, arc_y - p.z * s + dy * c, arc_z + p.z * c + dy * s};
    const float stretch = (p.y > 0.0f && p.y < len) ? 1.0f - k * p.z : 1.0f;
    return {point, Mat3{{1.0f, 0.0f, 0.0f}, {0.0f, stretch * c, stretch * s}, {0.0f, -s, c}}};
}

InverseResult bend_inverse_local(float k, float segment_length, Vec3 q) noexcept {
    const float len = std::max(segment_length, 0.0f);
    if (k == 0.0f || len == 0.0f) {
        return {q, true, 0};
    }
    // Bent segment: (k Y, 1 - k Z) = (1 - k z) (sin theta, cos theta) with theta = k y.
    const float ky = k * q.y;
    const float kz = 1.0f - k * q.z;
    const float y = std::atan2(ky, kz) / k;
    if (y >= 0.0f && y <= len) {
        const float s = std::hypot(ky, kz);  // 1 - k z
        // z = (1 - s) / k, rewritten without cancellation for small k.
        const float z = (2.0f * q.z - k * (q.y * q.y + q.z * q.z)) / (1.0f + s);
        return {{q.x, y, z}, true, 0};
    }
    // Before the segment space is unchanged.
    if (q.y < 0.0f) {
        return {q, true, 0};
    }
    // Beyond the segment: (Y, Z) = end + R(theta_end) (y - length, z).
    const float theta = k * len;
    const float s = std::sin(theta);
    const float c = std::cos(theta);
    const float half = 0.5f * theta;
    const float ry = q.y - len * sinc(theta);
    const float rz = q.z - len * std::sin(half) * sinc(half);
    const float dy = c * ry + s * rz;
    return {{q.x, len + dy, -s * ry + c * rz}, dy >= 0.0f, 0};
}

// --- Twist and taper ------------------------------------------------------------------

Local twist_local(float angle, float segment_length, Vec3 p) noexcept {
    const float inv_len = segment_length > 0.0f ? 1.0f / segment_length : 0.0f;
    const float t = p.y * inv_len;
    const float theta = angle * smoothstep01(t);
    const float dtheta = angle * smoothstep01_derivative(t) * inv_len;
    const float s = std::sin(theta);
    const float c = std::cos(theta);
    const float x = c * p.x + s * p.z;
    const float z = -s * p.x + c * p.z;
    return {{x, p.y, z}, Mat3{{c, 0.0f, -s}, {dtheta * z, 1.0f, -dtheta * x}, {s, 0.0f, c}}};
}

InverseResult twist_inverse_local(float angle, float segment_length, Vec3 q) noexcept {
    const float inv_len = segment_length > 0.0f ? 1.0f / segment_length : 0.0f;
    const float theta = angle * smoothstep01(q.y * inv_len);
    const float s = std::sin(theta);
    const float c = std::cos(theta);
    return {{c * q.x - s * q.z, q.y, s * q.x + c * q.z}, true, 0};
}

struct TaperProfile {
    float fx, fz;    ///< Scale factors at y.
    float dfx, dfz;  ///< Their derivatives along y.
};

TaperProfile taper_profile(const Taper& d, float y) noexcept {
    const float inv_len = d.length > 0.0f ? 1.0f / d.length : 0.0f;
    const float t = y * inv_len;
    const float h = smoothstep01(t);
    const float dh = smoothstep01_derivative(t) * inv_len;
    const float ax = d.weight * (d.scale_x - 1.0f);
    const float az = d.weight * (d.scale_z - 1.0f);
    return {1.0f + ax * h, 1.0f + az * h, ax * dh, az * dh};
}

}  // namespace

// --- Public API -----------------------------------------------------------------------

Evaluation evaluate(const GlobalBasis& d, Vec3 p) noexcept {
    const Mat3 j = Mat3::identity() * (1.0f - d.weight) + d.basis * d.weight;
    return {d.origin + j * (p - d.origin), j};
}

InverseResult inverse(const GlobalBasis& d, Vec3 q, const InverseOptions& /*options*/) noexcept {
    const Mat3 j = Mat3::identity() * (1.0f - d.weight) + d.basis * d.weight;
    if (!(std::abs(determinant(j)) > kTiny)) {
        return {q, false, 0};
    }
    return {d.origin + inverse(j) * (q - d.origin), true, 0};
}

float volume_distance(const AffineVolume& d, Vec3 p) noexcept {
    const Vec3 rel = p - d.centre;
    if (d.shape == VolumeShape::Sphere) {
        return length(rel);
    }
    const Vec3 local = rotate(conjugate(d.orientation), rel);
    return length(max(abs(local) - d.half_extents, Vec3{}));
}

Evaluation evaluate(const AffineVolume& d, Vec3 p) noexcept {
    const Falloff f = volume_falloff(d, p);
    const Mat3 m = d.basis - Mat3::identity();
    const Vec3 u = m * (p - d.centre);
    return {p + u * f.weight, Mat3::identity() + m * f.weight + outer(u, f.gradient)};
}

InverseResult inverse(const AffineVolume& d, Vec3 q, const InverseOptions& options) noexcept {
    if (d.weight == 0.0f) {
        return {q, true, 0};
    }
    const Mat3 m = d.basis - Mat3::identity();

    // Inside the full-effect region the mapping is affine.
    const Mat3 inner_map = Mat3::identity() + m * d.weight;
    if (std::abs(determinant(inner_map)) > kTiny) {
        const Vec3 p = d.centre + inverse(inner_map) * (q - d.centre);
        if (volume_distance(d, p) <= d.inner) {
            return {p, true, 0};
        }
    }
    // Beyond the outer edge it is the identity.
    if (volume_distance(d, q) >= d.outer) {
        return {q, true, 0};
    }

    // In the soft edge: Newton's method with backtracking, starting from the affine map
    // with the weight taken at q.
    const float tolerance = options.tolerance * std::max(1.0f, length(q));
    const Mat3 guess_map = Mat3::identity() + m * volume_falloff(d, q).weight;
    Vec3 p = std::abs(determinant(guess_map)) > kTiny
                 ? d.centre + inverse(guess_map) * (q - d.centre)
                 : q;
    Evaluation e = evaluate(d, p);
    float error = length(e.point - q);
    int iterations = 0;
    while (iterations < options.max_iterations && error > tolerance) {
        if (!(std::abs(determinant(e.jacobian)) > kTiny)) {
            break;
        }
        const Vec3 step = inverse(e.jacobian) * (e.point - q);
        ++iterations;
        bool improved = false;
        float scale = 1.0f;
        for (int halving = 0; halving < 8 && !improved; ++halving, scale *= 0.5f) {
            const Vec3 candidate = p - step * scale;
            const Evaluation ce = evaluate(d, candidate);
            const float candidate_error = length(ce.point - q);
            if (candidate_error < error) {
                p = candidate;
                e = ce;
                error = candidate_error;
                improved = true;
            }
        }
        if (!improved) {
            break;
        }
    }
    return {p, error <= tolerance, iterations};
}

Evaluation evaluate(const Bend& d, Vec3 p) noexcept {
    const float k = d.weight * d.curvature;
    return evaluate_in_frame(d.frame, p, [&](Vec3 l) { return bend_local(k, d.length, l); });
}

InverseResult inverse(const Bend& d, Vec3 q, const InverseOptions& /*options*/) noexcept {
    const float k = d.weight * d.curvature;
    return inverse_in_frame(d.frame, q, [&](Vec3 l) { return bend_inverse_local(k, d.length, l); });
}

Evaluation evaluate(const Twist& d, Vec3 p) noexcept {
    const float angle = d.weight * d.angle;
    return evaluate_in_frame(d.frame, p, [&](Vec3 l) { return twist_local(angle, d.length, l); });
}

InverseResult inverse(const Twist& d, Vec3 q, const InverseOptions& /*options*/) noexcept {
    const float angle = d.weight * d.angle;
    return inverse_in_frame(d.frame, q,
                            [&](Vec3 l) { return twist_inverse_local(angle, d.length, l); });
}

Evaluation evaluate(const Taper& d, Vec3 p) noexcept {
    return evaluate_in_frame(d.frame, p, [&](Vec3 l) {
        const TaperProfile f = taper_profile(d, l.y);
        return Local{
            {f.fx * l.x, l.y, f.fz * l.z},
            Mat3{{f.fx, 0.0f, 0.0f}, {f.dfx * l.x, 1.0f, f.dfz * l.z}, {0.0f, 0.0f, f.fz}}};
    });
}

InverseResult inverse(const Taper& d, Vec3 q, const InverseOptions& /*options*/) noexcept {
    return inverse_in_frame(d.frame, q, [&](Vec3 l) {
        const TaperProfile f = taper_profile(d, l.y);
        if (!(std::abs(f.fx) > kTiny) || !(std::abs(f.fz) > kTiny)) {
            return InverseResult{l, false, 0};
        }
        return InverseResult{{l.x / f.fx, l.y, l.z / f.fz}, true, 0};
    });
}

Evaluation evaluate(const CorridorCompression& d, Vec3 p) noexcept {
    if (!(d.weight > 0.0f) || !(d.max_distance > 0.0f)) {
        return {p, Mat3::identity()};
    }
    const float a = d.max_distance / d.weight;
    return evaluate_in_frame(d.frame, p, [&](Vec3 l) {
        const TanhSech2 near_player = tanh_sech2((l.y - d.player) / a);
        const float anchor = tanh_sech2(d.player / a).tanh;
        return Local{{l.x, a * (near_player.tanh + anchor), l.z},
                     Mat3::diagonal({1.0f, near_player.sech2, 1.0f})};
    });
}

InverseResult inverse(const CorridorCompression& d, Vec3 q,
                      const InverseOptions& /*options*/) noexcept {
    if (!(d.weight > 0.0f) || !(d.max_distance > 0.0f)) {
        return {q, true, 0};
    }
    const float a = d.max_distance / d.weight;
    return inverse_in_frame(d.frame, q, [&](Vec3 l) {
        // Visible y is limited to an open interval of length 2a; points outside it have
        // no preimage, so clamp them to the nearest one and report no convergence.
        const float v = l.y / a - tanh_sech2(d.player / a).tanh;
        const bool inside = std::abs(v) < 1.0f;
        const float vc = clamp(v, -1.0f + 1e-6f, 1.0f - 1e-6f);
        return InverseResult{{l.x, d.player + a * std::atanh(vc), l.z}, inside, 0};
    });
}

Evaluation evaluate(const Deformer& d, Vec3 p) noexcept {
    return std::visit([p](const auto& x) { return evaluate(x, p); }, d);
}

InverseResult inverse(const Deformer& d, Vec3 q, const InverseOptions& options) noexcept {
    return std::visit([q, &options](const auto& x) { return inverse(x, q, options); }, d);
}

LayerMask layers(const Deformer& d) noexcept {
    return std::visit([](const auto& x) { return x.layers; }, d);
}

}  // namespace dbte
