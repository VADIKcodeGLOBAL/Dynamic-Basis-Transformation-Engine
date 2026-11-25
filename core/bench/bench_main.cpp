// Microbenchmark for dbte-core: cost of map (with Jacobian) and inverse per point.
//
// Build in Release and run: dbte_bench. Numbers depend on the machine; use them to
// compare changes on the same machine, not as absolute targets.

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <random>
#include <vector>

#include "dbte/dbte.hpp"

namespace {

using Clock = std::chrono::steady_clock;

std::vector<dbte::Vec3> random_points(std::size_t count, float extent) {
    std::mt19937 engine(42);
    const auto uniform = [&] {
        return (static_cast<float>(static_cast<std::uint32_t>(engine()) >> 8) / 16777216.0f) *
                   2.0f * extent -
               extent;
    };
    std::vector<dbte::Vec3> points(count);
    for (dbte::Vec3& p : points) {
        p = {uniform(), uniform(), uniform()};
    }
    return points;
}

/// Runs fn over all points several times and returns the best time per point in ns.
template <typename Fn>
double best_ns_per_point(const std::vector<dbte::Vec3>& points, Fn&& fn, float& sink) {
    double best = 1e30;
    for (int run = 0; run < 5; ++run) {
        const auto start = Clock::now();
        for (const dbte::Vec3& p : points) {
            sink += fn(p);
        }
        const std::chrono::duration<double, std::nano> elapsed = Clock::now() - start;
        best = std::min(best, elapsed.count() / static_cast<double>(points.size()));
    }
    return best;
}

void report(const char* name, double map_ns, double inverse_ns) {
    std::printf("%-22s %10.1f %12.1f\n", name, map_ns, inverse_ns);
}

}  // namespace

int main() {
    using namespace dbte;

    AffineVolume sphere;
    sphere.basis = basis_from_vectors({1.0f, 0.0f, 0.0f}, {0.3f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f});
    sphere.inner = 2.0f;
    sphere.outer = 6.0f;
    AffineVolume box = sphere;
    box.shape = VolumeShape::Box;
    box.half_extents = {2.0f, 1.0f, 2.0f};
    box.inner = 0.0f;
    box.outer = 4.0f;
    Bend bend;
    bend.curvature = 0.1f;
    bend.length = 8.0f;
    Twist twist;
    twist.length = 8.0f;
    twist.angle = 1.0f;
    Taper taper;
    taper.length = 8.0f;
    taper.scale_x = 0.7f;
    taper.scale_z = 1.3f;
    CorridorCompression corridor;
    corridor.max_distance = 20.0f;
    corridor.player = 3.0f;
    GlobalBasis global;
    global.basis = sphere.basis;

    const std::vector<Deformer> single = {global, sphere, box, bend, twist, taper, corridor};
    const char* names[] = {"global basis", "sphere volume", "box volume", "bend",
                           "twist",        "taper",         "corridor"};
    const std::vector<Vec3> points = random_points(200'000, 6.0f);
    float sink = 0.0f;

    std::printf("dbte-core %s microbenchmark, %zu points, best of 5 runs\n",
                version_string().data(), points.size());
    std::printf("%-22s %10s %12s\n", "deformer", "map ns", "inverse ns");
    for (std::size_t n = 0; n < single.size(); ++n) {
        const Deformer& d = single[n];
        const double map_ns = best_ns_per_point(
            points, [&](Vec3 p) { return determinant(evaluate(d, p).jacobian); }, sink);
        const double inverse_ns =
            best_ns_per_point(points, [&](Vec3 p) { return inverse(d, p).point.x; }, sink);
        report(names[n], map_ns, inverse_ns);
    }

    const DeformationField field(single);
    const double map_ns = best_ns_per_point(
        points, [&](Vec3 p) { return determinant(field.evaluate(p).jacobian); }, sink);
    const double inverse_ns =
        best_ns_per_point(points, [&](Vec3 p) { return field.inverse(p).point.x; }, sink);
    report("chain of all 7", map_ns, inverse_ns);

    // Print the sink so the compiler cannot drop the work.
    std::printf("(checksum %g)\n", static_cast<double>(sink));
    return 0;
}
