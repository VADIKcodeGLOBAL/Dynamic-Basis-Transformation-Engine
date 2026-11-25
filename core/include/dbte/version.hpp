#pragma once

#include <string_view>

namespace dbte {

/// Semantic version of dbte-core.
struct Version {
    int major;
    int minor;
    int patch;
};

/// Version of the headers you compile against. Keep in sync with project() in CMakeLists.txt.
inline constexpr Version kVersion{0, 1, 0};

/// Version of the compiled library, to detect a header/library mismatch at run time.
[[nodiscard]] Version version() noexcept;

/// The library version as text, for example "0.1.0".
[[nodiscard]] std::string_view version_string() noexcept;

}  // namespace dbte
