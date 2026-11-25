#include <doctest/doctest.h>

#include <format>
#include <string>

#include "dbte/version.hpp"

TEST_CASE("compiled library matches the headers") {
    const dbte::Version lib = dbte::version();
    CHECK(lib.major == dbte::kVersion.major);
    CHECK(lib.minor == dbte::kVersion.minor);
    CHECK(lib.patch == dbte::kVersion.patch);
}

TEST_CASE("version string matches the numeric version") {
    const dbte::Version v = dbte::kVersion;
    const std::string expected = std::format("{}.{}.{}", v.major, v.minor, v.patch);
    CHECK(dbte::version_string() == expected);
}
