include(FetchContent)

# doctest is header-only. Use an installed copy when there is one (for example the
# doctest-dev package on Ubuntu); otherwise download the pinned release. SOURCE_SUBDIR
# points at a directory without a CMakeLists.txt, so doctest's own build scripts are
# not pulled into ours.
find_package(doctest 2.4 CONFIG QUIET)
if(NOT doctest_FOUND)
    FetchContent_Declare(doctest
        URL https://github.com/doctest/doctest/archive/refs/tags/v2.4.11.tar.gz
        DOWNLOAD_EXTRACT_TIMESTAMP TRUE
        SOURCE_SUBDIR do-not-add-subdirectory)
    FetchContent_MakeAvailable(doctest)
    add_library(dbte_doctest INTERFACE)
    target_include_directories(dbte_doctest SYSTEM INTERFACE "${doctest_SOURCE_DIR}")
    add_library(doctest::doctest ALIAS dbte_doctest)
endif()

add_executable(dbte_core_tests
    main.cpp
    test_version.cpp)
target_link_libraries(dbte_core_tests PRIVATE dbte::core doctest::doctest)
# Let doctest include the real <ostream> instead of forward-declaring std types.
# Newer MSVC STL releases fail to stringify std::string_view against doctest's
# forward declarations ("use of undefined type std::basic_ostream").
target_compile_definitions(dbte_core_tests PRIVATE DOCTEST_CONFIG_USE_STD_HEADERS)
dbte_set_warnings(dbte_core_tests)

add_test(NAME dbte_core_tests COMMAND dbte_core_tests)
