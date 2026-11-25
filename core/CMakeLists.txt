# dbte-core: engine-agnostic math for dynamic basis transformations.
# No dependencies besides the C++20 standard library.

add_library(dbte_core
    src/basis.cpp
    src/deformers.cpp
    src/field.cpp
    src/gpu_layout.cpp
    src/math.cpp
    src/validate.cpp
    src/version.cpp)
add_library(dbte::core ALIAS dbte_core)

target_include_directories(dbte_core PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_compile_features(dbte_core PUBLIC cxx_std_20)
dbte_set_warnings(dbte_core)

if(DBTE_BUILD_TESTS)
    add_subdirectory(tests)
endif()

if(DBTE_BUILD_BENCHMARKS)
    add_subdirectory(bench)
endif()
