cmake_minimum_required(VERSION 3.24)

project(DBTE
    VERSION 0.1.0
    DESCRIPTION "Dynamic Basis Transformation Engine: real-time non-orthonormal basis transformations for games"
    HOMEPAGE_URL "https://github.com/VADIKcodeGLOBAL/Dynamic-Basis-Transformation-Engine"
    LANGUAGES CXX)

option(DBTE_BUILD_TESTS "Build the dbte-core unit tests" ${PROJECT_IS_TOP_LEVEL})
option(DBTE_WARNINGS_AS_ERRORS "Treat compiler warnings as errors" ${PROJECT_IS_TOP_LEVEL})

set(CMAKE_CXX_EXTENSIONS OFF)

include(cmake/Warnings.cmake)

if(DBTE_BUILD_TESTS)
    enable_testing()
endif()

add_subdirectory(core)
