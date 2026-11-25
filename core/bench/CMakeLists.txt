# Microbenchmark for the hot paths: map, Jacobian and inverse.
add_executable(dbte_bench bench_main.cpp)
target_link_libraries(dbte_bench PRIVATE dbte::core)
dbte_set_warnings(dbte_bench)
