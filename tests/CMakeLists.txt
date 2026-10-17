# Checks for the std-only math modules. They compile the module sources directly (no
# engine lib, no GL), which also enforces the math layer's std-only rule.

add_executable(math_tests
    math_tests.cpp
    "${CMAKE_SOURCE_DIR}/engine/src/Eigen3.cpp"
    "${CMAKE_SOURCE_DIR}/engine/src/InvariantManifold.cpp"
    "${CMAKE_SOURCE_DIR}/engine/src/LorenzSystem.cpp"
    "${CMAKE_SOURCE_DIR}/engine/src/TaylorIntegrator.cpp"
)
set_property(TARGET math_tests PROPERTY CXX_STANDARD 20)
target_include_directories(math_tests PRIVATE "${CMAKE_SOURCE_DIR}/engine/include")

add_test(NAME math_tests COMMAND math_tests)
