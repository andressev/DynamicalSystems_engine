# Each sim is its own static library (sim_<name>) so executables can link
# exactly the sims they want. add_sim() also creates a <name>_standalone
# executable that hosts just that one sim.
#
# Usage (from the sim's own CMakeLists.txt):
#   add_sim(<name> <header> <factory> <sources...>)
# e.g.
#   add_sim(riemann_sphere RiemannSphereSim.h makeRiemannSphereSim RiemannSphereSim.cpp)

function(add_sim NAME HEADER FACTORY)
    add_library(sim_${NAME} STATIC ${ARGN})
    set_property(TARGET sim_${NAME} PROPERTY CXX_STANDARD 20)
    target_link_libraries(sim_${NAME} PUBLIC engine)
    target_include_directories(sim_${NAME} PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
    # sims load their shaders relative to the sims/ root
    target_compile_definitions(sim_${NAME} PRIVATE SIMS_DIR="${CMAKE_SOURCE_DIR}/sims/")

    if(BUILD_STANDALONE_SIMS)
        add_executable(${NAME}_standalone "${CMAKE_SOURCE_DIR}/apps/standalone/main.cpp")
        set_property(TARGET ${NAME}_standalone PROPERTY CXX_STANDARD 20)
        target_link_libraries(${NAME}_standalone PRIVATE sim_${NAME})
        target_compile_definitions(${NAME}_standalone PRIVATE
            STANDALONE_SIM_HEADER="${HEADER}"
            STANDALONE_SIM_FACTORY=${FACTORY}
            STANDALONE_SIM_NAME="${NAME}"
        )
    endif()
endfunction()

add_subdirectory("riemann_sphere")
add_subdirectory("surface_renderer_ray_marched")
add_subdirectory("lorenz_manifold")
add_subdirectory("vector_field")
add_subdirectory("diagonal_linear")
