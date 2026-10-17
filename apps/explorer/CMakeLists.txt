# The multi-sim explorer: links the engine plus every sim it hosts.
add_executable(explorer "main.cpp")
set_property(TARGET explorer PROPERTY CXX_STANDARD 20)
# menu thumbnails: docs/screenshots/<sim_name>.png (missing file = plain tile)
target_compile_definitions(explorer PRIVATE THUMBS_DIR="${CMAKE_SOURCE_DIR}/docs/screenshots/")
target_link_libraries(explorer PRIVATE
    sim_riemann_sphere
    sim_surface_renderer_ray_marched
    sim_lorenz_manifold
    sim_vector_field
    sim_diagonal_linear
    sim_lorenz_flow
    sim_lorenz_equilibrium
)
