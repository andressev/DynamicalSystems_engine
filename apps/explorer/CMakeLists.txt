# The multi-sim explorer: links the engine plus every sim it hosts.
add_executable(explorer "main.cpp")
set_property(TARGET explorer PROPERTY CXX_STANDARD 20)
target_link_libraries(explorer PRIVATE
    sim_riemann_sphere
)
