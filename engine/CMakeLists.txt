# The engine static library: window/GL context, shader, camera, mesh,
# sim interface + registry, app main loop, plus glad and Dear ImGui.

set(IMGUI_DIR ${CMAKE_SOURCE_DIR}/External/imgui)

add_library(engine STATIC
    "src/Window.cpp"
    "src/Shader.cpp"
    "src/Expression.cpp"
    "src/VectorField.cpp"
    "src/Integrator.cpp"
    "src/Eigen3.cpp"
    "src/InvariantManifold.cpp"
    "src/LorenzSystem.cpp"
    "src/TaylorIntegrator.cpp"
    "src/ConjugacyMap.cpp"
    "src/Camera.cpp"
    "src/Mesh.cpp"
    "src/ArrowGlyphs.cpp"
    "src/FlowKeys.cpp"
    "src/GpuTimer.cpp"
    "src/FrameCopy.cpp"
    "src/Trajectory.cpp"
    "src/TrajectoryRunner.cpp"
    "src/LatexImage.cpp"
    "src/SimRegistry.cpp"
    "src/App.cpp"
    "${CMAKE_SOURCE_DIR}/External/Glad/src/glad.c"
    "${IMGUI_DIR}/imgui.cpp"
    "${IMGUI_DIR}/imgui_draw.cpp"
    "${IMGUI_DIR}/imgui_tables.cpp"
    "${IMGUI_DIR}/imgui_widgets.cpp"
    "${IMGUI_DIR}/imgui_demo.cpp"
    "${IMGUI_DIR}/backends/imgui_impl_glfw.cpp"
    "${IMGUI_DIR}/backends/imgui_impl_opengl3.cpp"
)

set_property(TARGET engine PROPERTY CXX_STANDARD 20)

target_include_directories(engine PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
    "${CMAKE_SOURCE_DIR}/External/glfw/include"
    "${CMAKE_SOURCE_DIR}/External/Glad/include"
    "${CMAKE_SOURCE_DIR}/External/glm"
    "${CMAKE_SOURCE_DIR}/External/stb_image"
    "${IMGUI_DIR}"
    "${IMGUI_DIR}/backends"
)

target_link_libraries(engine PUBLIC
    "${CMAKE_SOURCE_DIR}/External/glfw/lib-vc2022/glfw3.lib"
    opengl32
)

# LaTeX layout for LatexImage (cmake/microtex.cmake); private: only LatexImage.cpp
# includes MicroTeX headers.
target_link_libraries(engine PRIVATE microtex)
