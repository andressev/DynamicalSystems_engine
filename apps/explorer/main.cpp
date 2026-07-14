#include <engine/App.h>
#include "RiemannSphereSim.h"
#include "SurfaceRendererRayMarchedSim.h"

#include <iostream>

int main()
{
    try
    {
        engine::App app("Math Explorer", 1280, 720);

        // one line per sim
        app.sims().add("surface_renderer_ray_marched", &makeSurfaceRendererRayMarchedSim,
                       THUMBS_DIR "surface_renderer_ray_marched.png");
        app.sims().add("riemann_sphere", &makeRiemannSphereSim,
                       THUMBS_DIR "riemann_sphere.png");

        return app.run();
    }
    catch (const std::exception& e)
    {
        std::cout << "Fatal: " << e.what() << std::endl;
        return -1;
    }
}
