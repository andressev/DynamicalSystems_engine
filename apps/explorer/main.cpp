#include <engine/App.h>
#include "RiemannSphereSim.h"

#include <iostream>

int main()
{
    try
    {
        engine::App app("Math Explorer", 1280, 720);

        // one line per sim
        app.sims().add("riemann_sphere", &makeRiemannSphereSim);

        return app.run();
    }
    catch (const std::exception& e)
    {
        std::cout << "Fatal: " << e.what() << std::endl;
        return -1;
    }
}
