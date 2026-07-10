// Generic single-sim entry point. add_sim() in sims/CMakeLists.txt compiles
// this once per sim with STANDALONE_SIM_* definitions selecting the sim.
#include <engine/App.h>
#include STANDALONE_SIM_HEADER

#include <iostream>

int main()
{
    try
    {
        engine::App app(STANDALONE_SIM_NAME, 1280, 720);
        app.sims().add(STANDALONE_SIM_NAME, &STANDALONE_SIM_FACTORY);
        return app.run();
    }
    catch (const std::exception& e)
    {
        std::cout << "Fatal: " << e.what() << std::endl;
        return -1;
    }
}
