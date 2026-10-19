#include <engine/App.h>
#include "RiemannSphereSim.h"
#include "SurfaceRendererRayMarchedSim.h"
#include "LorenzManifoldSim.h"
#include "VectorFieldSim.h"
#include "DiagonalLinearSim.h"
#include "LorenzFlowSim.h"
#include "LorenzEquilibriumSim.h"
#include "LorenzManifoldGrowthSim.h"
#include "LearnGeodesicLevelSetsSim.h"

#include <iostream>

int main()
{
    try
    {
        engine::App app("Math Explorer", 1280, 720);

        // one line per sim
        app.sims().add("vector_field", &makeVectorFieldSim,
                       THUMBS_DIR "vector_field.png");
        app.sims().add("diagonal_linear", &makeDiagonalLinearSim,
                       THUMBS_DIR "diagonal_linear.png");
        app.sims().add("lorenz_flow", &makeLorenzFlowSim,
                       THUMBS_DIR "lorenz_flow.png");
        app.sims().add("lorenz_equilibrium", &makeLorenzEquilibriumSim,
                       THUMBS_DIR "lorenz_equilibrium.png");
        app.sims().add("lorenz_manifold_growth", &makeLorenzManifoldGrowthSim,
                       THUMBS_DIR "lorenz_manifold_growth.png");
        app.sims().add("learn_geodesic_level_sets", &makeLearnGeodesicLevelSetsSim,
                       THUMBS_DIR "learn_geodesic_level_sets.png");

        return app.run();
    }
    catch (const std::exception& e)
    {
        std::cout << "Fatal: " << e.what() << std::endl;
        return -1;
    }
}
