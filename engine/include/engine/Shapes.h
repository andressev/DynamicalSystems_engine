#ifndef ENGINE_SHAPES_H
#define ENGINE_SHAPES_H

#include <glm.hpp>
#include <gtc/constants.hpp>
#include <vector>
#include <cmath>

namespace engine {

// Procedural geometry generators. All vertex data is interleaved floats.
class Shapes {
public:
    // pos(3) + normal(3) + uv(2) interleaved -> layout {3, 3, 2}
    static std::vector<float> sphereVertices(int sectorCount, int stackCount, float radius)
    {
        std::vector<float> v;
        v.reserve((stackCount + 1) * (sectorCount + 1) * 8);

        float lengthInv = 1.0f / radius;
        float sectorStep = 2 * glm::pi<float>() / sectorCount;
        float stackStep = glm::pi<float>() / stackCount;

        for (int i = 0; i <= stackCount; ++i)
        {
            float stackAngle = glm::pi<float>() / 2 - i * stackStep;
            float xy = radius * cosf(stackAngle);
            float z = radius * sinf(stackAngle);

            for (int j = 0; j <= sectorCount; ++j)
            {
                float sectorAngle = j * sectorStep;
                float x = xy * cosf(sectorAngle);
                float y = xy * sinf(sectorAngle);

                v.push_back(x);
                v.push_back(y);
                v.push_back(z);

                v.push_back(x * lengthInv);
                v.push_back(y * lengthInv);
                v.push_back(z * lengthInv);

                v.push_back((float)j / sectorCount);
                v.push_back((float)i / stackCount);
            }
        }
        return v;
    }

    static std::vector<unsigned int> sphereIndices(int sectorCount, int stackCount)
    {
        std::vector<unsigned int> idx;
        idx.reserve(stackCount * sectorCount * 6);

        for (int i = 0; i < stackCount; ++i)
        {
            int k1 = i * (sectorCount + 1);
            int k2 = k1 + sectorCount + 1;

            for (int j = 0; j < sectorCount; ++j, ++k1, ++k2)
            {
                if (i != 0)
                {
                    idx.push_back(k1);
                    idx.push_back(k2);
                    idx.push_back(k1 + 1);
                }
                if (i != (stackCount - 1))
                {
                    idx.push_back(k1 + 1);
                    idx.push_back(k2);
                    idx.push_back(k2 + 1);
                }
            }
        }
        return idx;
    }

    // three axis-aligned lines through the origin -> layout {3}, draw with GL_LINES
    static std::vector<float> axisVertices(float extent = 500.0f)
    {
        return {
             extent,  0.0f,   0.0f,
            -extent,  0.0f,   0.0f,
             0.0f,    extent, 0.0f,
             0.0f,   -extent, 0.0f,
             0.0f,    0.0f,   extent,
             0.0f,    0.0f,  -extent,
        };
    }

    static std::vector<unsigned int> axisIndices()
    {
        return { 0, 1, 2, 3, 4, 5 };
    }

    // unit quad in the XY plane -> layout {3}
    static std::vector<float> planeVertices()
    {
        return {
             1.f,  .0f, 1.0f,
             1.f, 0.f, -1.0f,
            -1.f, 0.f, -1.0f,
            -1.f,  0.f, 1.0f,
        };
    }

    static std::vector<unsigned int> planeIndices()
    {
        return { 0, 1, 2, 0, 2, 3 };
    }
};

} // namespace engine

#endif
