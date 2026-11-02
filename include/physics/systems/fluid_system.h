#pragma once
#include "physics/types.h"

class PhysicsWorld;

class FluidSystem
{
public:
    static void SetupObstacles(PhysicsWorld& world);
    static void SimulateFluids(PhysicsWorld& world, float dt, float gravity, int iterations, float over_relaxation);

private:
    enum class Field { U, V, Smoke };
    
    static void integrate(Fluid& fluid, float dt, float gravity);
    static void SolveIncompressibility(Fluid& fluid, float dt, int iterations, float over_relaxation);
    static void ExtrapolateBoundaries(Fluid& fluid);
    [[nodiscard]] static float SampleField(const Fluid& fluid, float x, float y, Field field);
    
    [[nodiscard]] static float AverageU(const Fluid& fluid, const int i, const int j)
    {
        return 0.25f * (
            fluid.u_[Fluid::index(fluid, i, j - 1)]
            + fluid.u_[Fluid::index(fluid, i, j)]
            + fluid.u_[Fluid::index(fluid, i + 1, j - 1)]
            + fluid.u_[Fluid::index(fluid, i + 1, j)]);
    }

    [[nodiscard]] static float AverageV(const Fluid& fluid, const int i, const int j)
    {
        return 0.25f * (
            fluid.v_[Fluid::index(fluid, i - 1, j)]
            + fluid.v_[Fluid::index(fluid, i, j)]
            + fluid.v_[Fluid::index(fluid, i - 1, j + 1)]
            + fluid.v_[Fluid::index(fluid, i, j + 1)]);
    }

    static void AdvectVelocity(Fluid& fluid, float dt);
    static void AdvectSmoke(Fluid& fluid, float dt);
};


