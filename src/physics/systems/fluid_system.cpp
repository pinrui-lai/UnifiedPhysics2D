#include <algorithm>
#include <execution>
#include <ranges>

#include "physics/physics_world.h"
#include "physics/collision/narrow_phase/rigid_soft_collision.h"
#include "physics/systems/fluid_system.h"

void FluidSystem::SetupRightOpenVortexScene(Fluid& fluid, const float inlet_speed)
{
    // Solid left, bottom and top boundaries; the right side is an outlet.
    for (int i = 0; i < fluid.nx_; ++i)
    {
        for (int j = 0; j < fluid.ny_; ++j)
        {
            const bool wall = (i == 0 || j == 0 || j == fluid.ny_ - 1);
            fluid.solid_[index(fluid, i, j)] = wall ? 0.0f : 1.0f;
            
            if (i == 1) 
            {
                fluid.u_[index(fluid, i, j)] = inlet_speed;
            }
        }
    }
    
    // Inlet strip, this is stored in the left ghost column and is carried into the domain by advection.
    const float pipe_height = 0.1f * static_cast<float>(fluid.ny_);
    const int min_j = static_cast<int>(std::floor(0.5f * fluid.ny_ - 0.5f * pipe_height));
    const int max_j = static_cast<int>(std::floor(0.5f * fluid.ny_ + 0.5f * pipe_height));
    for (int j = std::max(0, min_j); j < std::min(fluid.ny_, max_j); ++j)
    {
        fluid.smoke_[index(fluid, 0, j)] = 1.0f;
    }
}

namespace 
{
    template<class... Ts> struct Overload : Ts... { using Ts::operator()...; };    
}
void FluidSystem::SetupObstacles(PhysicsWorld& world)
{
    for (auto[fluid_idx, rigid_idx] : world.fluid_rigid_pairs_)
    {
        auto& fluid = world.fluids_[fluid_idx];
        
        // Reset inner solids.
        for (int i = 1; i < fluid.nx_ - 2; ++i)
        {
            for (int j = 1; j < fluid.ny_ - 2; ++j)
            {
                fluid.solid_[index(fluid, i, j)] = 1.0f;
            }
        }
        
        auto set_obstacle = [&fluid](const int i, const int j)
        {
            fluid.solid_[index(fluid, i, j)] = 0.0f;
            fluid.smoke_[index(fluid, i, j)] = 0.0f;
            fluid.u_[index(fluid, i, j)] = 0.0f;
            fluid.u_[index(fluid, i + 1, j)] = 0.0f;
            fluid.v_[index(fluid, i, j)] = 0.0f;
            fluid.v_[index(fluid, i, j + 1)] = 0.0f;
        };
        
        Shape& shape = world.shapes_[rigid_idx];
        std::visit(Overload{
            [&](const BoxShape& b)
            {
                for (int i = 1; i < fluid.nx_ - 2; ++i)
                {
                    for (int j = 1; j < fluid.ny_ - 2; ++j)
                    {
                        const Vec2 cell_pos{(static_cast<float>(i) + 0.5f) * fluid.h_ + fluid.position_.x, 
                            (static_cast<float>(j) + 0.5f) * fluid.h_ + fluid.position_.y};
                       
                        Vec2 cell_pos_map_to_box = cell_pos - world.positions_[rigid_idx].val;
                        cell_pos_map_to_box = world.rots_[rigid_idx].MulT(cell_pos_map_to_box);
                        
                        const float depth = RigidSoftCollision::SdfBox(cell_pos_map_to_box, {b.half_width, b.half_height});
                        if (depth >= 0.f) continue;
                        
                        set_obstacle(i, j);
                    }
                }
            },
            [&](const CircleShape& c)
            {
                const float r2 = c.radius * c.radius;
                for (int i = 1; i < fluid.nx_ - 2; ++i)
                {
                    for (int j = 1; j < fluid.ny_ - 2; ++j)
                    {
                        // fluid.solid_[index(fluid, i, j)] = 1.0f;
                        const float dx = (static_cast<float>(i) + 0.5f) * fluid.h_ + fluid.position_.x - world.positions_[rigid_idx].val.x;
                        const float dy = (static_cast<float>(j) + 0.5f) * fluid.h_ + fluid.position_.y - world.positions_[rigid_idx].val.y;
                        if (dx * dx + dy * dy >= r2) continue;
                        
                        set_obstacle(i, j);
                    }
                }
            },
            [&](const PolygonShape& p){},
        }, shape);
    }
}

void FluidSystem::SimulateFluids(PhysicsWorld& world, const float dt, const float gravity, const int iterations, const float over_relaxation)
{
    SetupObstacles(world);
    
    for (auto& fluid : world.fluids_)
    {
        integrate(fluid, dt, gravity);
        std::ranges::fill(fluid.pressure_, 0.0f);
        SolveIncompressibility(fluid, dt, iterations, over_relaxation);
        ExtrapolateBoundaries(fluid);
        AdvectVelocity(fluid, dt);
        AdvectSmoke(fluid, dt);
    }
}

void FluidSystem::integrate(Fluid& fluid, const float dt, const float gravity)
{
    for (int i = 1; i < fluid.nx_; ++i)
    {
        for (int j = 1; j < fluid.ny_ - 1; ++j)
        {
            // Only accelerate vertical faces between two fluid cells.
            if (fluid.solid_[index(fluid, i, j)] != 0.0f && fluid.solid_[index(fluid, i, j - 1)] != 0.0f)
            {
                fluid.v_[index(fluid, i, j)] += gravity * dt;
            }
        }
    }
}

void FluidSystem::SolveIncompressibility(Fluid& fluid, const float dt, const int iterations, const float over_relaxation)
{
    const float pressure_scale = fluid.density_ * fluid.h_ / dt;

    // In-place Gauss-Seidel relaxation. The over-relaxation factor
    // (typically 1.9) accelerates convergence.
    for (int iter = 0; iter < iterations; ++iter)
    {
        auto index_range = std::views::iota(int{0}, fluid.nx_ - 1);
        std::for_each(std::execution::par, std::begin(index_range), std::end(index_range), [&](auto i)
        {
            for (int j = 1; j < fluid.ny_ - 1; ++j)
            {
                if (fluid.solid_[index(fluid, i, j)] == 0.0f)
                {
                    continue;
                }

                const float sx0 = fluid.solid_[index(fluid, i - 1, j)];
                const float sx1 = fluid.solid_[index(fluid, i + 1, j)];
                const float sy0 = fluid.solid_[index(fluid, i, j - 1)];
                const float sy1 = fluid.solid_[index(fluid, i, j + 1)];
                const float open_faces = sx0 + sx1 + sy0 + sy1;
                if (open_faces == 0.0f)
                {
                    continue;
                }

                // Discrete divergence: net outflow from this cell.
                const float divergence = fluid.u_[index(fluid, i + 1, j)]
                - fluid.u_[index(fluid, i, j)]
                + fluid.v_[index(fluid, i, j + 1)]
                - fluid.v_[index(fluid, i, j)];

                // Apply the same correction to all open faces, weighted
                // by whether the neighboring cell is fluid.
                const float correction = (-divergence / open_faces) * over_relaxation;

                fluid.pressure_[index(fluid, i, j)] += pressure_scale * correction;
                fluid.u_[index(fluid, i, j)] -= sx0 * correction;
                fluid.u_[index(fluid, i + 1, j)] += sx1 * correction;
                fluid.v_[index(fluid, i, j)] -= sy0 * correction;
                fluid.v_[index(fluid, i, j + 1)] += sy1 * correction;
            }
        });
    }
}

void FluidSystem::ExtrapolateBoundaries(Fluid& fluid)
{
    // Copy the nearest interior values into ghost cells to make sampling
    // near the domain boundary well-defined.
    for (int i = 0; i < fluid.nx_; ++i)
    {
        fluid.u_[index(fluid, i, 0)] = fluid.u_[index(fluid, i, 1)];
        fluid.u_[index(fluid, i, fluid.ny_ - 1)] = fluid.u_[index(fluid, i, fluid.ny_ - 2)];
    }
    for (int j = 0; j < fluid.ny_; ++j)
    {
        fluid.v_[index(fluid, 0, j)] = fluid.v_[index(fluid, 1, j)];
        fluid.v_[index(fluid, fluid.nx_ - 1, j)] = fluid.v_[index(fluid, fluid.nx_ - 2, j)];
    }
}

float FluidSystem::SampleField(const Fluid& fluid, float x, float y, const Field field)
{
    // Keep the back-traced point inside the grid's valid sampling region.
    x = std::clamp(x, fluid.h_, static_cast<float>(fluid.nx_) * fluid.h_);
    y = std::clamp(y, fluid.h_, static_cast<float>(fluid.ny_) * fluid.h_);

    float dx = 0.0f;
    float dy = 0.0f;
    const std::vector<float>* values = nullptr;
    switch (field)
    {
        case Field::U:
            values = &fluid.u_;
            dy = 0.5f * fluid.h_;
            break;
        case Field::V:
            values = &fluid.v_;
            dx = 0.5f * fluid.h_;
            break;
        case Field::Smoke:
            values = &fluid.smoke_;
            dx = 0.5f * fluid.h_;
            dy = 0.5f * fluid.h_;
            break;
    }

    const float gx = (x - dx) / fluid.h_;
    const float gy = (y - dy) / fluid.h_;
    const int x0 = std::clamp(static_cast<int>(std::floor(gx)), 0, fluid.nx_ - 1);
    const int y0 = std::clamp(static_cast<int>(std::floor(gy)), 0, fluid.ny_ - 1);
    const int x1 = std::min(x0 + 1, fluid.nx_ - 1);
    const int y1 = std::min(y0 + 1, fluid.ny_ - 1);
    const float tx = gx - static_cast<float>(x0);
    const float ty = gy - static_cast<float>(y0);

    const float a = (*values)[index(fluid, x0, y0)];
    const float b = (*values)[index(fluid, x1, y0)];
    const float c = (*values)[index(fluid, x1, y1)];
    const float d = (*values)[index(fluid, x0, y1)];
    
    return (1.0f - tx) * (1.0f - ty) * a
         + tx * (1.0f - ty) * b
         + tx * ty * c
         + (1.0f - tx) * ty * d;
}

void FluidSystem::AdvectVelocity(Fluid& fluid, const float dt)
{
    fluid.new_u_ = fluid.u_;
    fluid.new_v_ = fluid.v_;
    const float half_h = 0.5f * fluid.h_;

    auto index_range = std::views::iota(int{0}, fluid.nx_ - 1);
    std::for_each(std::execution::par, std::begin(index_range), std::end(index_range), [&](auto i)
    {
        for (int j = 1; j < fluid.ny_; ++j)
        {
            // Advect u, which lives at (i*h, j*h + h/2).
            if (fluid.solid_[index(fluid, i, j)] != 0.0f && fluid.solid_[index(fluid, i - 1, j)] != 0.0f && j < fluid.ny_ - 1)
            {
                float x = static_cast<float>(i) * fluid.h_;
                float y = static_cast<float>(j) * fluid.h_ + half_h;
                const float u = fluid.u_[index(fluid, i, j)];
                const float v = AverageV(fluid, i, j);
                x -= dt * u;
                y -= dt * v;
                fluid.new_u_[index(fluid, i, j)] = SampleField(fluid, x, y, Field::U);
            }

            // Advect v, which lives at (i*h + h/2, j*h).
            if (fluid.solid_[index(fluid, i, j)] != 0.0f && fluid.solid_[index(fluid, i, j - 1)] != 0.0f && i < fluid.nx_ - 1)
            {
                float x = static_cast<float>(i) * fluid.h_ + half_h;
                float y = static_cast<float>(j) * fluid.h_;
                const float u = AverageU(fluid, i, j);
                const float v = fluid.v_[index(fluid, i, j)];
                x -= dt * u;
                y -= dt * v;
                fluid.new_v_[index(fluid, i, j)] = SampleField(fluid, x, y, Field::V);
            }
        }
    });

    fluid.u_.swap(fluid.new_u_);
    fluid.v_.swap(fluid.new_v_);
}

void FluidSystem::AdvectSmoke(Fluid& fluid, const float dt)
{   
    fluid.new_smoke_ = fluid.smoke_;
    const float half_h = 0.5f * fluid.h_;
    
    auto index_range = std::views::iota(int{0}, fluid.nx_ - 1);
    std::for_each(std::execution::par, std::begin(index_range), std::end(index_range), [&](auto i)
    {
        for (int j = 1; j < fluid.ny_ - 1; ++j)
        {
            if (fluid.solid_[index(fluid, i, j)] == 0.0f)
            {
                continue;
            }

            // Cell-centered velocity, obtained by averaging adjacent faces.
            const float u = 0.5f * (fluid.u_[index(fluid, i, j)] + fluid.u_[index(fluid, i + 1, j)]);
            const float v = 0.5f * (fluid.v_[index(fluid, i, j)] + fluid.v_[index(fluid, i, j + 1)]);

            // Semi-Lagrangian backtrace: x_previous = x - dt * velocity.
            const float x = static_cast<float>(i) * fluid.h_ + half_h - dt * u;
            const float y = static_cast<float>(j) * fluid.h_ + half_h - dt * v;
            fluid.new_smoke_[index(fluid, i, j)] = SampleField(fluid, x, y, Field::Smoke);
        }
    });

    fluid.smoke_.swap(fluid.new_smoke_);
}
