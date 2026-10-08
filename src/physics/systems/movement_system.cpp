#include <algorithm>
#include <execution>
#include <ranges>

#include "physics_world.h"
#include "movement_system.h"

void MovementSystem::IntegrateVelocity(PhysicsWorld& world, const float dt)
{
    const uint32_t count = world.positions_.size();
    
    auto index_range = std::views::iota(uint32_t{0}, count);
    std::for_each(std::execution::par_unseq, index_range.begin(), index_range.end(),[&](auto i)
    {
        if (world.rigid_bodies_[i].inv_mass == 0.0f) return;
        
        world.velocities_[i].val += (world.forces_[i] * world.rigid_bodies_[i].inv_mass) * dt;
        world.forces_[i] = {0.f, 0.f};
    });
}

void MovementSystem::IntegratePosition(PhysicsWorld& world, const float dt)
{
    const uint32_t count = world.positions_.size();

    auto index_range = std::views::iota(uint32_t{0}, count);
    std::for_each(std::execution::par_unseq, index_range.begin(), index_range.end(),[&](auto i)
    {
        // Split Impulse : use both velocity and  pseudo-velocity to update position.
        // Pseudo Velocities don't use "delta time" to integrate.
        world.positions_[i].val += world.velocities_[i].val * dt + world.pseudo_velocities_[i].val;
        world.rotations_[i].val += world.angular_velocities_[i].val * dt + world.pseudo_angular_velocities_[i].val;
        world.rots_[i].Set(world.rotations_[i].val);
        
        // Clear pseudo velocity to avoid interfering in the next frame.
        world.pseudo_velocities_[i].val = {0.f, 0.f};
        world.pseudo_angular_velocities_[i].val = 0.f;
    });
}

