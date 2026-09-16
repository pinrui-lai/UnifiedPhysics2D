#include <algorithm>
#include <execution>

#include "physics_world.h"
#include "movement_system.h"

void MovementSystem::IntegrateVelocity(PhysicsWorld& world, const float dt)
{
    const size_t count = world.positions_.size();
    
    for (size_t i = 0; i < count; ++i)
    {
        if (world.rigid_bodies_[i].mass == 0.0f) continue;
        
        world.velocities_[i].val += (world.rigid_bodies_[i].force * world.rigid_bodies_[i].inv_mass) * dt;
        world.rigid_bodies_[i].force = {0.f, 0.f};
    }
}

void MovementSystem::IntegratePosition(PhysicsWorld& world, const float dt)
{
    const size_t count = world.positions_.size();
    for (size_t i = 0; i < count; ++i)
    {
        // Split Impulse : use both velocity and  pseudo-velocity to update position.
        world.positions_[i].val += world.velocities_[i].val * dt;
        world.rotations_[i].val += world.angular_velocities_[i].val * dt;
        
        // Pseudo Velocities don't use "delta time" to integrate.
        world.positions_[i].val += world.pseudo_velocities_[i].val;
        world.rotations_[i].val += world.pseudo_angular_velocities_[i].val;
        
        // Clear pseudo velocity to avoid interfering in the next frame.
        world.pseudo_velocities_[i].val = {0.f, 0.f};
        world.pseudo_angular_velocities_[i].val = 0.f;
    }
}

void MovementSystem::ApplyGravity(PhysicsWorld& world)
{
    for (auto& rigid_body : world.rigid_bodies_)
    {
        rigid_body.force += Vec2{0.0f, world.gravity_} * rigid_body.mass;
    }
}
