#include <algorithm>

#include "physics_world.h"
#include "movement_system.h"
#include "collision_solver_system.h"
#include "collision_detection_system.h"

BodyHandle PhysicsWorld::CreateBody(const Vec2& pos, float rotation, const Vec2& velocity, const float angular_velocity, const float mass, const float inertia, float restitution, float friction)
{
    const size_t new_id = positions_.size();
    
    positions_.emplace_back(pos);
    rotations_.emplace_back(rotation);
    velocities_.emplace_back(velocity);
    angular_velocities_.emplace_back(angular_velocity);
    pseudo_velocities_.emplace_back(Vec2{0.f, 0.f});
    pseudo_angular_velocities_.emplace_back(0.f);
    
    float inv_mass = mass == 0.0f ? 0.0f : 1.0f / mass;
    float inv_inertia = inertia == 0.0f ? 0.0f : 1.0f / inertia;
    rigid_bodies_.emplace_back(mass, inv_mass, inertia, inv_inertia, restitution, friction, Vec2{0.0f, 0.0f});
    
    return {new_id};
}

BodyHandle PhysicsWorld::CreateCircle(const Vec2& pos, const float rotation, const Vec2& velocity, const float angular_velocity, const float mass, const float radius, const float restitution, const float friction)
{
    local_aabbs_.emplace_back(-radius, -radius, radius, radius);
    shapes_.push_back(CircleShape{radius});
    const float inertia = 0.5f * mass * radius * radius;
    return CreateBody(pos, rotation, velocity, angular_velocity, mass, inertia, restitution, friction);
}

BodyHandle PhysicsWorld::CreateBox(const Vec2& pos, const float rotation, const Vec2& velocity, const float angular_velocity, const float mass, const float half_width, const float half_height, const float restitution, const float friction)
{
    local_aabbs_.emplace_back(-half_width, -half_height, half_width, half_height);
    shapes_.push_back(BoxShape{half_width, half_height});
    const float width = half_width * 2;
    const float height = half_height * 2;
    const float inertia = (1.0f / 12.0f) * mass * (width * width + height * height);
    return CreateBody(pos, rotation, velocity, angular_velocity, mass, inertia, restitution, friction);
}

BodyHandle PhysicsWorld::CreatePolygon(const Vec2& pos, const float rotation, const Vec2& velocity, const float angular_velocity, const float mass, const std::vector<Vec2>& vertices, const float restitution, const float friction)
{
    float min_x = std::numeric_limits<float>::max();
    float min_y = std::numeric_limits<float>::max();
    float max_x = std::numeric_limits<float>::min();
    float max_y = std::numeric_limits<float>::min();
    for (const Vec2& vertex : vertices)
    {
        min_x = std::min(vertex.x, min_x);
        max_x = std::max(vertex.x, max_x);
        min_y = std::min(vertex.y, min_y);
        max_y = std::max(vertex.y, max_y);
    }
    local_aabbs_.emplace_back(min_x, min_y, max_x, max_y);
    shapes_.push_back(PolygonShape(vertices_.size(), vertices.size()));
    return CreateBody(pos, rotation, velocity, angular_velocity, mass, 1, restitution, friction);
}

void PhysicsWorld::ApplyForce(const BodyHandle handle, const Vec2& force)
{
    rigid_bodies_[handle.id].force += force;
}

void PhysicsWorld::ApplyGravity()
{
    for (auto& rigid_body : rigid_bodies_)
    {
        rigid_body.force += Vec2{0.0f, gravity_} * rigid_body.mass;
    }
}

void PhysicsWorld::Step(const float dt)
{
    constexpr int sub_steps = 4;
    const float sub_dt = dt / static_cast<float>(sub_steps);
    constexpr int velocity_iterations = 2;
    constexpr int position_iterations = 2;
    
    for (int sub = 0; sub < sub_steps; ++sub)
    {
        ApplyGravity();
        MovementSystem::IntegrateVelocity(*this, sub_dt);
        CollisionDetectionSystem::CheckCollision(*this);

        for (auto& manifold : manifolds_)
        {
            for (int i = 0; i < manifold.contact_count; ++i)
            {
                CollisionSolverSystem::InitializeContact(*this, manifold, i);        
            }
        }
        
        for (auto& manifold : manifolds_)
        {
            for (int i = 0; i < manifold.contact_count; ++i)
            {
                CollisionSolverSystem::WarmStartContact(*this, manifold, i);        
            }
        }
        
        for (int iter = 0; iter < velocity_iterations; ++iter)
        {
            for (auto& manifold : manifolds_)
            {
                for (int i = 0; i < manifold.contact_count; ++i)
                {
                    CollisionSolverSystem::SolveVelocityConstraint(*this, manifold, i);        
                }
            }
        }
        
        for (int iter = 0; iter < position_iterations; ++iter)
        {
            for (auto& manifold : manifolds_)
            {
                for (int i = 0; i < manifold.contact_count; ++i)
                {
                    CollisionSolverSystem::SolvePositionConstraint(*this, manifold, i);      
                }
            }
        }

        MovementSystem::IntegratePosition(*this, sub_dt);
    }
}

void PhysicsWorld::SetPosition(const BodyHandle handle, const Vec2& pos)
{
    positions_[handle.id].val = pos;
}

void PhysicsWorld::SetAngularVelocity(const BodyHandle handle, const float angular_vel)
{
    angular_velocities_[handle.id].val = angular_vel;
}

void PhysicsWorld::SetRestitution(const BodyHandle handle, const float restitution)
{
    rigid_bodies_[handle.id].restitution = restitution;
}

void PhysicsWorld::SetFriction(const BodyHandle handle, const float friction)
{
    rigid_bodies_[handle.id].friction = friction;
}
