#include "physics_world.h"

BodyHandle PhysicsWorld::CreateBody(const Vec2& pos, float rotation, const Vec2& velocity, const float angular_velocity, const float mass, const float inertia, float restitution, float friction)
{
    size_t new_id = positions_.size();
    
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

BodyHandle PhysicsWorld::CreateCircle(const Vec2& pos, float rotation, const Vec2& velocity, const float angular_velocity, const float mass, const float radius, const float restitution, const float friction)
{
    shapes_.push_back(CircleShape{radius});
    float inertia = 0.5f * mass * radius * radius;
    return CreateBody(pos, rotation, velocity, angular_velocity, mass, inertia, restitution, friction);
}

BodyHandle PhysicsWorld::CreateBox(const Vec2& pos, float rotation, const Vec2& velocity, const float angular_velocity, const float mass, const float half_width, const float half_height, const float restitution, const float friction)
{
    shapes_.push_back(BoxShape{half_width, half_height});
    const float width = half_width * 2;
    const float height = half_height * 2;
    const float inertia = (1.0f / 12.0f) * mass * (width * width + height * height);
    return CreateBody(pos, rotation, velocity, angular_velocity, mass, inertia, restitution, friction);
}

BodyHandle PhysicsWorld::CreatePolygon(const Vec2& pos, float rotation, const Vec2& velocity, const float angular_velocity, const float mass, const std::vector<Vec2>& vertices, const float restitution, const float friction)
{
    shapes_.push_back(PolygonShape(vertices_.size(), vertices.size()));
    return CreateBody(pos, rotation, velocity, angular_velocity, mass, 1, restitution, friction);
}

void PhysicsWorld::ApplyForce(const BodyHandle handle, const Vec2& force)
{
    rigid_bodies_[handle.id].force += force;
}

void PhysicsWorld::SetPosition(const BodyHandle handle, const Vec2& pos)
{
    positions_[handle.id].val = pos;
}
