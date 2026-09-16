#pragma once

#include <memory_resource>
#include <vector>

#include "components.h"
#include "../physics_config.h"

class MovementSystem;
class CollisionSystem;

class PhysicsWorld
{
private:
    std::vector<RigidBodyComponent> rigid_bodies_;
    std::vector<PositionComponent> positions_;
    std::vector<LinearVelocityComponent> velocities_;
    std::vector<AngularVelocityComponent> angular_velocities_;
    std::vector<PseudoLinearVelocityComponent> pseudo_velocities_;
    std::vector<PseudoAngularVelocityComponent> pseudo_angular_velocities_;
    std::vector<RotationComponent> rotations_;
    std::vector<Shape> shapes_;
    std::vector<Vec2> vertices_;

    std::pmr::monotonic_buffer_resource frame_arena_;
    
public:
    // Pre-allocate memory on heap.
    explicit PhysicsWorld(const float gravity = physics_config::kDefaultGravity, const size_t arena_size_bytes = 4 * 1024 * 1024) 
    : frame_arena_(arena_size_bytes), gravity_(gravity){}
    
    BodyHandle CreateBody(const Vec2& pos, float rotation, const Vec2& velocity, float angular_velocity, float mass, float inertia, float restitution = 0.1f, float friction = 0.1f);
    BodyHandle CreateCircle(const Vec2& pos, float rotation, const Vec2& velocity, float angular_velocity, float mass, float radius, float restitution = 0.1f, float friction = 0.1f);
    BodyHandle CreateBox(const Vec2& pos, float rotation, const Vec2& velocity, float angular_velocity, float mass, float half_width, float half_height, float restitution = 0.1f, float friction = 0.1f);
    BodyHandle CreatePolygon(const Vec2& pos, float rotation, const Vec2& velocity, float angular_velocity, float mass, const std::vector<Vec2>& vertices, float restitution = 0.1f, float friction = 0.1f);
    void ApplyForce(BodyHandle handle, const Vec2& force);
    
    std::vector<PositionComponent>& GetPositions() { return positions_; }
    std::vector<RotationComponent>& GetRotations() { return rotations_; }
    std::vector<Shape>& GetShapes() { return shapes_; }
    std::vector<Vec2>& GetVertices() { return vertices_; }
    
    void SetPosition(BodyHandle handle, const Vec2& pos);
    void SetAngularVelocity(BodyHandle handle, float angular_vel);
    void SetRestitution(BodyHandle handle, float restitution);
    void SetFriction(BodyHandle handle, float friction);
    
    bool show_debug_contact_points_ = true;
    bool show_debug_clipping_ = true;
    std::vector<Contact> debug_contacts_;
    std::vector<std::array<Vec2, 2>> debug_incident_edges_;
    std::vector<std::array<Vec2, 2>> debug_reference_edges_;
    
    float gravity_;
    
    friend MovementSystem;
    friend CollisionSystem;
};
