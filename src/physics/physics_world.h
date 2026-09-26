#pragma once

#include <memory_resource>
#include <span>
#include <vector>

#include "components.h"
#include "../physics_config.h"

class XpbdSolver;
class CollisionSolverSystem;
class CollisionDetectionSystem;
class MovementSystem;

struct PhysicsProfileData {
    double broad_phase_ms = 0.0;
    double narrow_phase_ms = 0.0;
    double solve_xpbd_constraints_ms = 0.0;
    double solve_si_constraints_ms = 0.0;

    void Reset() {
        broad_phase_ms = 0.0;
        narrow_phase_ms = 0.0;
        solve_xpbd_constraints_ms = 0.0;
        solve_si_constraints_ms = 0.0;
    }
};

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
    std::vector<Aabb> local_aabbs_;
    
    // ===== XPBD Soft Body =====
    std::vector<SoftBodyHandle> soft_body_handles_;
    std::vector<Vec2> particle_positions_;
    std::vector<Vec2> particle_prev_positions_;
    std::vector<Vec2> particle_velocities_;
    std::vector<float> particle_inv_masses_;
    std::vector<Vec2> particle_forces_;
    std::vector<XpbdDistanceConstraint> distance_constraints_;
    std::vector<XpbdContactConstraint> soft_rigid_contacts_;
    std::vector<XpbdTriangleAreaConstraint> triangle_area_constraints_;
    std::vector<XpbdGlobalAreaConstraint> global_area_constraints_;
    
    std::pmr::monotonic_buffer_resource frame_arena_;
    
public:
    // Pre-allocate memory on heap.
    explicit PhysicsWorld(const float gravity = physics_config::kDefaultGravity, const size_t arena_size_bytes = 4 * 1024 * 1024) 
    : frame_arena_(arena_size_bytes), gravity_(gravity){}
    
    BodyHandle CreateBody(const Vec2& pos, float rotation, const Vec2& velocity, float angular_velocity, float mass, float inertia, float restitution = 0.1f, float friction = 0.1f);
    BodyHandle CreateCircle(const Vec2& pos, float rotation, const Vec2& velocity, float angular_velocity, float mass, float radius, float restitution = 0.1f, float friction = 0.1f);
    BodyHandle CreateBox(const Vec2& pos, float rotation, const Vec2& velocity, float angular_velocity, float mass, float half_width, float half_height, float restitution = 0.1f, float friction = 0.1f);
    BodyHandle CreatePolygon(const Vec2& pos, float rotation, const Vec2& velocity, float angular_velocity, float mass, const std::vector<Vec2>& vertices, float restitution = 0.1f, float friction = 0.1f);
    static size_t AddParticle(PhysicsWorld& world, const Vec2& pos, float inv_mass);
    static size_t AddDistanceConstraint(PhysicsWorld& world, size_t p1, size_t p2, float compliance);
    static size_t AddTriangleAreaConstraint(PhysicsWorld& world, size_t p1, size_t p2, size_t p3, float compliance);
    static size_t AddGlobalAreaConstraint(PhysicsWorld& world, size_t start_particle_idx, size_t particle_count, float compliance);
    SoftBodyHandle CreateRope(const Vec2& start, const Vec2& end, size_t segments, float mass, float compliance = 0.0f, bool fix_start = true, bool fix_end = false);
    SoftBodyHandle CreateSoftBox(const Vec2& pos, float width, float height, size_t res_x, size_t res_y, float mass, float compliance);
    SoftBodyHandle CreateSoftCircle(const Vec2& pos, float radius, size_t segments, float mass, float compliance);
    SoftBodyHandle CreatePressureSoftCircle(const Vec2& pos, float radius, size_t segments, float mass, float compliance);
    
    void ApplyForce(BodyHandle handle, const Vec2& force);
    void ApplyForceToAll(const Vec2& force);
    void ApplyGravity();
    void Step(float dt);
    
    std::span<Vec2> GetSoftBodyParticlePositions(size_t i);
    const std::vector<PositionComponent>& GetPositions() { return positions_; }
    const std::vector<RotationComponent>& GetRotations() { return rotations_; }
    const std::vector<Shape>& GetShapes() { return shapes_; }
    const std::vector<Vec2>& GetVertices() { return vertices_; }
    const std::vector<BvhNode>& GetBvhNodes() { return bvh_tree_.nodes_; }
    const std::vector<Manifold>& GetManifolds() { return manifolds_; }
    const std::vector<Vec2>& GetParticlePositions() { return particle_positions_; }
    const std::vector<XpbdDistanceConstraint>& GetDistanceConstraints() { return distance_constraints_; }
    const std::vector<SoftBodyHandle>& GetSoftBodyHandles() { return soft_body_handles_; }
    
    void SetPosition(const BodyHandle handle, const Vec2& pos) { positions_[handle.id].val = pos; }
    void SetAngularVelocity(const BodyHandle handle, const float angular_vel) { angular_velocities_[handle.id].val = angular_vel; }
    void SetRestitution(const BodyHandle handle, const float restitution) { rigid_bodies_[handle.id].restitution = restitution; }
    void SetFriction(const BodyHandle handle, const float friction) { rigid_bodies_[handle.id].friction = friction; }
    
    float gravity_;
    MortonBvh bvh_tree_;
    std::vector<Manifold> manifolds_;
    
    bool show_debug_contact_points_ = true;
    bool show_debug_clipping_ = true;
    std::vector<std::array<Vec2, 2>> debug_incident_edges_;
    std::vector<std::array<Vec2, 2>> debug_reference_edges_;
    PhysicsProfileData profile_data_;
    
    friend MovementSystem;
    friend CollisionDetectionSystem;
    friend CollisionSolverSystem;
    friend XpbdSolver;
    friend MortonBvh;
};
