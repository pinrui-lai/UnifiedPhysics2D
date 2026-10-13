#pragma once
#include <memory_resource>
#include <span>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "components.h"
#include "physics_config.h"

class CollisionNarrowPhase;
class CollisionBroadPhase;
class XpbdSolver;
class CollisionSolverSystem;
class CollisionDetectionSystem;
class MovementSystem;

struct PhysicsProfileData {
    double integrate_rigid_vel = 0.0;
    double predict_particle_pos = 0.0;
    
    double broad_phase_ms = 0.0;
    double build_bvh_ms = 0.0;
    double query_bvh_ms = 0.0;
    double erase_manifold_ms = 0.0;
    
    double narrow_phase_ms = 0.0;
    double rigid_rigid_ms = 0.0;
    double rigid_soft_ms = 0.0;
    double soft_soft_ms = 0.0;
    
    double solve_xpbd_constraints_ms = 0.0;
    double solve_si_constraints_ms = 0.0;
    
    double integrate_rigid_pos = 0.0;
    double update_particle_vel = 0.0;

    void Reset() {
        integrate_rigid_vel = 0.0;
        predict_particle_pos = 0.0;
        
        broad_phase_ms = 0.0;
        build_bvh_ms = 0.0;
        query_bvh_ms = 0.0;
        erase_manifold_ms = 0.0;
        
        narrow_phase_ms = 0.0;
        rigid_rigid_ms = 0.0;
        rigid_soft_ms = 0.0;
        soft_soft_ms = 0.0;
        
        solve_xpbd_constraints_ms = 0.0;
        solve_si_constraints_ms = 0.0;
        
        integrate_rigid_pos = 0.0;
        update_particle_vel = 0.0;
    }
};

class PhysicsWorld
{
private:
    std::vector<RigidBodyComponent> rigid_bodies_;
    std::vector<Vec2> forces_;
    std::vector<PositionComponent> positions_;
    std::vector<RotationComponent> rotations_;
    std::vector<Rot> rots_;
    std::vector<LinearVelocityComponent> velocities_;
    std::vector<AngularVelocityComponent> angular_velocities_;
    std::vector<PseudoLinearVelocityComponent> pseudo_velocities_;
    std::vector<PseudoAngularVelocityComponent> pseudo_angular_velocities_;
    std::vector<Shape> shapes_;
    std::vector<Vec2> vertices_;
    std::vector<Aabb> local_aabbs_;
    
    // ===== XPBD Soft Body =====
    std::vector<SoftBodyHandle> soft_body_handles_;
    std::vector<Vec2> particle_positions_;
    std::vector<Vec2> particle_prev_positions_;
    std::vector<Vec2> particle_velocities_;
    std::vector<float> particle_inv_masses_;
    std::vector<float> particle_frictions_;
    std::vector<Vec2> particle_forces_;
    std::vector<std::pair<uint32_t, uint32_t>> surface_edges_;
    std::vector<XpbdDistanceConstraint> distance_constraints_;
    std::vector<XpbdTriangleAreaConstraint> triangle_area_constraints_;
    std::vector<XpbdGlobalAreaConstraint> global_area_constraints_;
    std::vector<XpbdRigidParticleCollisionConstraint> rigid_particle_collision_constraints_;
    std::vector<XpbdParticleEdgeCollisionConstraint> particle_edge_collision_constraints_;
    std::vector<XpbdRigidParticleJointConstraint> rigid_particle_joint_constraints_;
    std::vector<XpbdRigidParticleAngularConstraint> rigid_particle_angular_constraints_;
    std::vector<Aabb> particle_aabbs_;
    std::vector<Aabb> surface_edge_aabbs_;
    std::vector<std::pmr::vector<BvhNode>> soft_body_surface_particle_bvhs_;
    std::unordered_set<uint64_t> rigid_soft_collision_ignore_;
    std::unordered_set<uint64_t> soft_soft_collision_ignore_;
    
    std::vector<std::byte> buffer_;
    std::pmr::monotonic_buffer_resource frame_arena_;
    
public:
    // Pre-allocate memory on heap.
    explicit PhysicsWorld(const float gravity = physics_config::kDefaultGravity, const size_t arena_size_bytes = 4 * 1024 * 1024) 
    : buffer_(arena_size_bytes), frame_arena_(buffer_.data(), sizeof(buffer_)), gravity_(gravity){}
    
    BodyHandle CreateBody(const Vec2& pos, float rotation, const Vec2& velocity, float angular_velocity, float mass, float inertia, float restitution = 0.1f, float friction = 0.1f);
    BodyHandle CreateCircle(const Vec2& pos, float rotation, const Vec2& velocity, float angular_velocity, float mass, float radius, float restitution = 0.1f, float friction = 0.1f);
    BodyHandle CreateBox(const Vec2& pos, float rotation, const Vec2& velocity, float angular_velocity, float mass, float half_width, float half_height, float restitution = 0.1f, float friction = 0.1f);
    BodyHandle CreatePolygon(const Vec2& pos, float rotation, const Vec2& velocity, float angular_velocity, float mass, const std::vector<Vec2>& vertices, float restitution = 0.1f, float friction = 0.1f);
    uint32_t AddParticle(const Vec2& pos, float inv_mass, float friction);
    uint32_t AddDistanceConstraint(uint32_t p1, uint32_t p2, float compliance);
    uint32_t AddTriangleAreaConstraint(uint32_t p1, uint32_t p2, uint32_t p3, float compliance);
    uint32_t AddGlobalAreaConstraint(uint32_t start_particle_idx, uint16_t particle_count, float compliance);
    uint32_t AddRigidParticleJointConstraint(uint32_t particle_idx, uint32_t rigid_idx, Vec2 anchor_local);
    void AddSoftSoftCollisionIgnore(uint32_t body_idx1, uint32_t body_idx2);
    void AddRigidSoftCollisionIgnore(uint32_t rigid_idx, uint32_t soft_idx);
    SoftBodyHandle CreateRope(const Vec2& start, const Vec2& end, uint16_t segments, float mass, float friction, float compliance = 0.f, bool fix_start = true, bool fix_end = false);
    SoftBodyHandle CreateSoftBox(const Vec2& pos, float width, float height, uint16_t res_x, uint16_t res_y, float mass, float friction, float compliance);
    SoftBodyHandle CreateSoftCircle(const Vec2& pos, float radius, uint16_t segments, float mass, float friction, float compliance);
    SoftBodyHandle CreatePressureSoftCircle(const Vec2& pos, float radius, uint16_t segments, float mass, float friction, float compliance);
    void CreateCar(const Vec2& pos, float wheel_radius, float wheel_mass, float wheel_friction, float wheel_restitution,
                   float suspension_height, float suspension_mass, float suspension_compliance,
                   float car_body_width, float car_body_height, float car_body_mass, float car_body_restitution, float car_body_friction);
    
    void ApplyForce(BodyHandle handle, const Vec2& force);
    void ApplyForceToAll(const Vec2& force);
    void ApplyGravity();
    void Step(float dt);
    
    std::span<Vec2> GetSoftBodyParticlePositions(uint32_t i);
    const std::vector<PositionComponent>& GetPositions() { return positions_; }
    const std::vector<Rot>& GetRots() { return rots_; }
    const std::vector<Shape>& GetShapes() { return shapes_; }
    const std::vector<Vec2>& GetVertices() { return vertices_; }
    std::span<BvhNode> GetBvhNodes() { return bvh_tree_nodes_; }
    const std::vector<Manifold>& GetManifolds() { return manifolds_; }
    const std::vector<Vec2>& GetParticlePositions() { return particle_positions_; }
    const std::vector<XpbdDistanceConstraint>& GetDistanceConstraints() { return distance_constraints_; }
    const std::vector<SoftBodyHandle>& GetSoftBodyHandles() { return soft_body_handles_; }
    auto GetParticleRigidConstraintsSize() const { return rigid_particle_collision_constraints_.size(); }
    auto GetParticleEdgeConstraintsSize() const { return particle_edge_collision_constraints_.size(); }
    std::array<Vec2, 4> GetBodyVertices(const BoxShape& box_shape, uint32_t i);
    std::vector<Vec2> GetBodyVertices(const PolygonShape& polygon_shape, uint32_t i);
    const std::vector<std::pmr::vector<BvhNode>>& GetSubBvhs(){return soft_body_surface_particle_bvhs_;}
    
    void SetPosition(const BodyHandle handle, const Vec2& pos) { positions_[handle.id].val = pos; }
    void SetAngularVelocity(const BodyHandle handle, const float angular_vel) { angular_velocities_[handle.id].val = angular_vel; }
    void SetRestitution(const BodyHandle handle, const float restitution) { rigid_bodies_[handle.id].restitution = restitution; }
    void SetFriction(const BodyHandle handle, const float friction) { rigid_bodies_[handle.id].friction = friction; }
    
    void ClearWorld();
    
    float gravity_;
    std::vector<BvhNode> bvh_tree_nodes_;
    uint32_t bvh_tree_root_index_{std::numeric_limits<uint32_t>::max()};
    std::vector<Manifold> manifolds_;
    std::unordered_map<uint64_t, uint32_t> manifold_map_;
    
    bool show_debug_contact_points_ = true;
    bool show_debug_clipping_ = false;
    bool is_multi_thread_{true};
    std::vector<std::array<Vec2, 2>> debug_incident_edges_;
    std::vector<std::array<Vec2, 2>> debug_reference_edges_;
    PhysicsProfileData profile_data_;
    
    friend MovementSystem;
    friend CollisionDetectionSystem;
    friend CollisionBroadPhase;
    friend CollisionNarrowPhase;
    friend CollisionSolverSystem;
    friend XpbdSolver;
};
