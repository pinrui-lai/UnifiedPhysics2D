#include <algorithm>

#include "physics/physics_world.h"
#include "physics/collision/collision_broad_phase.h"
#include "physics/systems/movement_system.h"
#include "physics/systems/collision_solver_system.h"
#include "physics/systems/collision_detection_system.h"
#include "physics/systems/xpbd_solver_system.h"

BodyHandle PhysicsWorld::CreateBody(const Vec2& pos, float rotation, const Vec2& velocity, const float angular_velocity, const float mass, const float inertia, float restitution, float friction)
{
    const uint32_t new_id = positions_.size();
    
    positions_.emplace_back(pos);
    rotations_.emplace_back(rotation);
    rots_.emplace_back(std::cos(rotation), std::sin(rotation));
    velocities_.emplace_back(velocity);
    angular_velocities_.emplace_back(angular_velocity);
    forces_.emplace_back(Vec2{0.f, 0.f});
    pseudo_velocities_.emplace_back(Vec2{0.f, 0.f});
    pseudo_angular_velocities_.emplace_back(0.f);
    
    float inv_mass = mass == 0.f ? 0.f : 1.f / mass;
    float inv_inertia = inertia == 0.f ? 0.f : 1.f / inertia;
    rigid_bodies_.emplace_back(inv_mass, inv_inertia, restitution, friction);
    
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
    const float inertia = (1.f / 12.f) * mass * (width * width + height * height);
    return CreateBody(pos, rotation, velocity, angular_velocity, mass, inertia, restitution, friction);
}

BodyHandle PhysicsWorld::CreatePolygon(const Vec2& pos, const float rotation, const Vec2& velocity, const float angular_velocity, const float mass, const std::vector<Vec2>& vertices, const float restitution, const float friction)
{
    float min_x = std::numeric_limits<float>::max();
    float min_y = std::numeric_limits<float>::max();
    float max_x = std::numeric_limits<float>::lowest();
    float max_y = std::numeric_limits<float>::lowest();
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

uint32_t PhysicsWorld::AddParticle(const Vec2& pos, const float inv_mass, const float friction)
{
    const uint32_t idx = particle_positions_.size();
    particle_positions_.push_back(pos);
    particle_prev_positions_.push_back(pos);
    particle_velocities_.push_back(Vec2{0.f, 0.f});
    particle_inv_masses_.push_back(inv_mass);
    particle_forces_.push_back(Vec2{0.f, 0.f});
    particle_frictions_.push_back(friction);
    return idx;
}

uint32_t PhysicsWorld::AddDistanceConstraint(const uint32_t p1, const uint32_t p2, const float compliance)
{
    const uint32_t idx = distance_constraints_.size();
    const float rest_len = (particle_positions_[p1] - particle_positions_[p2]).Length();
    distance_constraints_.emplace_back(p1, p2, rest_len, compliance, 0.f);
    return idx;
}

uint32_t PhysicsWorld::AddTriangleAreaConstraint(uint32_t p1, uint32_t p2, uint32_t p3, float compliance)
{
    const uint32_t idx = triangle_area_constraints_.size();
    const Vec2 pos_p1 = particle_positions_[p1];
    const Vec2 pos_p2 = particle_positions_[p2];
    const Vec2 pos_p3 = particle_positions_[p3];
    float area = 0.5f * (pos_p1.x * (pos_p3.y - pos_p2.y) + pos_p2.x * (pos_p1.y - pos_p3.y) + pos_p3.x * (pos_p2.y - pos_p1.y));
    triangle_area_constraints_.emplace_back(p1, p2, p3, area, compliance, 0.f);
    return idx;
}

uint32_t PhysicsWorld::AddGlobalAreaConstraint(uint32_t start_particle_idx, uint16_t particle_count,
                                               float compliance)
{
    const uint32_t idx = global_area_constraints_.size();
    
    float rest_area = 0.f;
    for (uint32_t i = 0; i < particle_count; ++i)
    {
        const uint32_t curr = start_particle_idx + i;
        const uint32_t next = start_particle_idx + ((i + 1) % particle_count);
        const Vec2& p1 = particle_positions_[curr];
        const Vec2& p2 = particle_positions_[next];
        
        rest_area += (p2.x * p1.y - p1.x * p2.y);
    }
    rest_area *= 0.5f;
    
    global_area_constraints_.emplace_back(start_particle_idx, particle_count, rest_area, compliance, 0.f);
    return idx;
}

uint32_t PhysicsWorld::AddRigidParticleJointConstraint(uint32_t particle_idx, uint32_t rigid_idx, Vec2 anchor_local)
{
    const uint32_t idx = rigid_particle_joint_constraints_.size();
    rigid_particle_joint_constraints_.emplace_back(anchor_local, particle_idx, rigid_idx);
    return idx;
}

void PhysicsWorld::AddSoftSoftCollisionIgnore(const uint32_t body_idx1, const uint32_t body_idx2)
{
    soft_soft_collision_ignore_.insert(CollisionBroadPhase::MakePairKey(body_idx1, body_idx2));
}

void PhysicsWorld::AddRigidSoftCollisionIgnore(const uint32_t rigid_idx, const uint32_t soft_idx)
{
    rigid_soft_collision_ignore_.insert(CollisionBroadPhase::MakePairKey(rigid_idx, soft_idx));
}

SoftBodyHandle PhysicsWorld::CreateRope(const Vec2& start, const Vec2& end, const uint16_t segments, const float mass,
                                        const float friction, const float compliance, const bool fix_start, const bool fix_end)
{
    if (segments == 0) return {};

    SoftBodyHandle handle;
    handle.start_particle_idx = particle_positions_.size();
    handle.particle_count = segments + 1;
    handle.start_surface_edge_idx = surface_edges_.size();
    handle.surface_edge_count = segments;
    handle.start_surface_particle_idx = handle.start_particle_idx;
    handle.surface_particle_count = segments + 1;

    const float particle_mass = mass / static_cast<float>(handle.particle_count);
    const float inv_particle_mass = particle_mass > 0.f ? 1.f / particle_mass : 0.f;
    const Vec2 step = (end - start) / static_cast<float>(segments);

    // Create Particles
    for (uint16_t i = 0; i <= segments; ++i)
    {
        Vec2 pos = start + step * static_cast<float>(i);
        float current_inv_particle_mass = inv_particle_mass;
        if ((i == 0 && fix_start) || (i == segments && fix_end))
        {
            current_inv_particle_mass = 0.f; // Mass of fixed particle is infinity
        }
        AddParticle(pos, current_inv_particle_mass, friction);
    }

    // Create distance constraints
    for (uint16_t i = 0; i < segments; ++i)
    {
        const uint32_t p1 = handle.start_particle_idx + i;
        const uint32_t p2 = handle.start_particle_idx + i + 1;
        AddDistanceConstraint(p1, p2, compliance);
        surface_edges_.emplace_back(p1, p2);
    }

    soft_body_handles_.push_back(handle);
    return handle;
}

SoftBodyHandle PhysicsWorld::CreateSoftBox(const Vec2& pos, const float width, const float height, const uint16_t res_x, const uint16_t res_y,
    const float mass, const float friction, const float compliance)
{
    if (res_x < 2 || res_y < 2) return {};

    SoftBodyHandle handle;
    handle.start_particle_idx = particle_positions_.size();
    handle.particle_count = res_x * res_y;
    handle.start_surface_edge_idx = surface_edges_.size();
    handle.surface_edge_count = (res_x - 1) * 2 + (res_y - 1) * 2;

    // outer particles are arranged continuously at the front of the particles.
    handle.start_surface_particle_idx = handle.start_particle_idx;
    handle.surface_particle_count = res_x * 2 + res_y * 2 - 4;

    const float particle_mass = mass / static_cast<float>(handle.particle_count);
    const float inv_mass = particle_mass > 0.f ? 1.f / particle_mass : 0.f;

    const Vec2 half_extents{width * 0.5f, height * 0.5f};
    const Vec2 min_pos = pos - half_extents;
    const Vec2 step{width / static_cast<float>(res_x - 1), height / static_cast<float>(res_y - 1)};
    
    // Record the actual particle Index corresponding to the (x, y) grid position.
    std::pmr::vector<uint32_t> grid_idx(res_x * res_y, &frame_arena_);

    auto add_and_map_particle = [&](const uint16_t x, const uint16_t y) {
        const Vec2 p_pos = min_pos + Vec2{step.x * static_cast<float>(x), step.y * static_cast<float>(y)};
        const uint32_t current_idx = static_cast<uint32_t>(particle_positions_.size());
        AddParticle(p_pos, inv_mass, friction);
        grid_idx[y * res_x + x] = current_idx;
    };

    // Add outer particles in a clockwise order.
    // Top Row: (0, res_y-1) -> (res_x-1, res_y-1)
    for (uint16_t x = 0; x < res_x; ++x) {
        add_and_map_particle(x, res_y - 1);
    }
    // Right Column: (res_x-1, res_y-2) -> (res_x-1, 0)
    for (int32_t y = res_y - 2; y >= 0; --y) {
        add_and_map_particle(res_x - 1, static_cast<uint16_t>(y));
    }
    // Bottom Row: (res_x-2, 0) -> (0, 0)
    for (int32_t x = res_x - 2; x >= 0; --x) {
        add_and_map_particle(static_cast<uint16_t>(x), 0);
    }
    // Left Column: (0, 1) -> (0, res_y-2)
    for (uint16_t y = 1; y < res_y - 1; ++y) {
        add_and_map_particle(0, y);
    }

    // Inner particles
    for (uint16_t y = 1; y < res_y - 1; ++y) {
        for (uint16_t x = 1; x < res_x - 1; ++x) {
            add_and_map_particle(x, y);
        }
    }

    // Get the index of (x, y) in the particle_positions_
    auto get_idx = [&](const uint16_t x, const uint16_t y) { 
        return grid_idx[y * res_x + x]; 
    };

    // Constraints
    for (uint16_t y = 0; y < res_y - 1; ++y) 
    {
        for (uint16_t x = 0; x < res_x - 1; ++x) 
        {
            const uint32_t p00 = get_idx(x, y);
            const uint32_t p10 = get_idx(x + 1, y);
            const uint32_t p01 = get_idx(x, y + 1);
            const uint32_t p11 = get_idx(x + 1, y + 1);

            // Structural Springs
            if (y == 0){
                AddDistanceConstraint(p00, p10, compliance);
            }
            if (x == 0){
                AddDistanceConstraint(p00, p01, compliance);
            } 
                
            AddDistanceConstraint(p10, p11, compliance);
            AddDistanceConstraint(p01, p11, compliance);

            // Triangle Area Constraints
            AddTriangleAreaConstraint(p00, p11, p10, 0.f); 
            AddTriangleAreaConstraint(p00, p01, p11, 0.f);
        }
    }

    // Surface Edges
    // Top Edges
    for (uint16_t x = 0; x < res_x - 1; ++x) {
        surface_edges_.emplace_back(get_idx(x, res_y - 1), get_idx(x + 1, res_y - 1));
    }
    // Right Edges
    for (int32_t y = res_y - 1; y > 0; --y) {
        surface_edges_.emplace_back(get_idx(res_x - 1, y), get_idx(res_x - 1, y - 1));
    }
    // Bottom Edges
    for (int32_t x = res_x - 1; x > 0; --x) {
        surface_edges_.emplace_back(get_idx(x, 0), get_idx(x - 1, 0));
    }
    // Left Edges
    for (uint16_t y = 0; y < res_y - 1; ++y) {
        surface_edges_.emplace_back(get_idx(0, y), get_idx(0, y + 1));
    }
    
    soft_body_handles_.push_back(handle);
    return handle;
}

SoftBodyHandle PhysicsWorld::CreateSoftCircle(const Vec2& pos, const float radius, const uint16_t segments, const float mass,
                                              const float friction, const float compliance)
{
    if (segments < 3) return {};

    SoftBodyHandle handle;
    handle.start_particle_idx = particle_positions_.size();
    handle.particle_count = segments + 1; // 1 Center + N Outer Ring Particles
    handle.start_surface_edge_idx = surface_edges_.size();
    handle.surface_edge_count = segments;
    handle.start_surface_particle_idx = handle.start_particle_idx + 1;
    handle.surface_particle_count = segments;

    const float particle_mass = mass / static_cast<float>(handle.particle_count);
    const float inv_mass = particle_mass > 0.f ? 1.f / particle_mass : 0.f;

    // Center Particle
    const uint32_t center_idx = AddParticle(pos, inv_mass, friction);

    // Clockwise Outer Ring Particles
    static constexpr float two_pi = 6.28318530718f;
    for (uint16_t i = 0; i < segments; ++i)
    {
        const float angle = -(static_cast<float>(i) * two_pi) / static_cast<float>(segments);
        Vec2 p_pos = pos + Vec2{std::cos(angle) * radius, std::sin(angle) * radius};
        AddParticle(p_pos, inv_mass, friction);
    }

    // Constraints
    for (uint16_t i = 0; i < segments; ++i) 
    {
        const uint32_t curr = handle.start_particle_idx + 1 + i;
        const uint32_t next = handle.start_particle_idx + 1 + ((i + 1) % segments);

        AddDistanceConstraint(handle.start_particle_idx, curr, compliance);
        
        // Outer Ring Distance  Constraint
        AddDistanceConstraint(curr, next, compliance);
        surface_edges_.emplace_back(curr, next);
        
        // Internal Circular Sector Area Constraint (Clockwise)
        AddTriangleAreaConstraint(center_idx, curr, next, 0.f);
    }

    soft_body_handles_.push_back(handle);
    return handle;
}

SoftBodyHandle PhysicsWorld::CreatePressureSoftCircle(const Vec2& pos, const float radius, const uint16_t segments, const float mass,
    const float friction, const float compliance)
{
    if (segments < 3) return {};

    SoftBodyHandle handle;
    handle.start_particle_idx = particle_positions_.size();
    handle.particle_count = segments; // Outer Ring Particles
    handle.start_surface_edge_idx = surface_edges_.size();
    handle.surface_edge_count = segments;
    handle.start_surface_particle_idx = handle.start_particle_idx;
    handle.surface_particle_count = segments;
    
    const float particle_mass = mass / static_cast<float>(handle.particle_count);
    const float inv_mass = particle_mass > 0.f ? 1.f / particle_mass : 0.f;
    
    // Clockwise Outer Ring Particles
    static constexpr float two_pi = 6.28318530718f;
    for (uint16_t i = 0; i < segments; ++i) 
    {
        const float angle = -(static_cast<float>(i) * two_pi) / static_cast<float>(segments);
        Vec2 p_pos = pos + Vec2{std::cos(angle) * radius, std::sin(angle) * radius};
        AddParticle(p_pos, inv_mass, friction);
    }
    
    // Constraints
    for (uint16_t i = 0; i < segments; ++i) 
    {
        const uint32_t curr = handle.start_particle_idx + i;
        const uint32_t next = handle.start_particle_idx + ((i + 1) % segments);
        surface_edges_.emplace_back(curr, next);

        // Outer Ring Distance Constraint
        AddDistanceConstraint(curr, next, compliance);
    }
    
    // Internal Global Area Constraint
    AddGlobalAreaConstraint(handle.start_particle_idx, handle.particle_count, 0.f);
    
    soft_body_handles_.push_back(handle);
    return handle;
}

void PhysicsWorld::CreateCar(const Vec2& pos, float wheel_radius, float wheel_mass, float wheel_friction, float wheel_restitution,
                             float absorber_height, float absorber_mass, float absorber_compliance,
                             float car_body_width, float car_body_height, float car_body_mass, float car_body_restitution, float car_body_friction)
{
    // Left Wheel
    const uint32_t left_wheel_idx = positions_.size();
    CreateCircle(pos, 0.f, {0.f, 0.f}, 0.f, wheel_mass, wheel_radius, wheel_restitution, wheel_friction);
    
    // Right Wheel
    const uint32_t right_wheel_idx = positions_.size();
    CreateCircle({pos.x + car_body_width, pos.y}, 0.f, {0.f, 0.f}, 0.f, wheel_mass, wheel_radius, wheel_restitution, wheel_friction);
    
    // Left Absorber
    Vec2 left_wheel_center_pos = positions_[left_wheel_idx].val;
    uint32_t left_absorber_upper_particle_idx = particle_positions_.size();
    const uint32_t left_absorber_lower_particle_idx = particle_positions_.size() + 1;
    const uint32_t left_absorber_handle_idx = soft_body_handles_.size();
    CreateRope({left_wheel_center_pos.x, left_wheel_center_pos.y + absorber_height}, left_wheel_center_pos, 
        1, absorber_mass, 0.f, absorber_compliance, false, false);
    
    // Right Absorber
    Vec2 right_wheel_center_pos = positions_[right_wheel_idx].val;
    uint32_t right_absorber_upper_particle_idx = particle_positions_.size();
    const uint32_t right_absorber_lower_particle_idx = particle_positions_.size() + 1;
    const uint32_t right_absorber_handle_idx = soft_body_handles_.size();
    CreateRope({right_wheel_center_pos.x, right_wheel_center_pos.y + absorber_height}, right_wheel_center_pos, 
        1, absorber_mass, 0.f, absorber_compliance, false, false);
    
    // Car Body
    uint32_t car_body_idx = positions_.size();
    CreateBox({pos.x + car_body_width * 0.5f, pos.y + absorber_height + car_body_height * 0.5f}, 0.f,
{0.f, 0.f}, 0.f,  car_body_mass, car_body_width * 0.5f, car_body_height * 0.5f, car_body_restitution, car_body_friction);
    
    // Absorber-Wheel Joint Constraints
    AddRigidParticleJointConstraint(left_absorber_lower_particle_idx, left_wheel_idx, {0.f, 0.f});
    AddRigidParticleJointConstraint(right_absorber_lower_particle_idx, right_wheel_idx, {0.f, 0.f});
    
    // Absorber-CarBody Joint Constraints
    Vec2 left_anchor{-car_body_width * 0.5f, -car_body_height * 0.5f};
    Vec2 right_anchor{car_body_width * 0.5f, -car_body_height * 0.5f};
    rigid_particle_joint_constraints_.emplace_back(left_anchor, left_absorber_upper_particle_idx, car_body_idx);
    rigid_particle_joint_constraints_.emplace_back(right_anchor, right_absorber_upper_particle_idx, car_body_idx);
    
    // Absorber-CarBody Angle Constraints
    rigid_particle_angular_constraints_.emplace_back(Vec2{0.f, -1.f}, left_anchor, left_absorber_lower_particle_idx, car_body_idx, 1e-6f);
    rigid_particle_angular_constraints_.emplace_back(Vec2{0.f, -1.f}, right_anchor, right_absorber_lower_particle_idx, car_body_idx, 1e-6f);
    
    // Add collision ignore
    rigid_soft_collision_ignore_.insert(CollisionBroadPhase::MakePairKey(left_wheel_idx, left_absorber_handle_idx));
    rigid_soft_collision_ignore_.insert(CollisionBroadPhase::MakePairKey(right_wheel_idx, right_absorber_handle_idx));
    rigid_soft_collision_ignore_.insert(CollisionBroadPhase::MakePairKey(car_body_idx, left_absorber_handle_idx));
    rigid_soft_collision_ignore_.insert(CollisionBroadPhase::MakePairKey(car_body_idx, right_absorber_handle_idx));
}

void PhysicsWorld::ApplyForce(const BodyHandle handle, const Vec2& force)
{
    forces_[handle.id] += force;
}

void PhysicsWorld::ApplyForceToAll(const Vec2& force)
{
    for (int i = 0; i < rigid_bodies_.size(); ++i)
    {
        forces_[i] += force;
    }
    
    for (const auto& soft_body_handle : soft_body_handles_)
    {
        const Vec2 force_per_particle = force / soft_body_handle.particle_count;
        for (uint16_t i = 0; i < soft_body_handle.particle_count; ++i)
        {
            particle_forces_[soft_body_handle.start_particle_idx + i] += force_per_particle;
        }
    }
}

void PhysicsWorld::ApplyGravity()
{
    for (int i = 0; i < rigid_bodies_.size(); ++i)
    {
        forces_[i] += Vec2{0.f, gravity_} / rigid_bodies_[i].inv_mass;
    }
    
    for (int i = 0; i < particle_forces_.size(); ++i)
    {
        particle_forces_[i] += Vec2{0.f, gravity_} / particle_inv_masses_[i];
    }
}

void PhysicsWorld::Step(const float dt)
{
    using Clock = std::chrono::high_resolution_clock;
    using Duration = std::chrono::duration<double, std::milli>;
    profile_data_.Reset();
    
    constexpr int sub_steps = 4;
    const float sub_dt = dt / static_cast<float>(sub_steps);
    const float inv_sub_dt = 1.f / sub_dt;
    const float inv_sdt_sq = 1.f / (sub_dt * sub_dt);
    constexpr int velocity_iterations = 2;
    constexpr int position_iterations = 2;
    constexpr int xpbd_iterations = 2;
    
    for (int sub_step = 0; sub_step < sub_steps; ++sub_step)
    {
        frame_arena_.release();
        rigid_particle_collision_constraints_.clear();
        particle_edge_collision_constraints_.clear();
        
        ApplyGravity();
        
        auto start = Clock::now();
        MovementSystem::IntegrateVelocity(*this, sub_dt);
        profile_data_.integrate_rigid_vel += Duration(Clock::now() - start).count();
        
        start = Clock::now();
        XpbdSolver::PredictParticles(*this, sub_dt);
        profile_data_.predict_particle_pos += Duration(Clock::now() - start).count();
        
        CollisionDetectionSystem::CheckCollision(*this);

        start = Clock::now();
        for (auto& distance_constraint : distance_constraints_)
        {
            distance_constraint.lambda = 0.f;
        }
        for (auto& triangle_area_constraint : triangle_area_constraints_)
        {
            triangle_area_constraint.lambda = 0.f;
        }
        for (auto& global_area_constraint : global_area_constraints_)
        {
            global_area_constraint.lambda = 0.f;
        }
        for (auto& rigid_particle_angle_constraint : rigid_particle_angular_constraints_)
        {
            rigid_particle_angle_constraint.lambda = 0.f;
        }
        
        for (int iter = 0; iter < xpbd_iterations; ++iter)
        {
            if (iter % 2 == 0) 
            {
                for (auto& c : distance_constraints_) 
                {
                    XpbdSolver::SolveDistanceConstraint(*this, inv_sdt_sq, c);
                }
                for (auto& c : triangle_area_constraints_) 
                {
                    XpbdSolver::SolveTriangleAreaConstraint(*this, inv_sdt_sq, c);
                }
            } 
            else 
            {
                for (int i = distance_constraints_.size()-1; i >= 0; --i) 
                {
                    auto& c = distance_constraints_[i];
                    XpbdSolver::SolveDistanceConstraint(*this, inv_sdt_sq, c);
                }
                for (int i = triangle_area_constraints_.size()-1; i >= 0; --i) 
                {
                    auto& c = triangle_area_constraints_[i];
                    XpbdSolver::SolveTriangleAreaConstraint(*this, inv_sdt_sq, c);
                }
            }
            XpbdSolver::SolveGlobalAreaConstraints(*this, inv_sdt_sq);
            XpbdSolver::SolveRigidParticleAngularConstraints(*this, inv_sub_dt);
            XpbdSolver::SolveRigidParticleJointConstraints(*this, inv_sub_dt);
            XpbdSolver::SolveRigidParticleCollisionConstraints(*this, sub_dt, inv_sub_dt);
            XpbdSolver::SolveParticleEdgeCollisionConstraints(*this);
        }
        profile_data_.solve_xpbd_constraints_ms += Duration(Clock::now() - start).count();
        
        start = Clock::now();
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
        profile_data_.solve_si_constraints_ms += Duration(Clock::now() - start).count();

        start = Clock::now();
        MovementSystem::IntegratePosition(*this, sub_dt);
        profile_data_.integrate_rigid_pos += Duration(Clock::now() - start).count();
        
        start = Clock::now();
        XpbdSolver::FinalizeParticles(*this, 1.f/sub_dt);
        profile_data_.update_particle_vel += Duration(Clock::now() - start).count();
        
    }
}

std::span<Vec2> PhysicsWorld::GetSoftBodyParticlePositions(const uint32_t i)
{
    const SoftBodyHandle& soft_body_handle = soft_body_handles_[i];
    return std::span(particle_positions_).subspan(soft_body_handle.start_particle_idx, soft_body_handle.particle_count);
}

std::array<Vec2, 4> PhysicsWorld::GetBodyVertices(const BoxShape& box_shape, uint32_t i)
{
    // Vertices are arranged in clockwise.
    std::array<Vec2, 4> vertices = {
        Vec2{-box_shape.half_width, box_shape.half_height}, Vec2{box_shape.half_width, box_shape.half_height},
        Vec2{box_shape.half_width, -box_shape.half_height}, Vec2{-box_shape.half_width, -box_shape.half_height}
    };
    
    std::ranges::transform(vertices, vertices.begin(), [&](const Vec2 v){ return rots_[i].Mul(v) + positions_[i].val; });
    
    return vertices;
}

std::vector<Vec2> PhysicsWorld::GetBodyVertices(const PolygonShape& polygon_shape, uint32_t i)
{
    return std::vector<Vec2>{};
}

void PhysicsWorld::ClearWorld()
{
    rigid_bodies_.clear();
    forces_.clear();
    positions_.clear();
    rotations_.clear();
    rots_.clear();
    velocities_.clear();
    angular_velocities_.clear();
    pseudo_velocities_.clear();
    pseudo_angular_velocities_.clear();
    shapes_.clear();
    vertices_.clear();
    local_aabbs_.clear();
    soft_body_handles_.clear();
    particle_positions_.clear();
    particle_prev_positions_.clear();
    particle_velocities_.clear();
    particle_inv_masses_.clear();
    particle_frictions_.clear();
    particle_forces_.clear();
    surface_edges_.clear();
    distance_constraints_.clear();
    triangle_area_constraints_.clear();
    global_area_constraints_.clear();
    bvh_tree_root_index_ = std::numeric_limits<uint32_t>::max();
    bvh_tree_nodes_.clear();
    manifolds_.clear();
    manifold_map_.clear();
    rigid_particle_joint_constraints_.clear();
    rigid_particle_angular_constraints_.clear();
    rigid_soft_collision_ignore_.clear();
    soft_soft_collision_ignore_.clear();
}
