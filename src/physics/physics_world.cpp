#include <algorithm>
#include <unordered_set>

#include "physics_world.h"
#include "movement_system.h"
#include "collision_solver_system.h"
#include "collision_detection_system.h"
#include "xpbd_solver_system.h"

BodyHandle PhysicsWorld::CreateBody(const Vec2& pos, float rotation, const Vec2& velocity, const float angular_velocity, const float mass, const float inertia, float restitution, float friction)
{
    const uint32_t new_id = positions_.size();
    
    positions_.emplace_back(pos);
    rotations_.emplace_back(rotation);
    rots_.emplace_back(std::cos(rotation), std::sin(rotation));
    velocities_.emplace_back(velocity);
    angular_velocities_.emplace_back(angular_velocity);
    forces_.emplace_back(Vec2{0.0f, 0.0f});
    pseudo_velocities_.emplace_back(Vec2{0.f, 0.f});
    pseudo_angular_velocities_.emplace_back(0.f);
    
    float inv_mass = mass == 0.0f ? 0.0f : 1.0f / mass;
    float inv_inertia = inertia == 0.0f ? 0.0f : 1.0f / inertia;
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

uint32_t PhysicsWorld::AddParticle(PhysicsWorld& world, const Vec2& pos, const float inv_mass, const float friction)
{
    const uint32_t idx = world.particle_positions_.size();
    world.particle_positions_.push_back(pos);
    world.particle_prev_positions_.push_back(pos);
    world.particle_velocities_.push_back(Vec2{0.0f, 0.0f});
    world.particle_inv_masses_.push_back(inv_mass);
    world.particle_forces_.push_back(Vec2{0.0f, 0.0f});
    world.particle_frictions_.push_back(friction);
    return idx;
}

uint32_t PhysicsWorld::AddDistanceConstraint(PhysicsWorld& world, const uint32_t p1, const uint32_t p2, const float compliance)
{
    const uint32_t idx = world.distance_constraints_.size();
    const float rest_len = (world.particle_positions_[p1] - world.particle_positions_[p2]).Length();
    world.distance_constraints_.emplace_back(p1, p2, rest_len, compliance, 0.0f);
    return idx;
}

uint32_t PhysicsWorld::AddTriangleAreaConstraint(PhysicsWorld& world, uint32_t p1, uint32_t p2, uint32_t p3, float compliance)
{
    const uint32_t idx = world.triangle_area_constraints_.size();
    const Vec2 pos_p1 = world.particle_positions_[p1];
    const Vec2 pos_p2 = world.particle_positions_[p2];
    const Vec2 pos_p3 = world.particle_positions_[p3];
    float area = 0.5f * (pos_p1.x * (pos_p3.y - pos_p2.y) + pos_p2.x * (pos_p1.y - pos_p3.y) + pos_p3.x * (pos_p2.y - pos_p1.y));
    world.triangle_area_constraints_.emplace_back(p1, p2, p3, area, compliance, 0.f);
    return idx;
}

uint32_t PhysicsWorld::AddGlobalAreaConstraint(PhysicsWorld& world, uint32_t start_particle_idx, uint16_t particle_count,
    float compliance)
{
    const uint32_t idx = world.global_area_constraints_.size();
    
    float rest_area = 0.0f;
    for (uint32_t i = 0; i < particle_count; ++i)
    {
        const uint32_t curr = start_particle_idx + i;
        const uint32_t next = start_particle_idx + ((i + 1) % particle_count);
        const Vec2& p1 = world.particle_positions_[curr];
        const Vec2& p2 = world.particle_positions_[next];
        
        rest_area += (p2.x * p1.y - p1.x * p2.y);
    }
    rest_area *= 0.5f;
    
    world.global_area_constraints_.emplace_back(start_particle_idx, particle_count, rest_area, compliance, 0.0f);
    return idx;
}

SoftBodyHandle PhysicsWorld::CreateRope(const Vec2& start, const Vec2& end, const uint16_t segments, const float mass,
                                        const float friction, const float compliance, const bool fix_start, const bool fix_end)
{
    if (segments == 0) return {};

    SoftBodyHandle handle;
    handle.start_particle_idx = particle_positions_.size();
    handle.particle_count = segments + 1;
    handle.start_edge_idx = surface_edges_.size();
    handle.edge_count = segments;
    handle.start_surface_particle_idx = surface_particles_idx_.size();
    handle.surface_particle_count = segments + 1;

    const float particle_mass = mass / static_cast<float>(handle.particle_count);
    const float inv_particle_mass = particle_mass > 0.0f ? 1.0f / particle_mass : 0.0f;
    const Vec2 step = (end - start) / static_cast<float>(segments);

    // Create Particles
    for (uint16_t i = 0; i <= segments; ++i)
    {
        Vec2 pos = start + step * static_cast<float>(i);
        float current_inv_particle_mass = inv_particle_mass;
        if ((i == 0 && fix_start) || (i == segments && fix_end))
        {
            current_inv_particle_mass = 0.0f; // Mass of fixed particle is infinity
        }
        AddParticle(*this, pos, current_inv_particle_mass, friction);
        
        surface_particles_idx_.push_back(handle.start_particle_idx + i);
    }

    // Create distance constraints
    for (uint16_t i = 0; i < segments; ++i)
    {
        const uint32_t p1 = handle.start_particle_idx + i;
        const uint32_t p2 = handle.start_particle_idx + i + 1;
        AddDistanceConstraint(*this, p1, p2, compliance);
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
    handle.start_edge_idx = surface_edges_.size();
    handle.edge_count = (res_x - 1) * 2 + (res_y - 1) * 2;
    handle.start_surface_particle_idx = surface_particles_idx_.size();
    handle.surface_particle_count = res_x * 2 + res_y * 2 - 4;

    const float particle_mass = mass / static_cast<float>(handle.particle_count);
    const float inv_mass = particle_mass > 0.0f ? 1.0f / particle_mass : 0.0f;

    const Vec2 half_extents{width * 0.5f, height * 0.5f};
    const Vec2 min_pos = pos - half_extents;
    const Vec2 step{width / static_cast<float>(res_x - 1), height / static_cast<float>(res_y - 1)};

    // Particles
    for (uint16_t y = 0; y < res_y; ++y) 
    {
        for (uint16_t x = 0; x < res_x; ++x) 
        {
            Vec2 p_pos = min_pos + Vec2{step.x * static_cast<float>(x), step.y * static_cast<float>(y)};
            AddParticle(*this, p_pos, inv_mass, friction);
        }
    }

    auto get_idx = [&](const uint16_t x, const uint16_t y) { return handle.start_particle_idx + y * res_x + x; };

    std::pmr::unordered_set<uint32_t> seen(&frame_arena_);
    seen.reserve(handle.surface_particle_count);
    
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
                AddDistanceConstraint(*this, p00, p10, compliance);
            }
            if (x == 0){
                AddDistanceConstraint(*this, p00, p01, compliance);
            } 
                
            AddDistanceConstraint(*this, p10, p11, compliance);
            AddDistanceConstraint(*this, p01, p11, compliance);

            // Triangle Area Constraints
            // Each Quad is split into two triangles: (p00, p11, p10) and (p00, p01, p11).
            // compliance Setting it to 0 ensures that the volume does not degrade.
            AddTriangleAreaConstraint(*this, p00, p11, p10, 0.0f); 
            AddTriangleAreaConstraint(*this,p00, p01, p11, 0.0f);
        }
    }
    
    // Surface Particles (Clockwise Order)
    // Top Row
    for (uint16_t x = 0; x < res_x; ++x) {
        surface_particles_idx_.push_back(get_idx(x, res_y - 1));
    }
    // Right Column
    for (int32_t y = res_y - 2; y >= 0; --y) {
        surface_particles_idx_.push_back(get_idx(res_x - 1, y));
    }
    // Bottom Row
    for (int32_t x = res_x - 2; x >= 0; --x) {
        surface_particles_idx_.push_back(get_idx(x, 0));
    }
    // Left Column: (0, 0) -> (0, res_y-1)
    for (uint16_t y = 1; y < res_y - 1; ++y) {
        surface_particles_idx_.push_back(get_idx(0, y));
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
    handle.start_edge_idx = surface_edges_.size();
    handle.edge_count = segments;
    handle.start_surface_particle_idx = surface_particles_idx_.size();
    handle.surface_particle_count = segments;

    const float particle_mass = mass / static_cast<float>(handle.particle_count);
    const float inv_mass = particle_mass > 0.0f ? 1.0f / particle_mass : 0.0f;

    // Center Particle
    const uint32_t center_idx = AddParticle(*this, pos, inv_mass, friction);

    // Clockwise Outer Ring Particles
    static constexpr float two_pi = 6.28318530718f;
    for (uint16_t i = 0; i < segments; ++i)
    {
        const float angle = -(static_cast<float>(i) * two_pi) / static_cast<float>(segments);
        Vec2 p_pos = pos + Vec2{std::cos(angle) * radius, std::sin(angle) * radius};
        AddParticle(*this, p_pos, inv_mass, friction);
    }

    // Constraints
    for (uint16_t i = 0; i < segments; ++i) 
    {
        const uint32_t curr = handle.start_particle_idx + 1 + i;
        const uint32_t next = handle.start_particle_idx + 1 + ((i + 1) % segments);
        surface_particles_idx_.push_back(curr);

        // Outer Ring Distance  Constraint
        AddDistanceConstraint(*this, curr, next, compliance);
        surface_edges_.emplace_back(curr, next);
        
        // Internal Circular Sector Area Constraint (Clockwise)
        AddTriangleAreaConstraint(*this, center_idx, curr, next, 0.0f);
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
    handle.start_edge_idx = surface_edges_.size();
    handle.edge_count = segments;
    handle.start_surface_particle_idx = surface_particles_idx_.size();
    handle.surface_particle_count = segments;
    
    const float particle_mass = mass / static_cast<float>(handle.particle_count);
    const float inv_mass = particle_mass > 0.0f ? 1.0f / particle_mass : 0.0f;
    
    // Clockwise Outer Ring Particles
    static constexpr float two_pi = 6.28318530718f;
    for (uint16_t i = 0; i < segments; ++i) 
    {
        const float angle = -(static_cast<float>(i) * two_pi) / static_cast<float>(segments);
        Vec2 p_pos = pos + Vec2{std::cos(angle) * radius, std::sin(angle) * radius};
        AddParticle(*this, p_pos, inv_mass, friction);
    }
    
    // Constraints
    for (uint16_t i = 0; i < segments; ++i) 
    {
        const uint32_t curr = handle.start_particle_idx + i;
        const uint32_t next = handle.start_particle_idx + ((i + 1) % segments);
        surface_edges_.emplace_back(curr, next);
        surface_particles_idx_.push_back(curr);

        // Outer Ring Distance Constraint
        AddDistanceConstraint(*this, curr, next, compliance);
    }
    
    // Internal Global Area Constraint
    AddGlobalAreaConstraint(*this, handle.start_particle_idx, handle.particle_count, compliance);
    
    soft_body_handles_.push_back(handle);
    return handle;
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
        forces_[i] += Vec2{0.0f, gravity_} / rigid_bodies_[i].inv_mass;
    }
    
    for (int i = 0; i < particle_forces_.size(); ++i)
    {
        particle_forces_[i] += Vec2{0.0f, gravity_} / particle_inv_masses_[i];
    }
}

void PhysicsWorld::Step(const float dt)
{
    using Clock = std::chrono::high_resolution_clock;
    using Duration = std::chrono::duration<double, std::milli>;
    profile_data_.Reset();
    
    constexpr int sub_steps = 4;
    const float sub_dt = dt / static_cast<float>(sub_steps);
    const float inv_sdt_sq = 1 / (sub_dt * sub_dt);
    constexpr int velocity_iterations = 2;
    constexpr int position_iterations = 2;
    constexpr int xpbd_iterations = 2;
    
    for (int sub_step = 0; sub_step < sub_steps; ++sub_step)
    {
        frame_arena_.release();
        particle_rigid_contacts_.clear();
        particle_edge_contacts_.clear();
        
        ApplyGravity();
        
        auto start = Clock::now();
        MovementSystem::IntegrateVelocity(*this, sub_dt);
        profile_data_.integrate_rigid_vel += Duration(Clock::now() - start).count();
        
        start = Clock::now();
        XpbdSolver::PredictParticles(*this, sub_dt);
        profile_data_.predict_particle_pos += Duration(Clock::now() - start).count();
        
        CollisionDetectionSystem::CheckCollision(*this);

        start = Clock::now();
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
            XpbdSolver::SolveCollisionConstraints(*this, sub_dt);
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

std::array<Vec2, 4> PhysicsWorld::GetBodyVertices(const PhysicsWorld& world, const BoxShape& box_shape, uint32_t i)
{
    // Vertices are arranged in clockwise.
    std::array<Vec2, 4> vertices = {
        Vec2{-box_shape.half_width, box_shape.half_height}, Vec2{box_shape.half_width, box_shape.half_height},
        Vec2{box_shape.half_width, -box_shape.half_height}, Vec2{-box_shape.half_width, -box_shape.half_height}
    };
    
    std::ranges::transform(vertices, vertices.begin(), [&](const Vec2 v){ return world.rots_[i].Mul(v) + world.positions_[i].val; });
    
    return vertices;
}

std::vector<Vec2> PhysicsWorld::GetBodyVertices(const PhysicsWorld& world, const PolygonShape& polygon_shape, uint32_t i)
{
    return std::vector<Vec2>{};
}

void PhysicsWorld::ResetWorld()
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
    surface_particles_idx_.clear();
    distance_constraints_.clear();
    triangle_area_constraints_.clear();
    global_area_constraints_.clear();
    bvh_tree_root_index_ = std::numeric_limits<uint32_t>::max();
    bvh_tree_nodes_.clear();
    manifolds_.clear();
    manifold_map_.clear();
}
