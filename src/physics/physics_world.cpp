#include <algorithm>

#include "physics_world.h"
#include "movement_system.h"
#include "collision_solver_system.h"
#include "collision_detection_system.h"
#include "xpbd_solver_system.h"

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

size_t PhysicsWorld::AddParticle(PhysicsWorld& world, const Vec2& pos, const float inv_mass)
{
    const size_t idx = world.particle_positions_.size();
    world.particle_positions_.push_back(pos);
    world.particle_prev_positions_.push_back(pos);
    world.particle_velocities_.push_back(Vec2{0.0f, 0.0f});
    world.particle_inv_masses_.push_back(inv_mass);
    world.particle_forces_.push_back(Vec2{0.0f, 0.0f});
    return idx;
}

size_t PhysicsWorld::AddDistanceConstraint(PhysicsWorld& world, const size_t p1, const size_t p2, const float compliance)
{
    const size_t idx = world.distance_constraints_.size();
    const float rest_len = (world.particle_positions_[p1] - world.particle_positions_[p2]).Length();
    world.distance_constraints_.emplace_back(p1, p2, rest_len, compliance, 0.0f);
    return idx;
}

size_t PhysicsWorld::AddTriangleAreaConstraint(PhysicsWorld& world, size_t p1, size_t p2, size_t p3, float compliance)
{
    const size_t idx = world.triangle_area_constraints_.size();
    const Vec2 pos_p1 = world.particle_positions_[p1];
    const Vec2 pos_p2 = world.particle_positions_[p2];
    const Vec2 pos_p3 = world.particle_positions_[p3];
    float area = 0.5f * (pos_p1.x * (pos_p3.y - pos_p2.y) + pos_p2.x * (pos_p1.y - pos_p3.y) + pos_p3.x * (pos_p2.y - pos_p1.y));
    world.triangle_area_constraints_.emplace_back(p1, p2, p3, area, compliance, 0.f);
    return idx;
}

size_t PhysicsWorld::AddGlobalAreaConstraint(PhysicsWorld& world, size_t start_particle_idx, size_t particle_count,
    float compliance)
{
    const size_t idx = world.global_area_constraints_.size();
    
    float rest_area = 0.0f;
    for (size_t i = 0; i < particle_count; ++i)
    {
        const size_t curr = start_particle_idx + i;
        const size_t next = start_particle_idx + ((i + 1) % particle_count);
        const Vec2& p1 = world.particle_positions_[curr];
        const Vec2& p2 = world.particle_positions_[next];
        
        rest_area += (p2.x * p1.y - p1.x * p2.y);
    }
    rest_area *= 0.5f;
    
    world.global_area_constraints_.emplace_back(start_particle_idx, particle_count, rest_area, compliance, 0.0f);
    return idx;
}

SoftBodyHandle PhysicsWorld::CreateRope(const Vec2& start, const Vec2& end, const size_t segments, const float mass,
                                        const float compliance, const bool fix_start, const bool fix_end)
{
    if (segments == 0) return {};

    SoftBodyHandle handle;
    handle.start_particle_idx = particle_positions_.size();
    handle.particle_count = segments + 1;

    const float particle_mass = mass / static_cast<float>(handle.particle_count);
    const float inv_particle_mass = particle_mass > 0.0f ? 1.0f / particle_mass : 0.0f;
    const Vec2 step = (end - start) / static_cast<float>(segments);

    // Create Particles
    for (size_t i = 0; i <= segments; ++i)
    {
        Vec2 pos = start + step * static_cast<float>(i);
        float current_inv_particle_mass = inv_particle_mass;
        if ((i == 0 && fix_start) || (i == segments && fix_end))
        {
            current_inv_particle_mass = 0.0f; // Mass of fixed particle is infinity
        }
        AddParticle(*this, pos, current_inv_particle_mass);
    }

    // Create distance constraints
    for (size_t i = 0; i < segments; ++i)
    {
        const size_t p1 = handle.start_particle_idx + i;
        const size_t p2 = handle.start_particle_idx + i + 1;
        AddDistanceConstraint(*this, p1, p2, compliance);
    }

    soft_body_handles_.push_back(handle);
    return handle;
}

SoftBodyHandle PhysicsWorld::CreateSoftBox(const Vec2& pos, float width, float height, size_t res_x, size_t res_y,
    const float mass, const float compliance)
{
    if (res_x < 2 || res_y < 2) return {};

    SoftBodyHandle handle;
    handle.start_particle_idx = particle_positions_.size();
    handle.particle_count = res_x * res_y;

    const float particle_mass = mass / static_cast<float>(handle.particle_count);
    const float inv_mass = particle_mass > 0.0f ? 1.0f / particle_mass : 0.0f;

    const Vec2 half_extents{width * 0.5f, height * 0.5f};
    const Vec2 min_pos = pos - half_extents;
    const Vec2 step{width / static_cast<float>(res_x - 1), height / static_cast<float>(res_y - 1)};

    // Particles
    for (size_t y = 0; y < res_y; ++y) 
    {
        for (size_t x = 0; x < res_x; ++x) 
        {
            Vec2 p_pos = min_pos + Vec2{step.x * static_cast<float>(x), step.y * static_cast<float>(y)};
            AddParticle(*this, p_pos, inv_mass);
        }
    }

    auto get_idx = [&](const size_t x, const size_t y) { return handle.start_particle_idx + y * res_x + x; };

    // Constraints
    for (size_t y = 0; y < res_y - 1; ++y) 
    {
        for (size_t x = 0; x < res_x - 1; ++x) 
            {
            const size_t p00 = get_idx(x, y);
            const size_t p10 = get_idx(x + 1, y);
            const size_t p01 = get_idx(x, y + 1);
            const size_t p11 = get_idx(x + 1, y + 1);

            // Structural Springs
            if (y == 0) AddDistanceConstraint(*this, p00, p10, compliance);
            if (x == 0) AddDistanceConstraint(*this, p00, p01, compliance);
            AddDistanceConstraint(*this, p10, p11, compliance);
            AddDistanceConstraint(*this, p01, p11, compliance);

            // Triangle Area Constraints
            // Each Quad is split into two triangles: (p00, p11, p10) and (p00, p01, p11).
            // compliance Setting it to 0 ensures that the volume does not degrade.
            AddTriangleAreaConstraint(*this, p00, p11, p10, 0.0f); 
            AddTriangleAreaConstraint(*this,p00, p01, p11, 0.0f);
            
        }
    }
    
    soft_body_handles_.push_back(handle);
    return handle;
}

SoftBodyHandle PhysicsWorld::CreateSoftCircle(const Vec2& pos, const float radius, const size_t segments, const float mass,
    const float compliance)
{
    if (segments < 3) return {};

    SoftBodyHandle handle;
    handle.start_particle_idx = particle_positions_.size();
    handle.particle_count = segments + 1; // 1 Center + N Outer Ring Particles

    const float particle_mass = mass / static_cast<float>(handle.particle_count);
    const float inv_mass = particle_mass > 0.0f ? 1.0f / particle_mass : 0.0f;

    // Center Particle
    const size_t center_idx = AddParticle(*this, pos, inv_mass);

    // Clockwise Outer Ring Particles
    static constexpr float two_pi = 6.28318530718f;
    for (size_t i = 0; i < segments; ++i)
    {
        const float angle = -(static_cast<float>(i) * two_pi) / static_cast<float>(segments);
        Vec2 p_pos = pos + Vec2{std::cos(angle) * radius, std::sin(angle) * radius};
        AddParticle(*this, p_pos, inv_mass);
    }

    // Constraints
    for (size_t i = 0; i < segments; ++i) 
        {
        const size_t curr = handle.start_particle_idx + 1 + i;
        const size_t next = handle.start_particle_idx + 1 + ((i + 1) % segments);

        // Outer Ring Distance  Constraint
        AddDistanceConstraint(*this, curr, next, compliance);

        // Internal Circular Sector Area Constraint (Clockwise)
        AddTriangleAreaConstraint(*this, center_idx, curr, next, 0.0f);
    }

    soft_body_handles_.push_back(handle);
    return handle;
}

SoftBodyHandle PhysicsWorld::CreatePressureSoftCircle(const Vec2& pos, const float radius, const size_t segments, const float mass,
    const float compliance)
{
    if (segments < 3) return {};

    SoftBodyHandle handle;
    handle.start_particle_idx = particle_positions_.size();
    handle.particle_count = segments; // Outer Ring Particles

    const float particle_mass = mass / static_cast<float>(handle.particle_count);
    const float inv_mass = particle_mass > 0.0f ? 1.0f / particle_mass : 0.0f;
    
    // Clockwise Outer Ring Particles
    static constexpr float two_pi = 6.28318530718f;
    for (size_t i = 0; i < segments; ++i) 
    {
        const float angle = -(static_cast<float>(i) * two_pi) / static_cast<float>(segments);
        Vec2 p_pos = pos + Vec2{std::cos(angle) * radius, std::sin(angle) * radius};
        AddParticle(*this, p_pos, inv_mass);
    }

    // Constraints
    for (size_t i = 0; i < segments; ++i) 
    {
        const size_t curr = handle.start_particle_idx + i;
        const size_t next = handle.start_particle_idx + ((i + 1) % segments);

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
    rigid_bodies_[handle.id].force += force;
}

void PhysicsWorld::ApplyForceToAll(const Vec2& force)
{
    for (int i = 0; i < rigid_bodies_.size(); ++i)
    {
        rigid_bodies_[i].force += force;
    }
    
    for (auto& particle_force : particle_forces_)
    {
        particle_force += force;
    }
}

void PhysicsWorld::ApplyGravity()
{
    for (auto& rigid_body : rigid_bodies_)
    {
        rigid_body.force += Vec2{0.0f, gravity_} * rigid_body.mass;
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
    
    constexpr int sub_steps = 3;
    const float sub_dt = dt / static_cast<float>(sub_steps);
    const float inv_sdt_sq = 1 / (sub_dt * sub_dt);
    constexpr int velocity_iterations = 3;
    constexpr int position_iterations = 3;
    constexpr int xpbd_iterations = 3;
    
    for (int sub = 0; sub < sub_steps; ++sub)
    {
        
        frame_arena_.release();
        soft_rigid_contacts_.clear();
        ApplyGravity();
        
        MovementSystem::IntegrateVelocity(*this, sub_dt);
        XpbdSolver::PredictParticles(*this, sub_dt);
        
        CollisionDetectionSystem::CheckCollision(*this);

        auto start = Clock::now();
        for (int i = 0; i < xpbd_iterations; ++i)
        {
            XpbdSolver::SolveDistanceConstraints(*this, inv_sdt_sq);
            XpbdSolver::SolveTriangleAreaConstraints(*this, inv_sdt_sq);
            XpbdSolver::SolveGlobalAreaConstraints(*this, inv_sdt_sq);
            XpbdSolver::SolveCollisionConstraints(*this, inv_sdt_sq);
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

        MovementSystem::IntegratePosition(*this, sub_dt);
        XpbdSolver::FinalizeParticles(*this, 1.f/sub_dt);
    }
}

std::span<Vec2> PhysicsWorld::GetSoftBodyParticlePositions(const size_t i)
{
    const SoftBodyHandle& soft_body_handle = soft_body_handles_[i];
    return std::span(particle_positions_).subspan(soft_body_handle.start_particle_idx, soft_body_handle.particle_count);
}
