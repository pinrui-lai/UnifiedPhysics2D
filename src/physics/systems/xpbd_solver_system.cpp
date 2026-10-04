#include "xpbd_solver_system.h"

#include <algorithm>

#include "physics_world.h"

void XpbdSolver::PredictParticles(PhysicsWorld& world, const float sdt)
{
    const uint32_t count = world.particle_positions_.size();

    for (uint32_t i = 0; i < count; ++i)
    {
        if (world.particle_inv_masses_[i] == 0.0f) continue;

        world.particle_prev_positions_[i] = world.particle_positions_[i];
        
        Vec2 accel = world.particle_forces_[i] * world.particle_inv_masses_[i];
        world.particle_velocities_[i] += accel * sdt;
        
        world.particle_positions_[i] += world.particle_velocities_[i] * sdt;
            
        world.particle_forces_[i] = Vec2{0.0f, 0.0f};
    }
}

void XpbdSolver::SolveDistanceConstraint(PhysicsWorld& world, const float inv_sdt_sq, XpbdDistanceConstraint& c)
{
    const uint32_t i1 = c.p1;
    const uint32_t i2 = c.p2;
    
    const float w1 = world.particle_inv_masses_[i1];
    const float w2 = world.particle_inv_masses_[i2];
    const float w_sum = w1 + w2;
    
    if (w_sum == 0.0f) return;

    const Vec2 pos1 = world.particle_positions_[i1];
    const Vec2 pos2 = world.particle_positions_[i2];

    const Vec2 delta = pos1 - pos2;
    const float len = delta.Length();
    if (len < 1e-6f) return;

    const Vec2 n = delta / len;
    const float C = len - c.rest_length; // Constraint function C(x)
    
    // alpha_tilde = compliance / (sdt^2)
    const float alpha_tilde = c.compliance * inv_sdt_sq;
    
    // dLambda
    const float delta_lambda = (-C - alpha_tilde * c.lambda) / (w_sum + alpha_tilde);
    c.lambda += delta_lambda;
    
    // Position correction : dx = w * n * dLambda
    const Vec2 dx = n * delta_lambda;
    if (w1 > 0.0f) world.particle_positions_[i1] += dx * w1;
    if (w2 > 0.0f) world.particle_positions_[i2] -= dx * w2;
}

void XpbdSolver::SolveTriangleAreaConstraint(PhysicsWorld& world, float inv_sdt_sq, XpbdTriangleAreaConstraint& c)
{
    const uint32_t i1 = c.p1, i2 = c.p2, i3 = c.p3; // Clockwise
    
    const float w1 = world.particle_inv_masses_[i1];
    const float w2 = world.particle_inv_masses_[i2];
    const float w3 = world.particle_inv_masses_[i3];
    
    if (w1 + w2 + w3 == 0.0f) return;
    
    const Vec2 p1 = world.particle_positions_[i1];
    const Vec2 p2 = world.particle_positions_[i2];
    const Vec2 p3 = world.particle_positions_[i3];
    
    // Triangle Area
    const float current_area = 0.5f * (p1.x * (p3.y - p2.y) + p2.x * (p1.y - p3.y) + p3.x * (p2.y - p1.y));
    float C = current_area - c.rest_area;
    
    // Gradient of C
    const Vec2 grad1{ 0.5f * (p3.y - p2.y), 0.5f * (p2.x - p3.x) };
    const Vec2 grad2{ 0.5f * (p1.y - p3.y), 0.5f * (p3.x - p1.x) };
    const Vec2 grad3{ 0.5f * (p2.y - p1.y), 0.5f * (p1.x - p2.x) };

    const float grad_sq_sum = w1 * grad1.LengthSquared() + w2 * grad2.LengthSquared() + w3 * grad3.LengthSquared();
    
    if (grad_sq_sum < 1e-9f) return;
    
    // XPBD Position Correction
    const float alpha_tilde = c.compliance * inv_sdt_sq;
    const float delta_lambda = (-C - alpha_tilde * c.lambda) / (grad_sq_sum + alpha_tilde);
    c.lambda += delta_lambda;
    
    if (w1 > 0.0f) world.particle_positions_[i1] += grad1 * (w1 * delta_lambda);
    if (w2 > 0.0f) world.particle_positions_[i2] += grad2 * (w2 * delta_lambda);
    if (w3 > 0.0f) world.particle_positions_[i3] += grad3 * (w3 * delta_lambda);
}

void XpbdSolver::SolveGlobalAreaConstraints(PhysicsWorld& world, float inv_sdt_sq)
{
    // Particles arranged Clockwise.
    for (auto& c : world.global_area_constraints_)
    {
        if (c.particle_count < 3) return;

        float current_area = 0.0f;
        for (uint16_t i = 0; i < c.particle_count; ++i)
        {
            const uint32_t curr = c.start_particle_idx + i;
            const uint32_t next = c.start_particle_idx + ((i + 1) % c.particle_count);
            const Vec2& p1 = world.particle_positions_[curr];
            const Vec2& p2 = world.particle_positions_[next];
        
            current_area += (p2.x * p1.y - p1.x * p2.y);
        }
        current_area *= 0.5f;

        float C = current_area - c.rest_area; // C(x) = A - A0

        // Gradients
        std::pmr::vector<Vec2> grads(&world.frame_arena_);
        grads.resize(c.particle_count);
        float grad_sq_sum = 0.0f;

        for (uint16_t i = 0; i < c.particle_count; ++i)
        {
            const uint32_t prev_idx = c.start_particle_idx + ((i + c.particle_count - 1) % c.particle_count);
            const uint32_t next_idx = c.start_particle_idx + ((i + 1) % c.particle_count);

            const Vec2& p_prev = world.particle_positions_[prev_idx];
            const Vec2& p_next = world.particle_positions_[next_idx];

            // Grad_i = 0.5 * (y_{i-1} - y_{i+1}, x_{i+1} - x_{i-1})
            grads[i] = Vec2{ 0.5f * (p_prev.y - p_next.y), 0.5f * (p_next.x - p_prev.x) };

            const uint32_t curr_idx = c.start_particle_idx + i;
            const float w = world.particle_inv_masses_[curr_idx];
            grad_sq_sum += w * grads[i].LengthSquared();
        }

        if (grad_sq_sum < 1e-9f) return;

        const float alpha_tilde = c.compliance * inv_sdt_sq;
        const float delta_lambda = (-C - alpha_tilde * c.lambda) / (grad_sq_sum + alpha_tilde);
        c.lambda += delta_lambda;
        
        for (uint16_t i = 0; i < c.particle_count; ++i)
        {
            const uint32_t curr_idx = c.start_particle_idx + i;
            const float w = world.particle_inv_masses_[curr_idx];
            if (w > 0.0f)
            {
                world.particle_positions_[curr_idx] += grads[i] * (w * delta_lambda);
            }
        }
    }
}

void XpbdSolver::SolveCollisionConstraints(PhysicsWorld& world, const float sub_dt)
{
    for (auto& c : world.particle_rigid_contacts_)
    {
        const float w_particle = world.particle_inv_masses_[c.particle_idx];
        
        const auto& rigid_body = world.rigid_bodies_[c.body_idx];
        Vec2 rb = c.contact_point - world.positions_[c.body_idx].val;
        const float rb_cross_n = rb.Cross(c.contact_normal);
        const float w_rigid = rigid_body.inv_mass + rigid_body.inv_inertia * (rb_cross_n * rb_cross_n);
        const float w_sum = w_particle + w_rigid;
        
        if (w_sum <= 1e-6f) continue;
            
        // Calculate current penetration depth.
        float C = (world.particle_positions_[c.particle_idx] - c.contact_point).Dot(c.contact_normal);
        if (C >= 0.0f) continue;

        const float delta_lambda = -C / w_sum;

        // Move particle
        world.particle_positions_[c.particle_idx] += c.contact_normal * (delta_lambda * w_particle);
        
        // Update rigid velocity (linear + angular)
        Vec2 impulse_n = c.contact_normal * (delta_lambda / sub_dt);
        world.velocities_[c.body_idx].val -= impulse_n * rigid_body.inv_mass;
        world.angular_velocities_[c.body_idx].val -= rigid_body.inv_inertia * rb.Cross(impulse_n);
        
        // ===== Tangent Friction =====
        
        // Compute relative velocity
        Vec2 particle_vel = (world.particle_positions_[c.particle_idx] - world.particle_prev_positions_[c.particle_idx]) / sub_dt;
        Vec2 rigid_contact_point_vel = world.velocities_[c.body_idx].val + Vec2::ZCross(world.angular_velocities_[c.body_idx].val, rb);
        Vec2 rel_vel = particle_vel - rigid_contact_point_vel;
        Vec2 rel_vel_tangent = rel_vel - c.contact_normal * rel_vel.Dot(c.contact_normal);
        const float rel_vel_tangent_len = rel_vel_tangent.Length();
        
        if (rel_vel_tangent_len <= 1e-6f) continue;
        
        Vec2 tangent_dir = rel_vel_tangent / rel_vel_tangent_len;

        const float rb_cross_t = rb.Cross(tangent_dir);
        const float w_rigid_t =  rigid_body.inv_mass + rigid_body.inv_inertia * (rb_cross_t * rb_cross_t);
        const float w_sum_t = w_particle + w_rigid_t;
        
        float j_t_desired = rel_vel_tangent_len / w_sum_t; // Magnitude of impulse on tangent direction
        const float j_n_applied = delta_lambda / sub_dt; // Magnitude of impulse on normal direction
        
        float max_friction = rigid_body.friction * j_n_applied;
        const float j_t_final = std::min(j_t_desired, max_friction);
        
        // Move particle (convert magnitude of impulse back to position correction)
        world.particle_positions_[c.particle_idx] -= tangent_dir * (j_t_final * sub_dt) * w_particle;
        
        // Update rigid velocity
        Vec2 impulse_t = tangent_dir * j_t_final;
        world.velocities_[c.body_idx].val += impulse_t * rigid_body.inv_mass;
        world.angular_velocities_[c.body_idx].val += rigid_body.inv_inertia * rb.Cross(impulse_t);
    }
}

void XpbdSolver::SolveParticleEdgeCollisionConstraints(PhysicsWorld& world)
{
    for (const auto& c : world.particle_edge_contacts_) {
        Vec2& p_pos = world.particle_positions_[c.body_a_particle_idx];
        Vec2& a_pos = world.particle_positions_[c.body_b_p1_idx];
        Vec2& b_pos = world.particle_positions_[c.body_b_p2_idx];
        
        // Compute current penetration.
        Vec2 ab = b_pos - a_pos;
        Vec2 proj = a_pos + ab * c.t; 
        Vec2 diff = p_pos - proj;
        
        float C = diff.Dot(c.normal) - physics_config::kTwoParticlesThickness;
        if (C >= 0.0f) continue;
        
        const float w_p = world.particle_inv_masses_[c.body_a_particle_idx];
        const float w_a = world.particle_inv_masses_[c.body_b_p1_idx];
        const float w_b = world.particle_inv_masses_[c.body_b_p2_idx];

        // grad_p = n, grad_a = -(1-t)n, grad_b = -t*n
        const float w_sum = w_p + (1.f - c.t) * (1.f - c.t) * w_a + c.t * c.t * w_b;
        
        if (w_sum < 1e-6f) continue;

        const float delta_lambda = -C / w_sum ;

        // Move particles
        p_pos += c.normal * (w_p * delta_lambda);
        a_pos -= c.normal * ((1.f - c.t) * w_a * delta_lambda);
        b_pos -= c.normal * (c.t * w_b * delta_lambda);
        
        // ===== tangent friction =====
        
        // Compute relative delta x.
        Vec2 particle_dx = p_pos - world.particle_prev_positions_[c.body_a_particle_idx];
        Vec2 edge_p1_dx = a_pos - world.particle_prev_positions_[c.body_b_p1_idx];
        Vec2 edge_p2_dx = b_pos - world.particle_prev_positions_[c.body_b_p2_idx];
        Vec2 proj_dx = edge_p1_dx * (1 - c.t) + edge_p2_dx * c.t;
        Vec2 rel_dx = particle_dx - proj_dx;
        Vec2 rel_dx_tangent = rel_dx - c.normal * rel_dx.Dot(c.normal);
        const float rel_dx_tangent_len = rel_dx_tangent.Length();
        
        if (rel_dx_tangent_len <= 1e-6f) continue;
        
        Vec2 tangent_dir = rel_dx_tangent / rel_dx_tangent_len;
        
        const float mu = std::sqrt(world.particle_frictions_[c.body_a_particle_idx] * world.particle_frictions_[c.body_b_p1_idx]);
        const float normal_dx = -C;
        const float final_friction = std::min(rel_dx_tangent_len, normal_dx * mu);

        const float delta_lambda_t = final_friction / w_sum;
        
        p_pos -= tangent_dir * delta_lambda_t * w_p;
        a_pos += tangent_dir * delta_lambda_t * (1.f - c.t) * w_a;
        b_pos += tangent_dir * delta_lambda_t * c.t * w_b;
    }
}

void XpbdSolver::FinalizeParticles(PhysicsWorld& world, const float inv_sdt)
{
    const uint32_t count = world.particle_positions_.size();
    for (uint32_t i = 0; i < count; ++i)
    {
        if (world.particle_inv_masses_[i] == 0.0f) continue;

        // Update the velocity based on the position correction.
        world.particle_velocities_[i] = (world.particle_positions_[i] - world.particle_prev_positions_[i]) * inv_sdt;
    }
}
