#include "xpbd_solver_system.h"

#include "physics_world.h"

void XpbdSolver::PredictParticles(PhysicsWorld& world, const float sdt)
{
    const size_t count = world.particle_positions_.size();

    for (size_t i = 0; i < count; ++i)
    {
        if (world.particle_inv_masses_[i] == 0.0f) continue;

        world.particle_prev_positions_[i] = world.particle_positions_[i];
        
        Vec2 accel = world.particle_forces_[i] * world.particle_inv_masses_[i];
        world.particle_velocities_[i] += accel * sdt;
        
        world.particle_positions_[i] += world.particle_velocities_[i] * sdt;
            
        world.particle_forces_[i] = Vec2{0.0f, 0.0f};
    }
}

void XpbdSolver::SolveDistanceConstraints(PhysicsWorld& world, const float inv_sdt_sq)
{
    for (auto& c : world.distance_constraints_)
    {
        const size_t i1 = c.p1;
        const size_t i2 = c.p2;

        const float w1 = world.particle_inv_masses_[i1];
        const float w2 = world.particle_inv_masses_[i2];
        const float w_sum = w1 + w2;

        if (w_sum == 0.0f) continue;

        Vec2 pos1 = world.particle_positions_[i1];
        Vec2 pos2 = world.particle_positions_[i2];

        Vec2 delta = pos1 - pos2;
        const float len = delta.Length();
        if (len < 1e-6f) continue;

        Vec2 n = delta / len;
        const float C = len - c.rest_length; // Constraint function C(x)

        // alpha_tilde = compliance / (sdt^2)
        const float alpha_tilde = c.compliance * inv_sdt_sq;

        // dLambda
        const float delta_lambda = (-C - alpha_tilde * c.lambda) / (w_sum + alpha_tilde);
        c.lambda += delta_lambda;

        // Position correction : dx = w * n * dLambda
        Vec2 dx = n * delta_lambda;
        if (w1 > 0.0f) world.particle_positions_[i1] += dx * w1;
        if (w2 > 0.0f) world.particle_positions_[i2] -= dx * w2;
    }
}

void XpbdSolver::SolveCollisionConstraints(PhysicsWorld& world, const float inv_sdt_sq)
{
    for (auto& c : world.soft_rigid_contacts_)
    {
        const size_t p_idx = c.particle_idx;
        const float w1 = world.particle_inv_masses_[p_idx];
        if (w1 == 0.0f) continue;

        Vec2 p = world.particle_positions_[p_idx];
            
        // Calculate current penetration depth.
        float C = (p - c.contact_point).Dot(c.contact_normal);
        if (C >= 0.0f) continue;

        const float alpha_tilde = c.compliance * inv_sdt_sq;
        const float delta_lambda = (-C - alpha_tilde * c.lambda) / (w1 + alpha_tilde);
        c.lambda += delta_lambda;

        // Move particle position to body surface
        world.particle_positions_[p_idx] += c.contact_normal * (delta_lambda * w1);

        //  [Two-way Coupling] If software needs to push a rigid body, apply an equal and opposite Impulse to the rigid body here.
    }
}

void XpbdSolver::SolveTriangleAreaConstraints(PhysicsWorld& world, const float inv_sdt_sq)
{
    for (auto& c : world.triangle_area_constraints_)
    {
        const size_t i1 = c.p1, i2 = c.p2, i3 = c.p3; // Clockwise
    
        const float w1 = world.particle_inv_masses_[i1];
        const float w2 = world.particle_inv_masses_[i2];
        const float w3 = world.particle_inv_masses_[i3];
    
        if (w1 + w2 + w3 == 0.0f) continue;
    
        const Vec2 p1 = world.particle_positions_[i1];
        const Vec2 p2 = world.particle_positions_[i2];
        const Vec2 p3 = world.particle_positions_[i3];
    
        // Triangle Area
        const float current_area = 0.5f * (p1.x * (p3.y - p2.y) + p2.x * (p1.y - p3.y) + p3.x * (p2.y - p1.y));
        float C = current_area - c.rest_area;
    
        // Gradient of C
        Vec2 grad1{ 0.5f * (p3.y - p2.y), 0.5f * (p2.x - p3.x) };
        Vec2 grad2{ 0.5f * (p1.y - p3.y), 0.5f * (p3.x - p1.x) };
        Vec2 grad3{ 0.5f * (p2.y - p1.y), 0.5f * (p1.x - p2.x) };

        const float grad_sq_sum = w1 * grad1.LengthSquared() + w2 * grad2.LengthSquared() + w3 * grad3.LengthSquared();
    
        if (grad_sq_sum < 1e-9f) continue;
    
        // XPBD Position Correction
        const float alpha_tilde = c.compliance * inv_sdt_sq;
        const float delta_lambda = (-C - alpha_tilde * c.lambda) / (grad_sq_sum + alpha_tilde);
        c.lambda += delta_lambda;
    
        if (w1 > 0.0f) world.particle_positions_[i1] += grad1 * (w1 * delta_lambda);
        if (w2 > 0.0f) world.particle_positions_[i2] += grad2 * (w2 * delta_lambda);
        if (w3 > 0.0f) world.particle_positions_[i3] += grad3 * (w3 * delta_lambda);
    }
}

void XpbdSolver::SolveGlobalAreaConstraints(PhysicsWorld& world, float inv_sdt_sq)
{
    // Particles arranged Clockwise.
    for (auto& c : world.global_area_constraints_)
    {
        if (c.particle_count < 3) return;

        float current_area = 0.0f;
        for (size_t i = 0; i < c.particle_count; ++i)
        {
            const size_t curr = c.start_particle_idx + i;
            const size_t next = c.start_particle_idx + ((i + 1) % c.particle_count);
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

        for (size_t i = 0; i < c.particle_count; ++i)
        {
            const size_t prev_idx = c.start_particle_idx + ((i + c.particle_count - 1) % c.particle_count);
            const size_t next_idx = c.start_particle_idx + ((i + 1) % c.particle_count);

            const Vec2& p_prev = world.particle_positions_[prev_idx];
            const Vec2& p_next = world.particle_positions_[next_idx];

            // Grad_i = 0.5 * (y_{i-1} - y_{i+1}, x_{i+1} - x_{i-1})
            grads[i] = Vec2{ 0.5f * (p_prev.y - p_next.y), 0.5f * (p_next.x - p_prev.x) };

            const size_t curr_idx = c.start_particle_idx + i;
            const float w = world.particle_inv_masses_[curr_idx];
            grad_sq_sum += w * grads[i].LengthSquared();
        }

        if (grad_sq_sum < 1e-9f) return;

        const float alpha_tilde = c.compliance * inv_sdt_sq;
        const float delta_lambda = (-C - alpha_tilde * c.lambda) / (grad_sq_sum + alpha_tilde);
        c.lambda += delta_lambda;
        
        for (size_t i = 0; i < c.particle_count; ++i)
        {
            const size_t curr_idx = c.start_particle_idx + i;
            const float w = world.particle_inv_masses_[curr_idx];
            if (w > 0.0f)
            {
                world.particle_positions_[curr_idx] += grads[i] * (w * delta_lambda);
            }
        }
    }
}

void XpbdSolver::FinalizeParticles(PhysicsWorld& world, float inv_sdt)
{
    const size_t count = world.particle_positions_.size();
    for (size_t i = 0; i < count; ++i)
    {
        if (world.particle_inv_masses_[i] == 0.0f) continue;

        // Update the velocity based on the position correction.
        world.particle_velocities_[i] = (world.particle_positions_[i] - world.particle_prev_positions_[i]) * inv_sdt;
    }
}
