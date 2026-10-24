#pragma once
#include "physics/types.h"

class PhysicsWorld;

class XpbdSolver
{
private:
    static void PredictParticles(PhysicsWorld& world, float sdt);
    static void SolveDistanceConstraint(PhysicsWorld& world, float inv_sdt_sq, XpbdDistanceConstraint& c);
    static void SolveTriangleAreaConstraint(PhysicsWorld& world, float inv_sdt_sq, XpbdTriangleAreaConstraint& c);
    static void SolveGlobalAreaConstraints(PhysicsWorld& world, float inv_sdt_sq);
    static void SolveRigidParticleCollisionConstraints(PhysicsWorld& world, float sub_dt, float inv_sub_dt);
    
    template<std::ranges::input_range R>
    static void SolveParticleEdgeCollisionConstraints(PhysicsWorld& world, R&& batch)
    {
        for (const auto& i : batch)
        {
            const XpbdParticleEdgeCollisionConstraint& c = world.particle_edge_collision_constraint_soa_view_.GetConstraint(i);
            const Vec2 p_pos = world.GetParticlePosition(c.body_a_particle_idx);
            const Vec2 a_pos = world.GetParticlePosition(c.body_b_p1_idx);
            const Vec2 b_pos = world.GetParticlePosition(c.body_b_p2_idx);
            
            // Compute current penetration.
            Vec2 ab = b_pos - a_pos;
            Vec2 proj = a_pos + ab * c.t; 
            Vec2 diff = p_pos - proj;

            const float violation = diff.Dot(c.normal) - physics_config::kTwoParticlesThickness;
            if (violation >= 0.f) continue;
            
            const float w_p = world.particle_inv_masses_[c.body_a_particle_idx];
            const float w_a = world.particle_inv_masses_[c.body_b_p1_idx];
            const float w_b = world.particle_inv_masses_[c.body_b_p2_idx];

            // grad_p = n, grad_a = -(1-t)n, grad_b = -t*n
            const float w_sum = w_p + (1.f - c.t) * (1.f - c.t) * w_a + c.t * c.t * w_b;
            
            if (w_sum < 1e-6f) continue;

            const float delta_lambda = -violation / w_sum ;

            // Move particles
            world.PlusEqualParticlePosition(c.body_a_particle_idx, c.normal * (w_p * delta_lambda));
            world.PlusEqualParticlePosition(c.body_b_p1_idx, -(c.normal * ((1.f - c.t) * w_a * delta_lambda)));
            world.PlusEqualParticlePosition(c.body_b_p2_idx, -(c.normal * (c.t * w_b * delta_lambda)));
            
            // ===== tangent friction =====
            
            // Compute relative delta x.
            const Vec2 particle_dx = p_pos - world.GetParticlePrevPosition(c.body_a_particle_idx);
            const Vec2 edge_p1_dx = a_pos - world.GetParticlePrevPosition(c.body_b_p1_idx);
            const Vec2 edge_p2_dx = b_pos - world.GetParticlePrevPosition(c.body_b_p2_idx);
            Vec2 proj_dx = edge_p1_dx * (1 - c.t) + edge_p2_dx * c.t;
            Vec2 rel_dx = particle_dx - proj_dx;
            Vec2 rel_dx_tangent = rel_dx - c.normal * rel_dx.Dot(c.normal);
            const float rel_dx_tangent_len = rel_dx_tangent.Length();
            
            if (rel_dx_tangent_len <= 1e-6f) continue;
            
            Vec2 tangent_dir = rel_dx_tangent / rel_dx_tangent_len;
            
            const float mu = std::sqrt(world.particle_frictions_[c.body_a_particle_idx] * world.particle_frictions_[c.body_b_p1_idx]);
            const float normal_dx = -violation;
            const float final_friction = std::min(rel_dx_tangent_len, normal_dx * mu);

            const float delta_lambda_t = final_friction / w_sum;
            
            world.PlusEqualParticlePosition(c.body_a_particle_idx, -(tangent_dir * delta_lambda_t * w_p));
            world.PlusEqualParticlePosition(c.body_b_p1_idx, tangent_dir * delta_lambda_t * (1.f - c.t) * w_a);
            world.PlusEqualParticlePosition(c.body_b_p2_idx, tangent_dir * delta_lambda_t * c.t * w_b);
        }
    }
    
    static void SolveRigidParticleJointConstraints(PhysicsWorld& world, float inv_sub_dt);
    static void SolveRigidParticleAngularConstraints(PhysicsWorld& world, float inv_sub_dt);
    static void FinalizeParticles(PhysicsWorld& world, float inv_sdt);
    
    static void BuildBatches(PhysicsWorld& world, const ParticleEdgeCollisionConstraintSoAView& particle_edge_collision_constraint_soa_view,
        int body_count, std::pmr::vector<std::vector<int32_t>>& batches);
    
    static void SolveParticleEdgeBatchAvx2(PhysicsWorld& world, std::span<int32_t> batch);
    
    __attribute__((target("avx,avx2")))
    static __m256 Gather8(const std::vector<float>& base, __m256i idx)
    {
        return _mm256_i32gather_ps(base.data(), idx, 4);
    }
    
    friend PhysicsWorld;
};
