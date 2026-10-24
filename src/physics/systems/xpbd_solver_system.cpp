#include <algorithm>
#include <execution>
#include <ranges>

#include "physics/physics_world.h"
#include "physics/systems/xpbd_solver_system.h"

void XpbdSolver::PredictParticles(PhysicsWorld& world, const float sdt)
{
    const uint32_t count = world.particle_inv_masses_.size();

    auto index_range = std::views::iota(uint32_t{0}, count);
    std::for_each(std::execution::par_unseq, index_range.begin(), index_range.end(),[&](auto i)
    {
        if (world.particle_inv_masses_[i] == 0.f) return;
    
        world.SetParticlePrevPosition(i, world.GetParticlePosition(i));
    
        const Vec2 accel = world.particle_forces_[i] * world.particle_inv_masses_[i];
        world.particle_velocities_[i] += accel * sdt;
        
        world.PlusEqualParticlePosition(i, world.particle_velocities_[i] * sdt);    
        
        world.particle_forces_[i] = Vec2{0.f, 0.f};
    });
}

void XpbdSolver::SolveDistanceConstraint(PhysicsWorld& world, const float inv_sdt_sq, XpbdDistanceConstraint& c)
{
    const uint32_t i1 = c.p1;
    const uint32_t i2 = c.p2;
    
    const float w1 = world.particle_inv_masses_[i1];
    const float w2 = world.particle_inv_masses_[i2];
    const float w_sum = w1 + w2;
    
    if (w_sum == 0.f) return;

    const Vec2 pos1 = world.GetParticlePosition(i1);
    const Vec2 pos2 = world.GetParticlePosition(i2);

    const Vec2 delta = pos1 - pos2;
    const float len = delta.Length();
    if (len < 1e-6f) return;

    const Vec2 n = delta / len;
    const float violation = len - c.rest_length; // Constraint function C(x)
    
    // alpha_tilde = compliance / (sdt^2)
    const float alpha_tilde = c.compliance * inv_sdt_sq;
    
    // dLambda
    const float delta_lambda = (-violation - alpha_tilde * c.lambda) / (w_sum + alpha_tilde);
    c.lambda += delta_lambda;
    
    // Position correction : dx = w * n * dLambda
    const Vec2 dx = n * delta_lambda;
    if (w1 > 0.f) world.PlusEqualParticlePosition(i1, dx * w1);
    if (w2 > 0.f) world.PlusEqualParticlePosition(i2, -(dx * w2));
}

void XpbdSolver::SolveTriangleAreaConstraint(PhysicsWorld& world, const float inv_sdt_sq, XpbdTriangleAreaConstraint& c)
{
    const uint32_t i1 = c.p1, i2 = c.p2, i3 = c.p3; // Clockwise
    
    const float w1 = world.particle_inv_masses_[i1];
    const float w2 = world.particle_inv_masses_[i2];
    const float w3 = world.particle_inv_masses_[i3];
    
    if (w1 + w2 + w3 == 0.f) return;
    
    const Vec2 p1 = world.GetParticlePosition(i1);
    const Vec2 p2 = world.GetParticlePosition(i2);
    const Vec2 p3 = world.GetParticlePosition(i3);
    
    // Triangle Area
    const float current_area = 0.5f * (p1.x * (p3.y - p2.y) + p2.x * (p1.y - p3.y) + p3.x * (p2.y - p1.y));
    const float violation = current_area - c.rest_area;
    
    // Gradient of C
    const Vec2 grad1{ 0.5f * (p3.y - p2.y), 0.5f * (p2.x - p3.x) };
    const Vec2 grad2{ 0.5f * (p1.y - p3.y), 0.5f * (p3.x - p1.x) };
    const Vec2 grad3{ 0.5f * (p2.y - p1.y), 0.5f * (p1.x - p2.x) };

    const float grad_sq_sum = w1 * grad1.LengthSquared() + w2 * grad2.LengthSquared() + w3 * grad3.LengthSquared();
    
    if (grad_sq_sum < 1e-9f) return;
    
    // XPBD Position Correction
    const float alpha_tilde = c.compliance * inv_sdt_sq;
    const float delta_lambda = (-violation - alpha_tilde * c.lambda) / (grad_sq_sum + alpha_tilde);
    c.lambda += delta_lambda;
    
    if (w1 > 0.f) world.PlusEqualParticlePosition(i1, grad1 * (w1 * delta_lambda));
    if (w2 > 0.f) world.PlusEqualParticlePosition(i2, grad2 * (w2 * delta_lambda));
    if (w3 > 0.f) world.PlusEqualParticlePosition(i3, grad3 * (w3 * delta_lambda));
}

void XpbdSolver::SolveGlobalAreaConstraints(PhysicsWorld& world, const float inv_sdt_sq)
{
    // Particles arranged Clockwise.
    for (auto& c : world.global_area_constraints_)
    {
        if (c.particle_count < 3) return;

        float current_area = 0.f;
        for (uint16_t i = 0; i < c.particle_count; ++i)
        {
            const uint32_t curr = c.start_particle_idx + i;
            const uint32_t next = c.start_particle_idx + ((i + 1) % c.particle_count);
            const Vec2& p1 = world.GetParticlePosition(curr);
            const Vec2& p2 = world.GetParticlePosition(next);
        
            current_area += (p2.x * p1.y - p1.x * p2.y);
        }
        current_area *= 0.5f;

        const float violation = current_area - c.rest_area; // C(x) = A - A0

        // Gradients
        std::pmr::vector<Vec2> grads(c.particle_count, &world.frame_arena_);
        float grad_sq_sum = 0.f;

        for (uint16_t i = 0; i < c.particle_count; ++i)
        {
            const uint32_t prev_idx = c.start_particle_idx + ((i + c.particle_count - 1) % c.particle_count);
            const uint32_t next_idx = c.start_particle_idx + ((i + 1) % c.particle_count);

            const Vec2& p_prev = world.GetParticlePosition(prev_idx);
            const Vec2& p_next = world.GetParticlePosition(next_idx);

            // Grad_i = 0.5 * (y_{i-1} - y_{i+1}, x_{i+1} - x_{i-1})
            grads[i] = Vec2{ 0.5f * (p_prev.y - p_next.y), 0.5f * (p_next.x - p_prev.x) };

            const uint32_t curr_idx = c.start_particle_idx + i;
            const float w = world.particle_inv_masses_[curr_idx];
            grad_sq_sum += w * grads[i].LengthSquared();
        }

        if (grad_sq_sum < 1e-9f) return;

        const float alpha_tilde = c.compliance * inv_sdt_sq;
        const float delta_lambda = (-violation - alpha_tilde * c.lambda) / (grad_sq_sum + alpha_tilde);
        c.lambda += delta_lambda;
        
        for (uint16_t i = 0; i < c.particle_count; ++i)
        {
            const uint32_t curr_idx = c.start_particle_idx + i;
            const float w = world.particle_inv_masses_[curr_idx];
            if (w > 0.f)
            {
                world.PlusEqualParticlePosition(curr_idx, grads[i] * (w * delta_lambda));
            }
        }
    }
}

void XpbdSolver::SolveRigidParticleCollisionConstraints(PhysicsWorld& world, const float sub_dt, const float inv_sub_dt)
{
    for (auto& c : world.rigid_particle_collision_constraints_)
    {
        const float w_particle = world.particle_inv_masses_[c.particle_idx];
        
        const auto& rigid_body = world.rigid_bodies_[c.body_idx];
        Vec2 rb = c.contact_point - world.positions_[c.body_idx].val;
        const float rb_cross_n = rb.Cross(c.contact_normal);
        const float w_rigid = rigid_body.inv_mass + rigid_body.inv_inertia * (rb_cross_n * rb_cross_n);
        const float w_sum = w_particle + w_rigid;
        
        if (w_sum <= 1e-6f) continue;
            
        // Calculate current penetration depth.
        const float violation = (world.GetParticlePosition(c.particle_idx) - c.contact_point).Dot(c.contact_normal);
        if (violation >= 0.f) continue;

        const float delta_lambda = -violation / w_sum;

        // Move particle
        world.PlusEqualParticlePosition(c.particle_idx, c.contact_normal * (delta_lambda * w_particle));
        
        // Update rigid velocity (linear + angular)
        Vec2 impulse_n = c.contact_normal * delta_lambda * inv_sub_dt;
        world.velocities_[c.body_idx].val -= impulse_n * rigid_body.inv_mass;
        world.angular_velocities_[c.body_idx].val -= rigid_body.inv_inertia * rb.Cross(impulse_n);
        
        // ===== Tangent Friction =====
        
        // Compute relative velocity
        Vec2 particle_vel = (world.GetParticlePosition(c.particle_idx) - world.GetParticlePrevPosition(c.particle_idx)) * inv_sub_dt;
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
        const float j_n_applied = delta_lambda * inv_sub_dt; // Magnitude of impulse on normal direction
        
        float max_friction = rigid_body.friction * j_n_applied;
        const float j_t_final = std::min(j_t_desired, max_friction);
        
        // Move particle (convert magnitude of impulse back to position correction)
        world.PlusEqualParticlePosition(c.particle_idx, -(tangent_dir * (j_t_final * sub_dt) * w_particle));
        
        // Update rigid velocity
        Vec2 impulse_t = tangent_dir * j_t_final;
        world.velocities_[c.body_idx].val += impulse_t * rigid_body.inv_mass;
        world.angular_velocities_[c.body_idx].val += rigid_body.inv_inertia * rb.Cross(impulse_t);
    }
}

void XpbdSolver::SolveRigidParticleJointConstraints(PhysicsWorld& world, const float inv_sub_dt)
{
    for (auto& c : world.rigid_particle_joint_constraints_)
    {
        const float w_particle = world.particle_inv_masses_[c.particle_idx];
        
        // Get joint point of rigid body in world space.
        Vec2 rb = world.rots_[c.body_idx].Mul(c.body_local_anchor);
        Vec2 body_anchor_world = rb + world.positions_[c.body_idx].val; 
        
        // Calculate C.
        Vec2 body_anchor_to_particle = world.GetParticlePosition(c.particle_idx) - body_anchor_world;
        const float violation = body_anchor_to_particle.Length();
        if (violation < 1e-6f && violation > -1e-6f) continue;
        
        // Get Gradient.
        Vec2 dir = body_anchor_to_particle / violation;
        
        const auto& rigid_body = world.rigid_bodies_[c.body_idx];
        const float rb_cross_n = rb.Cross(dir);
        const float w_rigid = rigid_body.inv_mass + rigid_body.inv_inertia * (rb_cross_n * rb_cross_n);
        const float w_sum = w_particle + w_rigid;
        
        if (w_sum <= 1e-6f) continue;

        const float delta_lambda = -violation / w_sum;

        // Move particle
        world.PlusEqualParticlePosition(c.particle_idx, dir * (delta_lambda * w_particle));
        
        // Update rigid velocity (linear + angular)
        Vec2 impulse_n = dir * delta_lambda * inv_sub_dt;
        world.velocities_[c.body_idx].val -= impulse_n * rigid_body.inv_mass;
        world.angular_velocities_[c.body_idx].val -= rigid_body.inv_inertia * rb.Cross(impulse_n);
    }
}

void XpbdSolver::SolveRigidParticleAngularConstraints(PhysicsWorld& world, const float inv_sub_dt)
{
    for (auto& c : world.rigid_particle_angular_constraints_)
    {
        // Convert anchor to world space.
        const Vec2 anchor_offset = world.rots_[c.body_idx].Mul(c.anchor_local);
        const Vec2 anchor_world = anchor_offset + world.positions_[c.body_idx].val;

        // Anchor to Particle.
        const Vec2 r = world.GetParticlePosition(c.particle_idx) - anchor_world;

        const float r_sq = r.LengthSquared();
        if (r_sq < 1e-8f) return;

        // Convert rest angle direction to world space 
        const Vec2 rest_direction_world = world.rots_[c.body_idx].Mul(c.rest_direction_local);
        
        // Calculate angle Constraint C (Use Cross Product to save std::atan2 overhead)
        Vec2 r_normalized = r / std::sqrt(r_sq);
        const float violation = rest_direction_world.Cross(r_normalized);
        
        // Gradients
        const Vec2 grad_particle = rest_direction_world.NormalWithoutNormalized();
        const Vec2 grad_body = -grad_particle;
        const float grad_body_rot = -rest_direction_world.Dot(world.GetParticlePosition(c.particle_idx) - world.positions_[c.body_idx].val);
        
        const RigidBodyComponent rigid_body = world.rigid_bodies_[c.body_idx];
        const float particle_inv_mass = world.particle_inv_masses_[c.particle_idx];
        
        const float w_sum = (particle_inv_mass + rigid_body.inv_mass) * grad_particle.LengthSquared()
                            + rigid_body.inv_inertia * grad_body_rot * grad_body_rot;
        
        const float alpha = c.compliance * inv_sub_dt * inv_sub_dt;
        const float denominator = w_sum + alpha;

        if (denominator < 1e-8f) return;
        
        const float delta_lambda = (-violation - alpha * c.lambda) / denominator;
        c.lambda += delta_lambda;

        // Move particle
        world.PlusEqualParticlePosition(c.particle_idx, grad_particle * (particle_inv_mass * delta_lambda));
        
        // Update rigid velocity (linear + angular)
        const float impulse_magnitude = delta_lambda * inv_sub_dt;
        world.velocities_[c.body_idx].val += grad_body * impulse_magnitude * rigid_body.inv_mass;
        world.angular_velocities_[c.body_idx].val += grad_body_rot * impulse_magnitude * rigid_body.inv_inertia;
    }
}

void XpbdSolver::FinalizeParticles(PhysicsWorld& world, const float inv_sdt)
{
    const uint32_t count = world.particle_inv_masses_.size();
    
    auto index_range = std::views::iota(uint32_t{0}, count);
    std::for_each(std::execution::par_unseq, index_range.begin(), index_range.end(),[&](auto i)
    {
        if (world.particle_inv_masses_[i] == 0.f) return;
    
        // Update the velocity based on the position correction.
        world.particle_velocities_[i] = (world.GetParticlePosition(i) - world.GetParticlePrevPosition(i)) * inv_sdt;
    });
}

void XpbdSolver::BuildBatches(PhysicsWorld& world, const ParticleEdgeCollisionConstraintSoAView& particle_edge_collision_constraint_soa_view,
                              int body_count, std::pmr::vector<std::vector<int32_t>>& batches)
{
    // used_by_batch[i][particle] = Does this batch already contain this particle?
    std::pmr::vector<std::vector<uint8_t>> used_by_batch(&world.frame_arena_);
    
    for (auto i = 0; i < particle_edge_collision_constraint_soa_view.Size(); ++i)
    {
        const int32_t p_idx = particle_edge_collision_constraint_soa_view.p_idx[i];
        const int32_t a_idx = particle_edge_collision_constraint_soa_view.a_idx[i];
        const int32_t b_idx = particle_edge_collision_constraint_soa_view.b_idx[i];

        const bool is_p_static = world.particle_inv_masses_[p_idx] == 0.f;
        const bool is_a_static = world.particle_inv_masses_[a_idx] == 0.f;
        const bool is_b_static = world.particle_inv_masses_[b_idx] == 0.f;
        
        int chosen = -1;

        for (int j = 0; j < static_cast<int>(batches.size()); ++j)
        {
            bool conflict = false;

            if (!is_p_static && used_by_batch[j][p_idx])
            {
                conflict = true;
            }
            
            if (!is_a_static && used_by_batch[j][a_idx])
            {
                conflict = true;
            }
            
            if (!is_b_static && used_by_batch[j][b_idx])
            {
                conflict = true;
            }

            if (!conflict)
            {
                chosen = j;
                break;
            }
        }

        if (chosen == -1)
        {
            chosen = static_cast<int>(batches.size());
            batches.emplace_back();
            used_by_batch.emplace_back(body_count, 0);
        }

        batches[chosen].push_back(i);

        if (!is_p_static) used_by_batch[chosen][p_idx] = 1;
        if (!is_a_static) used_by_batch[chosen][a_idx] = 1;
        if (!is_b_static) used_by_batch[chosen][b_idx] = 1;
    }
}

__attribute__((target("avx,avx2")))
void XpbdSolver::SolveParticleEdgeBatchAvx2(PhysicsWorld& world, const std::span<int32_t> batch)
{
    using Clock = std::chrono::high_resolution_clock;
    using Duration = std::chrono::duration<double, std::milli>;
    
    auto& [x, y, prev_x, prev_y] = world.particle_soa_view_;
    const auto& [p_idx, a_idx, b_idx, ct, normal_x, normal_y] = 
        world.particle_edge_collision_constraint_soa_view_;
    
    constexpr size_t w = 8;

    const __m256 zero = _mm256_setzero_ps();
    const __m256 one  = _mm256_set1_ps(1.0f);
    const __m256 eps  = _mm256_set1_ps(1e-6f);
    const __m256 thick = _mm256_set1_ps(physics_config::kTwoParticlesThickness);
    
    for (size_t i = 0; i + w <= batch.size(); i += w) {
        // 1. Load 8 independent constraints.
        auto start = Clock::now();
        __m256i ci = _mm256_loadu_si256( reinterpret_cast<const __m256i*>( batch.data() + i));
        const __m256i ip = _mm256_i32gather_epi32( p_idx.data(), ci, 4);
        const __m256i ia = _mm256_i32gather_epi32( a_idx.data(), ci, 4);
        const __m256i ib = _mm256_i32gather_epi32( b_idx.data(), ci, 4);
        const __m256 t = _mm256_i32gather_ps( ct.data(), ci, 4);
        const __m256 nx = _mm256_i32gather_ps( normal_x.data(), ci, 4);
        const __m256 ny = _mm256_i32gather_ps( normal_y.data(), ci, 4);
        
        const __m256 q  = _mm256_sub_ps(one, t);

        // 2. Gather current positions and inverse masses.
        __m256 px = Gather8(x, ip);
        __m256 py = Gather8(y, ip);
        __m256 ax = Gather8(x, ia);
        __m256 ay = Gather8(y, ia);
        __m256 bx = Gather8(x, ib);
        __m256 by = Gather8(y, ib);

        const __m256 wp = Gather8(world.particle_inv_masses_, ip);
        const __m256 wa = Gather8(world.particle_inv_masses_, ia);
        const __m256 wb = Gather8(world.particle_inv_masses_, ib);
        world.profile_data_.simd_gather_ms += Duration(Clock::now() - start).count();

        // 3. Compute penetration C(violation).
        start = Clock::now();
        const __m256 abx = _mm256_sub_ps(bx, ax);
        const __m256 aby = _mm256_sub_ps(by, ay);

        const __m256 proj_x = _mm256_add_ps( ax, _mm256_mul_ps(abx, t));
        const __m256 proj_y = _mm256_add_ps( ay, _mm256_mul_ps(aby, t));

        const __m256 dx = _mm256_sub_ps(px, proj_x);
        const __m256 dy = _mm256_sub_ps(py, proj_y);

        const __m256 violation = _mm256_sub_ps( _mm256_add_ps( _mm256_mul_ps(dx, nx), _mm256_mul_ps(dy, ny)), thick);

        // 4. Compute effective inverse mass.
        const __m256 q2 = _mm256_mul_ps(q, q);
        const __m256 t2 = _mm256_mul_ps(t, t);

        const __m256 w_sum = _mm256_add_ps( wp, _mm256_add_ps( _mm256_mul_ps(q2, wa), _mm256_mul_ps(t2, wb)));

        // active = (C < 0) && (w_sum >= 1e-6)
        const __m256 penetrating = _mm256_cmp_ps( violation, zero, _CMP_LT_OQ);
        const __m256 mass_ok = _mm256_cmp_ps( w_sum, eps, _CMP_GE_OQ);

        const __m256 active = _mm256_and_ps( penetrating, mass_ok);

        // Safe denominator for inactive lanes.
        const __m256 safe_w_sum = _mm256_blendv_ps( one, w_sum, active);

        __m256 dl = _mm256_div_ps( _mm256_sub_ps(zero, violation), safe_w_sum);

        dl = _mm256_and_ps(dl, active);

        // 5. Normal position correction.
        px = _mm256_add_ps( px, _mm256_mul_ps(nx, _mm256_mul_ps(wp, dl)));
        py = _mm256_add_ps( py, _mm256_mul_ps(ny, _mm256_mul_ps(wp, dl)));

        ax = _mm256_sub_ps( ax, _mm256_mul_ps(nx, _mm256_mul_ps(q, _mm256_mul_ps(wa, dl))));
        ay = _mm256_sub_ps( ay, _mm256_mul_ps(ny, _mm256_mul_ps(q, _mm256_mul_ps(wa, dl))));

        bx = _mm256_sub_ps( bx, _mm256_mul_ps(nx, _mm256_mul_ps(t, _mm256_mul_ps(wb, dl))));
        by = _mm256_sub_ps( by, _mm256_mul_ps(ny, _mm256_mul_ps(t, _mm256_mul_ps(wb, dl))));

        // 6. Tangent friction.
        // Important: use the positions AFTER normal correction.
        const __m256 pdx = _mm256_sub_ps( px, Gather8(prev_x, ip));
        const __m256 pdy = _mm256_sub_ps( py, Gather8(prev_y, ip));

        const __m256 adx = _mm256_sub_ps( ax, Gather8(prev_x, ia));
        const __m256 ady = _mm256_sub_ps( ay, Gather8(prev_y, ia));

        const __m256 bdx = _mm256_sub_ps( bx, Gather8(prev_x, ib));
        const __m256 bdy = _mm256_sub_ps( by, Gather8(prev_y, ib));

        const __m256 edge_dx = _mm256_add_ps( _mm256_mul_ps(adx, q), _mm256_mul_ps(bdx, t));
        const __m256 edge_dy = _mm256_add_ps( _mm256_mul_ps(ady, q), _mm256_mul_ps(bdy, t));

        const __m256 rel_x = _mm256_sub_ps(pdx, edge_dx);
        const __m256 rel_y = _mm256_sub_ps(pdy, edge_dy);

        const __m256 normal_dot = _mm256_add_ps( _mm256_mul_ps(rel_x, nx), _mm256_mul_ps(rel_y, ny));

        __m256 tx = _mm256_sub_ps( rel_x, _mm256_mul_ps(nx, normal_dot));
        __m256 ty = _mm256_sub_ps( rel_y, _mm256_mul_ps(ny, normal_dot));

        const __m256 tangent_len = _mm256_sqrt_ps( _mm256_add_ps( _mm256_mul_ps(tx, tx), _mm256_mul_ps(ty, ty)));

        const __m256 friction_active = _mm256_and_ps( active, _mm256_cmp_ps(tangent_len, eps, _CMP_GT_OQ));

        const __m256 safe_len = _mm256_blendv_ps( one, tangent_len, friction_active);

        tx = _mm256_div_ps(tx, safe_len);
        ty = _mm256_div_ps(ty, safe_len);

        const __m256 mu = _mm256_sqrt_ps( _mm256_mul_ps( Gather8(world.particle_frictions_, ip), Gather8(world.particle_frictions_, ia)));

        const __m256 normal_dx = _mm256_sub_ps(zero, violation);

        const __m256 final_friction = _mm256_min_ps( tangent_len, _mm256_mul_ps(normal_dx, mu));

        __m256 dt = _mm256_div_ps( final_friction, safe_w_sum);

        dt = _mm256_and_ps(dt, friction_active);

        px = _mm256_sub_ps( px, _mm256_mul_ps(tx, _mm256_mul_ps(dt, wp)));
        py = _mm256_sub_ps( py, _mm256_mul_ps(ty, _mm256_mul_ps(dt, wp)));

        ax = _mm256_add_ps( ax, _mm256_mul_ps(tx, _mm256_mul_ps(dt, _mm256_mul_ps(q, wa))));
        ay = _mm256_add_ps( ay, _mm256_mul_ps(ty, _mm256_mul_ps(dt, _mm256_mul_ps(q, wa))));

        bx = _mm256_add_ps( bx, _mm256_mul_ps(tx, _mm256_mul_ps(dt, _mm256_mul_ps(t, wb))));
        by = _mm256_add_ps( by, _mm256_mul_ps(ty, _mm256_mul_ps(dt, _mm256_mul_ps(t, wb))));
        world.profile_data_.simd_compute_ms += Duration(Clock::now() - start).count();
        
        // 7. Save indices and final positions.
        // AVX2 has no native scatter instruction.
        start = Clock::now();
        alignas(32) int32_t pi[8], ai[8], bi[8];
        alignas(32) float px_out[8], py_out[8];
        alignas(32) float ax_out[8], ay_out[8];
        alignas(32) float bx_out[8], by_out[8];

        _mm256_store_si256( reinterpret_cast<__m256i*>(pi), ip);
        _mm256_store_si256( reinterpret_cast<__m256i*>(ai), ia);
        _mm256_store_si256( reinterpret_cast<__m256i*>(bi), ib);

        _mm256_store_ps(px_out, px);
        _mm256_store_ps(py_out, py);
        _mm256_store_ps(ax_out, ax);
        _mm256_store_ps(ay_out, ay);
        _mm256_store_ps(bx_out, bx);
        _mm256_store_ps(by_out, by);

        for (int lane = 0; lane < 8; ++lane) {
            x[pi[lane]] = px_out[lane];
            y[pi[lane]] = py_out[lane];

            x[ai[lane]] = ax_out[lane];
            y[ai[lane]] = ay_out[lane];

            x[bi[lane]] = bx_out[lane];
            y[bi[lane]] = by_out[lane];
        }
        world.profile_data_.simd_scatter_ms += Duration(Clock::now() - start).count();
    }

    // Handle the remaining end - begin (mod 8) constraints with the original scalar implementation.
    auto start = Clock::now();
    auto r = batch.size() % w;
    auto r_batch = std::span(batch).subspan(batch.size() - r, r);
    SolveParticleEdgeCollisionConstraints(world, r_batch);
    world.profile_data_.handle_remain_ms += Duration(Clock::now() - start).count();
}
