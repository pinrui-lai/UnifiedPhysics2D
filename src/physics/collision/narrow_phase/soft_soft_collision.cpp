#include <execution>
#include <ranges>

#include "physics/collision/narrow_phase/soft_soft_collision.h"
#include "physics/collision/broad_phase/collision_broad_phase.h"
#include "physics/physics_world.h"

void SoftSoftCollision::UpdateParticleEdgeAabbs(PhysicsWorld& world, const std::pmr::vector<std::pair<uint32_t, uint32_t>>& soft_soft_contacts)
{
    const uint32_t num_bodies = world.soft_body_handles_.size();
    if (num_bodies == 0 || soft_soft_contacts.empty()) return;
    
    // Mark soft bodies whose AABB overlaps with others.
    std::pmr::vector<uint8_t> is_body_active(num_bodies, 0, &world.frame_arena_);
    for (const auto& [a_idx, b_idx] : soft_soft_contacts)
    {
        is_body_active[a_idx] = 1;
        is_body_active[b_idx] = 1;
    }
    
    const uint32_t total_surface_edges = world.surface_edges_.size();
    
    world.particle_aabbs_.resize(world.particle_inv_masses_.size());
    world.surface_edge_aabbs_.resize(total_surface_edges);
    world.soft_body_surface_particle_bvhs_.resize(num_bodies);
    
    // Update the AABB only for soft bodies marked as Active.
    auto body_range = std::views::iota(uint32_t{0}, num_bodies);
    std::for_each(std::execution::par, body_range.begin(), body_range.end(), [&](size_t body_idx) 
    {
        if (!is_body_active[body_idx]) return;

        const auto& body = world.soft_body_handles_[body_idx];

        // Update surface particle AABBs
        for (uint32_t p = 0; p < body.surface_particle_count; ++p) 
        {
            const uint32_t particle_idx = body.start_surface_particle_idx + p;
            const Vec2 pos = world.GetParticlePosition(particle_idx);

            world.particle_aabbs_[particle_idx] = {
                pos.x - physics_config::kTwoParticlesThickness, pos.y - physics_config::kTwoParticlesThickness,
                pos.x + physics_config::kTwoParticlesThickness, pos.y + physics_config::kTwoParticlesThickness
            };
        }

        // Update surface edge AABBs
        for (uint16_t e = 0; e < body.surface_edge_count; ++e) 
        {
            const uint32_t global_e_idx = body.start_surface_edge_idx + e;
            const auto& edge = world.surface_edges_[global_e_idx];
            const Vec2 a_pos = world.GetParticlePosition(edge.first);
            const Vec2 b_pos = world.GetParticlePosition(edge.second);

            world.surface_edge_aabbs_[global_e_idx] = {
                std::min(a_pos.x, b_pos.x) - physics_config::kTwoParticlesThickness,
                std::min(a_pos.y, b_pos.y) - physics_config::kTwoParticlesThickness,
                std::max(a_pos.x, b_pos.x) + physics_config::kTwoParticlesThickness,
                std::max(a_pos.y, b_pos.y) + physics_config::kTwoParticlesThickness
            };
        }
        
        // Update soft body surface particle BVH
        if (world.use_local_bvh_)
        {
            CollisionBroadPhase::BuildLocalBvh(std::span(world.particle_aabbs_).subspan(
                body.start_surface_particle_idx, body.surface_particle_count), world.soft_body_surface_particle_bvhs_[body_idx]);    
        }
    });
}

void SoftSoftCollision::CheckParticleEdgeCollision(const PhysicsWorld& world, const SoftBodyHandle& soft_body_a, const SoftBodyHandle& soft_body_b, std::span<const Aabb> body_b_edge_aabbs,
    std::span<const Aabb> body_a_particle_aabbs, const std::span<BvhNode> body_a_surface_particle_bvh, std::vector<XpbdParticleEdgeCollisionConstraint>& out_contacts)
{
    constexpr float total_thickness_sq = physics_config::kTwoParticlesThickness * physics_config::kTwoParticlesThickness;
    const bool is_b_closed = world.surface_edges_[soft_body_b.start_surface_edge_idx + soft_body_b.surface_edge_count - 1].second == 
        world.surface_edges_[soft_body_b.start_surface_edge_idx].first;
    
    uint32_t stack[32];
    for (uint16_t j = 0; j < soft_body_b.surface_edge_count; ++j)
    {
        const auto edge_idx = soft_body_b.start_surface_edge_idx + j;
        const auto [fst, snd] = world.surface_edges_[edge_idx];
            
        const Vec2 a_pos = world.GetParticlePosition(fst);
        const Vec2 b_pos = world.GetParticlePosition(snd);
            
        Vec2 ab = b_pos - a_pos;
        const float ab_len_sq = ab.LengthSquared();
        if (ab_len_sq < 1e-6f) continue;
        
        uint32_t stack_ptr = 0;
        stack[stack_ptr++] = 0;
        
        bool is_found = false;
        
        auto check_and_add_contact = [&](const uint32_t local_particle_idx) 
        {
            const auto particle_idx = soft_body_a.start_surface_particle_idx + local_particle_idx;
            const Vec2 particle_pos = world.GetParticlePosition(particle_idx);

            float t = (particle_pos - a_pos).Dot(ab) / ab_len_sq;
            if (t < -0.05f || t > 1.05f) return;
            
            t = std::clamp(t, 0.f, 1.f);
            
            const Vec2 proj = a_pos + ab * t;
            const Vec2 diff = particle_pos - proj;

            const float dist_sq = diff.LengthSquared();
            // if (dist_sq > total_thickness_sq || dist_sq < 1e-12f) return;
            if (dist_sq > total_thickness_sq) return;
            
            Vec2 normal = ab.Normal();
            if (!is_b_closed)
            {
                if (dist_sq > 1e-8f)
                {
                    normal = diff / std::sqrt(dist_sq);
                }
                else
                {
                    const Vec2 prev_p_pos = world.GetParticlePrevPosition(particle_idx);
                    const Vec2 prev_a_pos = world.GetParticlePrevPosition(fst);
                    const Vec2 prev_b_pos = world.GetParticlePrevPosition(snd);
                    const float previous_side  = (prev_p_pos - prev_a_pos).Dot((prev_b_pos - prev_a_pos).NormalWithoutNormalized());
                    normal = previous_side >= 0 ? normal : -normal; 
                }
            }

            out_contacts.emplace_back(normal, particle_idx, fst, snd, t);
            is_found = true;
        };
        
        if (world.use_local_bvh_)
        {
            // Traverse BVH
            while (stack_ptr > 0 && !is_found)
            {
                const uint32_t curr = stack[--stack_ptr];
                const auto& node = body_a_surface_particle_bvh[curr];
            
                if (!Aabb::Overlap(node.aabb, body_b_edge_aabbs[j])) continue;
                
                if (node.IsLeaf())
                {
                    check_and_add_contact(node.leaf_id);
                }
                else
                {
                    stack[stack_ptr++] = node.left_child;
                    stack[stack_ptr++] = node.right_child;
                }
            }
        }
        else
        {
            // Brute Force
            for (uint16_t i = 0; i < soft_body_a.surface_particle_count; ++i)
            {
                if (!Aabb::Overlap(body_a_particle_aabbs[i], body_b_edge_aabbs[j])) continue;
            
                check_and_add_contact(i);
                if (is_found) break;
            }
        }
    }
}

void SoftSoftCollision::CheckSoftSoftCollision(PhysicsWorld& world, const std::pmr::vector<std::pair<uint32_t, uint32_t>>& pairs)
{
    const uint32_t num_pairs = pairs.size();
    if (num_pairs == 0) return;

    auto get_body_particle_aabbs = [&](const SoftBodyHandle& body) -> std::span<const Aabb> {
        return std::span<const Aabb>(world.particle_aabbs_).subspan(body.start_surface_particle_idx, body.surface_particle_count);
    };

    auto get_body_edge_aabbs = [&](const SoftBodyHandle& body) -> std::span<const Aabb> {
        return std::span<const Aabb>(world.surface_edge_aabbs_).subspan(body.start_surface_edge_idx, body.surface_edge_count);
    };
    
    // Maintain historical peak capacity
    static uint32_t estimated_capacity = 4096;
    const uint32_t required_capacity = std::max(num_pairs * 16, estimated_capacity);
    world.particle_edge_collision_constraint_soa_view_.Resize(required_capacity);

    std::atomic<uint32_t> total_new_contacts{0};
    auto index_range = std::views::iota(uint32_t{0}, num_pairs);

    auto fill_contacts = [&](const uint32_t i) 
    {
        thread_local std::vector<XpbdParticleEdgeCollisionConstraint> local_contacts;
        local_contacts.clear(); 

        const auto& body_a_handle = world.soft_body_handles_[pairs[i].first];
        const auto& body_b_handle = world.soft_body_handles_[pairs[i].second];
        
        const auto a_edge_aabbs = get_body_edge_aabbs(body_a_handle);
        const auto b_edge_aabbs = get_body_edge_aabbs(body_b_handle);
        const auto a_particle_aabbs = get_body_particle_aabbs(body_a_handle);
        const auto b_particle_aabbs = get_body_particle_aabbs(body_b_handle);
        auto& body_a_surface_particle_bvh = world.soft_body_surface_particle_bvhs_[pairs[i].first];
        auto& body_b_surface_particle_bvh = world.soft_body_surface_particle_bvhs_[pairs[i].second];
        
        CheckParticleEdgeCollision(world, body_a_handle, body_b_handle, b_edge_aabbs, a_particle_aabbs, body_a_surface_particle_bvh, local_contacts);
        CheckParticleEdgeCollision(world, body_b_handle, body_a_handle, a_edge_aabbs, b_particle_aabbs, body_b_surface_particle_bvh, local_contacts);

        if (local_contacts.empty()) return;
        
        const uint32_t count = local_contacts.size();
        const uint32_t write_idx = total_new_contacts.fetch_add(count, std::memory_order_relaxed);
        
        if (write_idx + count <= required_capacity) 
        {
            for (uint32_t j = 0; j < count; ++j)
            {
                const uint32_t idx = write_idx + j;
                world.particle_edge_collision_constraint_soa_view_.SetConstraint(idx, local_contacts[j]);
            }
        }
    };

    if (world.use_single_thread_) {
        std::for_each(std::execution::seq, index_range.begin(), index_range.end(), fill_contacts);
    } else {
        std::for_each(std::execution::par, index_range.begin(), index_range.end(), fill_contacts);
    }

    const uint32_t actual_new_contacts = total_new_contacts.load(std::memory_order_relaxed);
    
    // Adjust next estimated_capacity.
    estimated_capacity = std::max(estimated_capacity, actual_new_contacts * 2);
    
    // Shrink `world.particle_edge_contacts_` back to the actual number of constraints.
    world.particle_edge_collision_constraint_soa_view_.Resize(actual_new_contacts);
}