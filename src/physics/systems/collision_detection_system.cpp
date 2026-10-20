#include <algorithm>
#include <execution>
#include <ranges>

#include "physics/systems/collision_detection_system.h"
#include "physics/collision/collision_broad_phase.h"
#include "physics/collision/collision_narrow_phase.h"
#include "physics/physics_world.h"

void CollisionDetectionSystem::GenerateContacts(PhysicsWorld& world, std::span<const Aabb> world_aabbs, std::pmr::vector<std::pair<uint32_t, uint32_t>>& rigid_rigid_candidates,
    std::pmr::vector<std::pair<uint32_t, uint32_t>>& rigid_soft_contacts, std::pmr::vector<std::pair<uint32_t, uint32_t>>& soft_soft_contacts)
{
    const uint32_t rigid_count = static_cast<uint32_t>(world.positions_.size());
    const uint32_t soft_count = static_cast<uint32_t>(world.soft_body_handles_.size());
    const uint32_t query_count = rigid_count + soft_count;

    if (query_count == 0) return;

    constexpr uint32_t queries_per_chunk = 16;
    const uint32_t chunk_count = query_count / queries_per_chunk + (query_count % queries_per_chunk != 0);
    
    auto& query_outputs = world.query_outputs_;
    
    // Resize buffer only when more capacity is needed.
    if (query_outputs.size() < chunk_count)
        query_outputs.resize(chunk_count);

    for (uint32_t c = 0; c < chunk_count; ++c)
    {
        query_outputs[c].Clear();
    }

    // The output of each chunk is independent.
    const auto chunks = std::views::iota(uint32_t{0}, chunk_count);
    std::for_each( std::execution::par_unseq, chunks.begin(), chunks.end(), [&](uint32_t chunk_id)
    {
        auto& out = query_outputs[chunk_id];

        const uint32_t begin = chunk_id * queries_per_chunk;
        const uint32_t end = begin + std::min( queries_per_chunk, query_count - begin);

        for (uint32_t idx = begin; idx < end; ++idx)
        {
            CollisionBroadPhase::QueryBvh( world, world_aabbs, idx, out.rr, out.rs, out.ss);
        }
    });

    uint32_t rr_total = 0;
    uint32_t rs_total = 0;
    uint32_t ss_total = 0;

    for (uint32_t c = 0; c < chunk_count; ++c)
    {
        const auto& out = query_outputs[c];

        rr_total += out.rr.size();
        rs_total += out.rs.size();
        ss_total += out.ss.size();
    }
    
    rigid_rigid_candidates.reserve(rr_total);
    rigid_soft_contacts.reserve(rs_total);
    soft_soft_contacts.reserve(ss_total);

    // Collect results.
    for (uint32_t c = 0; c < chunk_count; ++c)
    {
        const auto& out = query_outputs[c];

        rigid_rigid_candidates.insert( rigid_rigid_candidates.end(), out.rr.begin(), out.rr.end());
        rigid_soft_contacts.insert( rigid_soft_contacts.end(), out.rs.begin(), out.rs.end());
        soft_soft_contacts.insert( soft_soft_contacts.end(), out.ss.begin(), out.ss.end());
    }
    
    // Update world.manifolds_.
    for (const auto& [a, b] : rigid_rigid_candidates)
    {
        const uint64_t key = CollisionBroadPhase::MakePairKey(a, b);

        if (auto it = world.manifold_map_.find(key); it != world.manifold_map_.end())
        {
            world.manifolds_[it->second].is_overlapping = true;
        }
        else
        {
            const size_t idx = world.manifolds_.size();

            world.manifolds_.emplace_back(a, b);
            world.manifold_map_.emplace(key, idx);
        }
    }
}

void CollisionDetectionSystem::CheckCollision(PhysicsWorld& world)
{
    using Clock = std::chrono::high_resolution_clock;
    using Duration = std::chrono::duration<double, std::milli>;
    
    world.debug_incident_edges_.clear();
    world.debug_reference_edges_.clear();

    // ===== Broad-Phase =====
    auto start = Clock::now();
    const uint32_t rigid_count = world.positions_.size();
    const uint32_t soft_count = world.soft_body_handles_.size();
    const uint32_t total_count = rigid_count + soft_count;
    if (total_count == 0) return;
    
    std::pmr::vector<Aabb> world_aabbs(total_count, &world.frame_arena_);
    CollisionBroadPhase::BuildWorldBvh(world, world_aabbs);
    
    std::pmr::vector<std::pair<uint32_t, uint32_t>> rigid_rigid_candidates(&world.frame_arena_);
    std::pmr::vector<std::pair<uint32_t, uint32_t>> rigid_particle_contacts(&world.frame_arena_);
    std::pmr::vector<std::pair<uint32_t, uint32_t>> soft_soft_contacts(&world.frame_arena_);
    
    world.profile_data_.build_bvh_ms += Duration(Clock::now() - start).count();
    
    start = Clock::now();
    GenerateContacts(world, world_aabbs, rigid_rigid_candidates, rigid_particle_contacts, soft_soft_contacts);
    world.profile_data_.query_bvh_ms += Duration(Clock::now() - start).count();
    
    // Remove manifold whose AABBs no longer overlap.
    start = Clock::now();
    CollisionBroadPhase::CleanupStaleManifolds(world);
    world.profile_data_.erase_manifold_ms += Duration(Clock::now() - start).count();
    world.profile_data_.broad_phase_ms = (world.profile_data_.build_bvh_ms + world.profile_data_.query_bvh_ms + world.profile_data_.erase_manifold_ms);
    
    // ===== Narrow-Phase =====
    // Rigid vs Soft Particles (SDF).
    start = Clock::now();
    for (const auto& [fst, snd] : rigid_particle_contacts)
    {
        CollisionNarrowPhase::CheckRigidParticleCollision(world, fst, snd);
    }
    world.profile_data_.rigid_soft_ms += Duration(Clock::now() - start).count();
    
    // Soft vs Soft.
    start = Clock::now();
    CollisionNarrowPhase::UpdateParticleEdgeAabbs(world, soft_soft_contacts);
    world.profile_data_.ss_update_local_aabb_bvh += Duration(Clock::now() - start).count();
    start = Clock::now();
    CollisionNarrowPhase::CheckSoftSoftCollision(world, soft_soft_contacts);
    world.profile_data_.ss_detection += Duration(Clock::now() - start).count();
    world.profile_data_.soft_soft_ms = world.profile_data_.ss_update_local_aabb_bvh + world.profile_data_.ss_detection;
    
    // Rigid vs Rigid (GJK + EPA + Clipping).
    auto process = [&world](auto& manifold)
    {
        if (world.rigid_bodies_[manifold.a].inv_mass == 0.f && world.rigid_bodies_[manifold.b].inv_mass == 0.f) return;
        
        // Store old contacts and contact_count from the previous frame.
        const std::array<Contact, 2> old_contacts = manifold.contacts;
        const uint8_t old_contact_count = manifold.contact_count;
        
        // Write new contacts of the current frame in manifold.
        CollisionNarrowPhase::Gjk(world, manifold);
        
        // Match Feature ID and inherit old impulse.
        for (uint8_t i = 0; i < manifold.contact_count; ++i)
        {
            Contact& new_c = manifold.contacts[i];
            
            for (uint8_t j = 0; j < old_contact_count; ++j)
            {
                const Contact& old_c = old_contacts[j];
                // ID matching successful, inheriting the old impulse.
                if (new_c.id.key == old_c.id.key)
                {
                    new_c.normal_impulse_sum = old_c.normal_impulse_sum;
                    new_c.tangent_impulse_sum = old_c.tangent_impulse_sum;
                    break;
                }
            }
        }
    };
    
    start = Clock::now();
    if (world.use_single_thread_){
        std::for_each(std::execution::seq, world.manifolds_.begin(), world.manifolds_.end(), process);
    }
    else{
        std::for_each(std::execution::par_unseq, world.manifolds_.begin(), world.manifolds_.end(), process);    
    }
    world.profile_data_.rigid_rigid_ms += Duration(Clock::now() - start).count();
    world.profile_data_.narrow_phase_ms = world.profile_data_.soft_soft_ms + world.profile_data_.rigid_soft_ms + world.profile_data_.rigid_rigid_ms;
}
