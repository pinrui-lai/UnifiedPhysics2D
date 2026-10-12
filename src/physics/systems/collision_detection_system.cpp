#include <algorithm>
#include <execution>

#include "physics/systems/collision_detection_system.h"
#include "physics/collision/collision_broad_phase.h"
#include "physics/collision/collision_narrow_phase.h"
#include "physics/physics_world.h"

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
    
    std::pmr::vector<std::pair<uint32_t, uint32_t>> rigid_particle_contacts(&world.frame_arena_);
    rigid_particle_contacts.reserve((rigid_count + soft_count) * 5);
    
    std::pmr::vector<std::pair<uint32_t, uint32_t>> soft_soft_contacts(&world.frame_arena_);
    soft_soft_contacts.reserve(soft_count * 5);
    world.profile_data_.build_bvh_ms += Duration(Clock::now() - start).count();
    
    start = Clock::now();
    // Query rigid bodies for rigid-rigid and rigid-soft contact.
    for (uint32_t i = 0; i < rigid_count; ++i)
    {
        CollisionBroadPhase::QueryBvh(world, world_aabbs, i, rigid_particle_contacts, soft_soft_contacts);
    }
    
    // Query soft bodies for soft-soft contact.
    for (uint32_t i = 0; i < soft_count; ++i)
    {
        const uint32_t idx = rigid_count + i;
        CollisionBroadPhase::QueryBvh(world, world_aabbs, idx, rigid_particle_contacts, soft_soft_contacts);
    }
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
    CollisionNarrowPhase::CheckSoftSoftCollision(world, soft_soft_contacts);
    world.profile_data_.soft_soft_ms += Duration(Clock::now() - start).count();
    
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
    if (world.is_multi_thread_){
        std::for_each(std::execution::par_unseq, world.manifolds_.begin(), world.manifolds_.end(), process);    
    }
    else{
        std::for_each(std::execution::seq, world.manifolds_.begin(), world.manifolds_.end(), process);
    }
    world.profile_data_.rigid_rigid_ms += Duration(Clock::now() - start).count();
    world.profile_data_.narrow_phase_ms = world.profile_data_.soft_soft_ms + world.profile_data_.rigid_soft_ms + world.profile_data_.rigid_rigid_ms;
}
