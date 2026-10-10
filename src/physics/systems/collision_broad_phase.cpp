#include <execution>
#include <ranges>

#include "collision_broad_phase.h"
#include "physics_world.h"

void CollisionBroadPhase::BuildWorldBvh(PhysicsWorld& world, std::span<Aabb> world_space_aabbs)
{
    const uint32_t rigid_count = world.positions_.size();
    const uint32_t soft_count = world.soft_body_handles_.size();
    
    world.bvh_tree_nodes_.clear();
    
    // Rigid Body world AABBs.
    auto index_range = std::views::iota(uint32_t{0}, rigid_count);
    std::for_each(std::execution::par_unseq, index_range.begin(), index_range.end(), [&](const uint32_t i) {
        std::visit([&](auto& s)
        {
            using S = std::decay_t<decltype(s)>;
            
            if constexpr (std::is_same_v<S, CircleShape>)
            {
                const Vec2 pos = world.positions_[i].val;
                auto [local_min_x, local_min_y, local_max_x, local_max_y] = world.local_aabbs_[i];
                world_space_aabbs[i] = {local_min_x + pos.x, local_min_y + pos.y,
                    local_max_x + pos.x, local_max_y + pos.y};
            }
            else
            {
                world_space_aabbs[i] = ComputeFastAabbFromLocalAabb(world.positions_[i].val, world.rots_[i], world.local_aabbs_[i]);
            }
        }, world.shapes_[i]);
    });
    
    // XPBD Soft Body world AABBs.
    index_range = std::views::iota(uint32_t{0}, soft_count);
    std::for_each(std::execution::par_unseq, index_range.begin(), index_range.end(), [&](const uint32_t i) {
        const uint32_t idx = rigid_count + i;
        
        float min_x = std::numeric_limits<float>::max();
        float min_y = std::numeric_limits<float>::max();
        float max_x = std::numeric_limits<float>::lowest();
        float max_y = std::numeric_limits<float>::lowest();
        
        for (const auto& pos : world.GetSoftBodyParticlePositions(i))
        {
            min_x = std::min(min_x, pos.x);
            min_y = std::min(min_y, pos.y);
            max_x = std::max(max_x, pos.x);
            max_y = std::max(max_y, pos.y);
        }

         world_space_aabbs[idx] = {
            min_x - physics_config::kTwoParticlesThickness, min_y - physics_config::kTwoParticlesThickness,
            max_x + physics_config::kTwoParticlesThickness, max_y + physics_config::kTwoParticlesThickness};
    });
    
    // Get Scene Bounding Box (boundary of all bodies in the world).
    Aabb scene_aabb = world_space_aabbs[0];
    for (const auto& world_aabb : world_space_aabbs) {
        scene_aabb = Aabb::Combine(scene_aabb, world_aabb);
    }
    const float inv_scene_aabb_width = 1 / (scene_aabb.max_x - scene_aabb.min_x);
    const float inv_scene_aabb_height = 1 / (scene_aabb.max_y - scene_aabb.min_y);
    
    // Get Morton Code and create sort item.
    std::pmr::vector<Item> items(world_space_aabbs.size(), &world.frame_arena_);
    
    index_range = std::views::iota(uint32_t{0}, rigid_count);
    std::for_each(std::execution::par_unseq, index_range.begin(), index_range.end(), [&](const uint32_t i)
    {
        items[i] = {i, GetMortonCode(world.positions_[i].val, scene_aabb.min_x, scene_aabb.min_y, inv_scene_aabb_width, inv_scene_aabb_height)};
    });

    index_range = std::views::iota(uint32_t{0}, soft_count);
    std::for_each(std::execution::par_unseq, index_range.begin(), index_range.end(), [&](const uint32_t i)
    {
        const uint32_t unified_idx = rigid_count + i;
        auto& [min_x, min_y, max_x, max_y] = world_space_aabbs[unified_idx];
        const auto center = Vec2{(max_x - min_x) * 0.5f, (max_y - min_y) * 0.5f};
        items[unified_idx] = {unified_idx, GetMortonCode(center, scene_aabb.min_x, scene_aabb.min_y, inv_scene_aabb_width, inv_scene_aabb_height)};
    });
    
    // Sort by morton code.
    std::ranges::sort(items, {}, &Item::morton_code);
    
    world.bvh_tree_root_index_ = BuildBvhRecursive(items, world_space_aabbs, world.bvh_tree_nodes_, 0, items.size());
}

void CollisionBroadPhase::BuildSubBvh(std::span<const Aabb> world_space_aabbs, std::pmr::vector<BvhNode>& sub_bvh_nodes)
{
    sub_bvh_nodes.clear();
    sub_bvh_nodes.reserve(world_space_aabbs.size() * 2);
    
    // Get Scene Bounding Box (boundary of all bodies in the world).
    Aabb scene_aabb = world_space_aabbs[0];
    for (const auto& world_aabb : world_space_aabbs) {
        scene_aabb = Aabb::Combine(scene_aabb, world_aabb);
    }
    
    const float inv_scene_aabb_width = 1 / (scene_aabb.max_x - scene_aabb.min_x);
    const float inv_scene_aabb_height = 1 / (scene_aabb.max_y - scene_aabb.min_y);
    
    // Get Morton Code and create sort item.
    // std::pmr::vector<Item> items(world_space_aabbs.size(), frame_arena_);
    thread_local  std::vector<Item> items;
    items.resize(world_space_aabbs.size());
    
    auto index_range = std::views::iota(uint16_t{0}, static_cast<uint16_t>(world_space_aabbs.size()));
    std::for_each(std::execution::seq, index_range.begin(), index_range.end(), [&](const auto i)
    {
        auto& [min_x, min_y, max_x, max_y] = world_space_aabbs[i];
        const auto center = Vec2{(max_x - min_x) * 0.5f, (max_y - min_y) * 0.5f};
        items[i] = {i, GetMortonCode(center, scene_aabb.min_x, scene_aabb.min_y, inv_scene_aabb_width, inv_scene_aabb_height)};
    });
    
    // Sort by morton code.
    std::ranges::sort(items, {}, &Item::morton_code);
    
    BuildBvhRecursive(items, world_space_aabbs, sub_bvh_nodes, 0, items.size());
}

void CollisionBroadPhase::QueryBvh(PhysicsWorld& world, const std::span<Aabb> world_aabbs, uint32_t query_obj_idx, std::pmr::vector<std::pair<uint32_t,
    uint32_t>>& rigid_soft_contacts, std::pmr::vector<std::pair<uint32_t, uint32_t>>& soft_soft_contacts)
{
    if (world.bvh_tree_root_index_ == std::numeric_limits<uint32_t>::max()) return;
    
    const uint32_t rigid_count = world.positions_.size();
    const bool is_query_rigid = (query_obj_idx < rigid_count);
    const Aabb query_aabb = world_aabbs[query_obj_idx];
    
    uint32_t stack[64];
    uint32_t stack_ptr = 0;
    stack[stack_ptr++] = world.bvh_tree_root_index_;

    while (stack_ptr > 0) {
        const uint32_t curr = stack[--stack_ptr];
        const BvhNode& node = world.bvh_tree_nodes_[curr];
        
        if (!Aabb::Overlap(node.aabb, query_aabb))  continue;
        
        if (node.IsLeaf()) 
        {
            // Rigid vs Rigid
            if (const uint32_t leaf_id = node.leaf_id; leaf_id < rigid_count)
            {
                if (!is_query_rigid || leaf_id <= query_obj_idx) continue; // Prevent "self-collision" and "duplicate pair (a, b) and (b, a)".
            
                const uint64_t pair_key = MakePairKey(query_obj_idx, leaf_id);

                // Find if the manifold already exist. 
                if (auto it = world.manifold_map_.find(pair_key); it != world.manifold_map_.end()) 
                {
                    const size_t manifold_idx = it->second;
                    world.manifolds_[manifold_idx].is_overlapping = true;
                } 
                else 
                {
                    const size_t new_idx = world.manifolds_.size();
                    world.manifolds_.emplace_back(query_obj_idx, leaf_id);
                    world.manifold_map_[pair_key] = new_idx;
                }
            }
            else
            {
                const uint32_t soft_idx = leaf_id - rigid_count;
                
                // Rigid vs Soft (Particle)
                if (is_query_rigid)
                {
                    const auto& soft_handle = world.soft_body_handles_[soft_idx];
                    for (uint32_t i = 0; i < soft_handle.surface_particle_count; ++i)
                    {
                        const auto particle_idx = soft_handle.start_surface_particle_idx + i; // Only detect surface particle of soft body
                        rigid_soft_contacts.emplace_back(query_obj_idx, particle_idx);
                    }
                }
                // Soft vs Soft
                else
                {
                    if (const uint32_t query_soft_idx = query_obj_idx - rigid_count; query_soft_idx < soft_idx) {
                        soft_soft_contacts.emplace_back(query_soft_idx, soft_idx); // a's particles to b's edges
                        soft_soft_contacts.emplace_back(soft_idx, query_soft_idx); // b's particles to a's edges
                    }
                }
            }
        } 
        else
        {
            stack[stack_ptr++] = node.left_child;
            stack[stack_ptr++] = node.right_child;
        }
    }
}

Aabb CollisionBroadPhase::ComputeFastAabbFromLocalAabb(const Vec2& pos, Rot rot, const Aabb& local_aabb)
{
    const Vec2 local_aabb_min{local_aabb.min_x, local_aabb.min_y};
    const Vec2 local_aabb_max{local_aabb.max_x, local_aabb.max_y};
    
    // Calculate the center and extent of local aabb.
    const Vec2 local_center = (local_aabb_min + local_aabb_max) * 0.5f;
    const Vec2 local_extent = (local_aabb_max - local_aabb_min) * 0.5f;

    // Transform center to world space.
    const Vec2 world_center = rot.Mul(local_center) + pos;

    const float abs_c = std::abs(rot.c);
    const float abs_s = std::abs(rot.s);

    // Calculate extent of AABB of rotated Local AABB(OBB)
    const Vec2 world_extent{
        abs_c * local_extent.x + abs_s * local_extent.y,
        abs_s * local_extent.x + abs_c * local_extent.y
    };

    // World AABB
    const Vec2 world_aabb_min = world_center - world_extent;
    const Vec2 world_aabb_max = world_center + world_extent;
    return Aabb{.min_x = world_aabb_min.x, .min_y = world_aabb_min.y, .max_x = world_aabb_max.x, .max_y = world_aabb_max.y };
}

void CollisionBroadPhase::CleanupStaleManifolds(PhysicsWorld& world)
{
    for (size_t i = 0; i < world.manifolds_.size(); ) 
    {
        if (auto& manifold = world.manifolds_[i]; !manifold.is_overlapping) 
        {
            const uint64_t key_to_remove = MakePairKey(manifold.a, manifold.b);

            if (const size_t last_idx = world.manifolds_.size() - 1; i != last_idx) 
            {
                // Swap
                const auto& last_manifold = world.manifolds_[last_idx];
                const uint64_t last_key = MakePairKey(last_manifold.a, last_manifold.b);

                world.manifolds_[i] = last_manifold;
                world.manifold_map_[last_key] = i; // Update index of the moved manifold in the Hash Map.
            }

            // Pop
            world.manifolds_.pop_back();
            world.manifold_map_.erase(key_to_remove);

            // Don't increment 'i' here because the manifold that was just moved in needs to be checked in the next loop.
        } 
        else 
        {
            // Reset is_overlapping for next frame.
            manifold.is_overlapping = false;
            ++i;
        }
    }
}
