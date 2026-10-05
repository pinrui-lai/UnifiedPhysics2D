#include "collision_broad_phase.h"

#include "physics_world.h"

void CollisionBroadPhase::BuildBvh(PhysicsWorld& world, std::pmr::vector<Aabb>& world_aabbs)
{
    const uint32_t rigid_count = world.positions_.size();
    const uint32_t soft_count = world.soft_body_handles_.size();
    
    world.bvh_tree_nodes_.clear();
    
    // Rigid Body world AABBs.
    for (auto i = 0; i < world.local_aabbs_.size(); ++i)
    {
        std::visit([&](auto& s)
        {
            using S = std::decay_t<decltype(s)>;
            
            if constexpr (std::is_same_v<S, CircleShape>)
            {
                const Vec2 pos = world.positions_[i].val;
                auto [local_min_x, local_min_y, local_max_x, local_max_y] = world.local_aabbs_[i];
                world_aabbs.emplace_back(local_min_x + pos.x, local_min_y + pos.y,
                    local_max_x + pos.x, local_max_y + pos.y);
            }
            else
            {
                world_aabbs.push_back(ComputeFastAabbFromLocalAabb(world.positions_[i].val, world.rots_[i], world.local_aabbs_[i]));
            }
            
        }, world.shapes_[i]);
    }
    
    // XPBD Soft Body world AABBs.
    for (uint32_t i = 0; i < soft_count; ++i)
    {
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

        world_aabbs.emplace_back(min_x - physics_config::kTwoParticlesThickness, min_y - physics_config::kTwoParticlesThickness,
            max_x + physics_config::kTwoParticlesThickness, max_y + physics_config::kTwoParticlesThickness);
    }
    
    // Get Scene Bounding Box (boundary of all bodies in the world).
    Aabb scene_aabb = world_aabbs[0];
    for (const auto& world_aabb : world_aabbs) {
        scene_aabb = Aabb::Combine(scene_aabb, world_aabb);
    }
    const float inv_scene_aabb_width = 1 / (scene_aabb.max_x - scene_aabb.min_x);
    const float inv_scene_aabb_height = 1 / (scene_aabb.max_y - scene_aabb.min_y);
    
    // Get Morton Code and create sort item.
    std::pmr::vector<Item> items(&world.frame_arena_);
    items.reserve(world_aabbs.size());
    
    for (uint32_t i = 0; i < rigid_count; ++i)
    {
        items.emplace_back(i, GetMortonCode(world.positions_[i].val, scene_aabb.min_x, scene_aabb.min_y, inv_scene_aabb_width, inv_scene_aabb_height));
    }
    for (uint32_t i = 0; i < soft_count; ++i)
    {
        const uint32_t unified_idx = rigid_count + i;
        auto& [min_x, min_y, max_x, max_y] = world_aabbs[unified_idx];
        auto center = Vec2{(max_x - min_x) * 0.5f, (max_y - min_y) * 0.5f};
        items.emplace_back(unified_idx, GetMortonCode(center, scene_aabb.min_x, scene_aabb.min_y, inv_scene_aabb_width, inv_scene_aabb_height));
    }
    
    // Sort by morton code.
    std::ranges::sort(items, {}, &Item::morton_code);
    
    world.bvh_tree_root_index_ = BuildBvhRecursive(world, items, world_aabbs, 0, items.size());
}

uint32_t CollisionBroadPhase::BuildBvhRecursive(PhysicsWorld& world, std::pmr::vector<Item>& items,
    const std::pmr::vector<Aabb>& world_aabbs, int start, int end)
{
    // Index of current node.
    const uint32_t node_index = world.bvh_tree_nodes_.size();
    
    // Place a placeholder in bvh_tree_nodes_.
    world.bvh_tree_nodes_.emplace_back();

    const int count = end - start;
    
    // Leaf node.
    if (count == 1)
    {
        world.bvh_tree_nodes_[node_index].aabb = world_aabbs[items[start].id];
        world.bvh_tree_nodes_[node_index].leaf_id = items[start].id;
        return node_index;
    }
    
    // Find the median and divide in half.
    const int mid = start + count / 2;
    
    const uint32_t left_node_index = BuildBvhRecursive(world, items, world_aabbs, start, mid);
    const uint32_t right_node_index = BuildBvhRecursive(world, items, world_aabbs, mid, end);
    
    world.bvh_tree_nodes_[node_index].aabb = Aabb::Combine(world.bvh_tree_nodes_[left_node_index].aabb, world.bvh_tree_nodes_[right_node_index].aabb);
    world.bvh_tree_nodes_[node_index].left_child = left_node_index;
    world.bvh_tree_nodes_[node_index].right_child = right_node_index;
    
    return node_index;
}

void CollisionBroadPhase::QueryBvh(PhysicsWorld& world, const std::pmr::vector<Aabb>& world_aabbs,
    uint32_t query_obj_idx, std::pmr::vector<std::pair<uint32_t, uint32_t>>& rigid_soft_contacts,
    std::pmr::vector<std::pair<uint32_t, uint32_t>>& soft_soft_contacts)
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
                    for (uint32_t i = soft_handle.start_surface_particle_idx; i < soft_handle.start_surface_particle_idx + soft_handle.surface_particle_count; ++i)
                    {
                        const auto particle_idx = world.surface_particles_idx_[i]; // Only detect surface particle of soft body
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
