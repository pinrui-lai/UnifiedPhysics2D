#pragma once
#include <algorithm>
#include <memory_resource>
#include <vector>
#include <span>

#include "types.h"

class CollisionNarrowPhase;
class CollisionDetectionSystem;
struct Rot;
class PhysicsWorld;

class CollisionBroadPhase
{
private:
    static void BuildWorldBvh(PhysicsWorld& world, std::pmr::vector<Aabb>& world_space_aabbs);
    static void BuildSubBvh(std::span<const Aabb> world_space_aabbs, std::pmr::vector<BvhNode>& sub_bvh_nodes);
    
    template <typename T, typename Alloc>
    static uint32_t BuildBvhRecursive(std::span<Item> items, std::span<const Aabb> world_space_aabbs, std::vector<T, Alloc>& bvh_nodes, int start, int end)
    {
        // Index of current node.
        const uint32_t node_index = bvh_nodes.size();
    
        // Place a placeholder in bvh_tree_nodes_.
        bvh_nodes.emplace_back();

        const int count = end - start;
    
        // Leaf node.
        if (count == 1)
        {
            bvh_nodes[node_index].aabb = world_space_aabbs[items[start].id];
            bvh_nodes[node_index].leaf_id = items[start].id;
            return node_index;
        }
    
        // Find the median and divide in half.
        const int mid = start + count / 2;
    
        const uint32_t left_node_index = BuildBvhRecursive(items, world_space_aabbs, bvh_nodes, start, mid);
        const uint32_t right_node_index = BuildBvhRecursive(items, world_space_aabbs, bvh_nodes, mid, end);
    
        bvh_nodes[node_index].aabb = Aabb::Combine(bvh_nodes[left_node_index].aabb, bvh_nodes[right_node_index].aabb);
        bvh_nodes[node_index].left_child = left_node_index;
        bvh_nodes[node_index].right_child = right_node_index;
    
        return node_index;
    }
    
    static void QueryBvh(PhysicsWorld& world, const std::pmr::vector<Aabb>& world_aabbs, uint32_t query_obj_idx, std::pmr::vector<std::pair<uint32_t, uint32_t>>& rigid_soft_contacts, std::pmr::vector<std::pair<uint32_t, uint32_t>>& soft_soft_contacts);
    static Aabb ComputeFastAabbFromLocalAabb(const Vec2& pos, Rot rot, const Aabb& local_aabb);
    
    static uint32_t ExpandBits(uint32_t bits){
        // 00000000 00000000 11111111 11111111 -> 00000000 11111111 00000000 11111111 -> 00001111 00001111 00001111 00001111 ...
        bits = (bits | (bits << 8)) & 0x00FF00FF;
        bits = (bits | (bits << 4)) & 0x0F0F0F0F;
        bits = (bits | (bits << 2)) & 0x33333333;
        bits = (bits | (bits << 1)) & 0x55555555;
        return bits;
    }
    
    static uint32_t GetMortonCode(const Vec2& pos, const float min_x, const float min_y, const float inv_scene_aabb_width, const float inv_scene_aabb_height)
    {
        // Clamp to avoid floating-point precision issues.
        const float norm_x = std::clamp((pos.x - min_x) * inv_scene_aabb_width, 0.0f, 1.0f);
        const float norm_y = std::clamp((pos.y - min_y) * inv_scene_aabb_height, 0.0f, 1.0f);

        const uint32_t x = static_cast<uint32_t>(norm_x * 65535.f);
        const uint32_t y = static_cast<uint32_t>(norm_y * 65535.f);
    
        return (ExpandBits(y) << 1) | ExpandBits(x); 
    }
    
    static uint64_t MakePairKey(const uint32_t a, const uint32_t b) {
        const uint32_t id1 = std::min(a, b);
        const uint32_t id2 = std::max(a, b);
        return (static_cast<uint64_t>(id1) << 32) | id2;
    }
    
    static void CleanupStaleManifolds(PhysicsWorld& world);
    
    friend CollisionDetectionSystem;
    friend CollisionNarrowPhase;
};
