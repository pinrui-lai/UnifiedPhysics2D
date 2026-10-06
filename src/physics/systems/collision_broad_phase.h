#pragma once
#include <algorithm>
#include <span>
#include <vector>

#include "types.h"

class CollisionDetectionSystem;
struct Rot;
class PhysicsWorld;

class CollisionBroadPhase
{
private:
    static void BuildWorldBvh(PhysicsWorld& world, std::pmr::vector<Aabb>& world_space_aabbs);
    static uint32_t BuildBvhRecursive(PhysicsWorld& world, std::pmr::vector<Item>& items, const std::pmr::vector<Aabb>& world_space_aabbs, std::vector<BvhNode>& bvh_nodes, int start, int end);
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
};
