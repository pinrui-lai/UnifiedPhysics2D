#pragma once

#include <algorithm>
#include <span>

#include "types.h"

struct Rot;
struct PolygonShape;
struct BoxShape;
class PhysicsWorld;

class CollisionDetectionSystem
{
public:
    static void CheckCollision(PhysicsWorld& world);
    
private:
    static void Gjk(PhysicsWorld& world, Manifold& manifold);
    static bool HandleSimplex(FixedSimplex& simplex, Vec2& dir);
    static Vec2 GetSupport(const PhysicsWorld& world, uint32_t i, Vec2 dir);
    static std::pair<Vec2, float> Epa(const PhysicsWorld& world, const FixedSimplex& simplex, const Manifold& manifold);
    static void GetCollisionPoints(PhysicsWorld& world, Vec2 collision_normal, float depth, Manifold& manifold);
    static void GetMostPerpEdge(const std::span<const Vec2>& vertices, Vec2 dir, std::array<Vec2, 2>& out_edge_pos, float& out_dot, std::array<uint32_t, 2>& out_edge_ids);
    static void ClipEdge(Vec2 clip_face_inner_normal, Vec2 point_on_clip_face, uint32_t clip_face_ids, std::array<Vec2, 2>& out_clipped_edges, std::array<int, 2>& out_clip_face_ids);
    
    // (A X B) X C = B(A dot C) - A(B dot C)
    static Vec2 TripleProduct(const Vec2& a, const Vec2& b, const Vec2& c) {return b * a.Dot(c) - a * b.Dot(c); }
    
    static void BuildBvh(PhysicsWorld& world, std::pmr::vector<Aabb>& world_aabbs);
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
    
    static uint32_t BuildBvhRecursive(PhysicsWorld& world, std::pmr::vector<Item>& items, const std::pmr::vector<Aabb>& world_aabbs, int start, int end);
    
    static uint64_t MakePairKey(const uint32_t a, const uint32_t b) {
        const uint32_t id1 = std::min(a, b);
        const uint32_t id2 = std::max(a, b);
        return (static_cast<uint64_t>(id1) << 32) | id2;
    }
    
    static void CleanupStaleManifolds(PhysicsWorld& world);
    
    static void CheckRigidParticleCollision(PhysicsWorld& world, uint32_t query_obj_idx, uint32_t particle_idx);
    static uint32_t CountParticleEdgeCollisions(const PhysicsWorld& world, float total_thickness, const SoftBodyHandle& soft_body_a, const SoftBodyHandle& soft_body_b);
    static void CheckParticleEdgeCollision(const PhysicsWorld& world, float total_thickness, const SoftBodyHandle& soft_body_a,
                                           const SoftBodyHandle& soft_body_b, XpbdParticleEdgeCollisionConstraint* out_contacts);
    static void CheckSoftSoftCollision(PhysicsWorld& world, const std::pmr::vector<std::pair<uint32_t, uint32_t>>& pairs, float total_thickness);
    static float SdfCircle(const Vec2 p, const float r) { return p.Length() - r; }
    static float SdfBox(Vec2 p, Vec2 b);
    static Vec2 GetParticleBoxCollisionNormal(Vec2 p, Vec2 b);
};
