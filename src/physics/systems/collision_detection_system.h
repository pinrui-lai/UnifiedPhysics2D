#pragma once

#include <span>

#include "types.h"

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
    static std::array<Vec2, 4> GetVertices(const PhysicsWorld& world, const BoxShape& box_shape, uint32_t i);
    static std::span<const Vec2> GetVertices(PhysicsWorld& world, const PolygonShape& polygon_shape, uint32_t i);
    
    // (A X B) X C = B(A dot C) - A(B dot C)
    static Vec2 TripleProduct(const Vec2& a, const Vec2& b, const Vec2& c) {return b * a.Dot(c) - a * b.Dot(c); }
    
    static void BuildBvh(PhysicsWorld& world, std::pmr::vector<Aabb>& world_aabbs);
    static void QueryBvh(PhysicsWorld& world, const std::pmr::vector<Aabb>& world_aabbs, uint32_t query_obj_idx, std::pmr::vector<std::pair<uint32_t, uint32_t>>& rigid_soft_contacts, std::pmr::vector<std::pair<uint32_t, uint32_t>>& soft_soft_contacts);
    static Aabb ComputeFastAabbFromLocalAabb(const Vec2& pos, float rot, const Aabb& local_aabb);
    static uint32_t ExpandBits(uint32_t bits);
    static uint32_t GetMortonCode(const Vec2& pos, float min_x, float min_y, float max_x, float max_y);
    static uint32_t BuildBvhRecursive(PhysicsWorld& world, std::pmr::vector<Item>& items, const std::pmr::vector<Aabb>& world_aabbs, int start, int end);
    static void CheckRigidParticleCollision(PhysicsWorld& world, uint32_t query_obj_idx, uint32_t particle_idx);
    static void CheckParticleEdgeCollision(PhysicsWorld& world, float total_thickness, const SoftBodyHandle& soft_body_a,
                                           const SoftBodyHandle& soft_body_b);
    static void CheckSoftSoftCollision(PhysicsWorld& world, uint32_t soft_idx1, uint32_t soft_idx2);
    static float SdfCircle(const Vec2 p, const float r) { return p.Length() - r; }
    static float SdfBox(Vec2 p, Vec2 b);
    static Vec2 GetParticleBoxCollisionNormal(Vec2 p, Vec2 b);
};
