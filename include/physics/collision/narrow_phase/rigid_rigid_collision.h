#pragma once
#include <span>

#include "physics/types.h"

class CollisionDetectionSystem;
class PhysicsWorld;

class RigidRigidCollision
{
private:
    // (A X B) X C = B(A dot C) - A(B dot C)
    static Vec2 TripleProduct(const Vec2& a, const Vec2& b, const Vec2& c) {return b * a.Dot(c) - a * b.Dot(c); }
    
    static Vec2 GetSupport(const PhysicsWorld& world, uint32_t i, Vec2 dir);
    static bool HandleSimplex(FixedSimplex& simplex, Vec2& dir);
    static void Gjk(PhysicsWorld& world, Manifold& manifold);
    static std::pair<Vec2, float> Epa(const PhysicsWorld& world, const FixedSimplex& simplex, const Manifold& manifold);
    static void GetMostPerpEdge(const std::span<const Vec2>& vertices, Vec2 dir, std::array<Vec2, 2>& out_edge_pos, float& out_dot, std::array<uint32_t, 2>& out_edge_ids);
    static void ClipEdge(Vec2 clip_face_inner_normal, Vec2 point_on_clip_face, uint32_t clip_face_id, std::array<Vec2, 2>& out_clipped_edges, std::array<int, 2>& out_clip_face_ids);
    static void GetCollisionPoints(PhysicsWorld& world, Vec2 collision_normal, float depth, Manifold& manifold);
    
    friend CollisionDetectionSystem;
};
