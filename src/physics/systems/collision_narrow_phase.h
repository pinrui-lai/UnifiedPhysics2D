#pragma once
#include <span>

#include "types.h"

class PhysicsWorld;
class CollisionDetectionSystem;

class CollisionNarrowPhase
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
    static float SdfCircle(const Vec2 p, const float r) { return p.Length() - r; }
    
    static float SdfBox(const Vec2 p, const Vec2 b)
    {
        const Vec2 d = { std::abs(p.x) - b.x, std::abs(p.y) - b.y }; // Map the point to the first quadrant.

        // External distance (point outside the rectangle corner or boundary)
        const Vec2 outside_vec = { std::max(d.x, 0.0f), std::max(d.y, 0.0f) };
        const float outside_dist = outside_vec.Length();

        // Interior distance (negative value if the point is inside the rectangle)
        const float inside_dist = std::min(std::max(d.x, d.y), 0.0f);

        return outside_dist + inside_dist;
    }
    
    static Vec2 GetParticleBoxCollisionNormal(Vec2 p, Vec2 b)
    {
        const Vec2 d = { std::abs(p.x) - b.x, std::abs(p.y) - b.y };
    
        // Inside the box, the shortest separation direction is perpendicular to the nearest edge.
        Vec2 n = (d.x > d.y) ? Vec2{1.0f, 0.0f} : Vec2{0.0f, 1.0f};
    
        // Map to the quadrant where point p is located.
        if (p.x < 0.0f) n.x = -n.x;
        if (p.y < 0.0f) n.y = -n.y;

        return n;
    }
    
    static void CheckRigidParticleCollision(PhysicsWorld& world, uint32_t query_obj_idx, uint32_t particle_idx);
    static uint32_t CountParticleEdgeCollisions(const PhysicsWorld& world, const SoftBodyHandle& soft_body_a, const SoftBodyHandle& soft_body_b);
    static uint32_t CheckParticleEdgeCollision(const PhysicsWorld& world, const SoftBodyHandle& soft_body_a, const SoftBodyHandle& soft_body_b, XpbdParticleEdgeCollisionConstraint* out_contacts);
    static void CheckSoftSoftCollision(PhysicsWorld& world, const std::pmr::vector<std::pair<uint32_t, uint32_t>>& pairs);
    
    friend CollisionDetectionSystem;
};
