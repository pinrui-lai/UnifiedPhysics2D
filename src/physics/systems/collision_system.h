#pragma once

#include <optional>
#include <span>

#include "types.h"

struct PolygonShape;
struct BoxShape;
class PhysicsWorld;

class CollisionSystem
{
public:
    static void SolveCollision(PhysicsWorld& world);
    
private:
    static std::array<std::optional<Contact>, 2> Gjk(PhysicsWorld& world, Manifold& manifold);
    static bool HandleSimplex(FixedSimplex& simplex, Vec2& dir);
    static Vec2 GetSupport(const PhysicsWorld& world, size_t i, Vec2 dir);
    static std::pair<Vec2, float> Epa(const PhysicsWorld& world, const FixedSimplex& simplex, Manifold& manifold);
    static std::pair<Contact, std::optional<Contact>> GetCollisionPoints(PhysicsWorld& world, Vec2 collision_normal, float depth, Manifold& manifold);
    static void GetMostPerpEdge(const std::span<const Vec2>& vertices, Vec2 dir, std::array<Vec2, 2>& out_edges, float& out_dot);
    static void ClipEdge(Vec2 clip_face_inner_normal, Vec2 point_on_clip_face, std::array<Vec2, 2>& out_clipped_edges);
    static std::array<Vec2, 4> GetVertices(const PhysicsWorld& world, const BoxShape& box_shape, size_t i);
    static std::span<const Vec2> GetVertices(PhysicsWorld& world, const PolygonShape& polygon_shape, size_t i);
    
    
    // (A X B) X C = B(A dot C) - A(B dot C)
    static Vec2 TripleProduct(const Vec2& a, const Vec2& b, const Vec2& c) {return b * a.Dot(c) - a * b.Dot(c); }
    
    static void PreSolve(const PhysicsWorld& world, Manifold& manifold, size_t contact_idx);
    static void SolveVelocityConstraint(PhysicsWorld& world, Manifold& manifold, size_t contact_idx);
    static void SolvePositionConstraint(PhysicsWorld& world, Manifold& manifold, size_t contact_idx);
};
