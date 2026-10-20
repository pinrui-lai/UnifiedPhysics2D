#pragma once
#include "physics/types.h"

class CollisionDetectionSystem;
class PhysicsWorld;

class RigidSoftCollision
{
private:
    static float SdfCircle(const Vec2 p, const float r) { return p.Length() - r; }
    
    static float SdfBox(const Vec2 p, const Vec2 b)
    {
        const Vec2 d = { std::abs(p.x) - b.x, std::abs(p.y) - b.y }; // Map the point to the first quadrant.

        // External distance (point outside the rectangle corner or boundary)
        const Vec2 outside_vec = { std::max(d.x, 0.f), std::max(d.y, 0.f) };
        const float outside_dist = outside_vec.Length();

        // Interior distance (negative value if the point is inside the rectangle)
        const float inside_dist = std::min(std::max(d.x, d.y), 0.f);

        return outside_dist + inside_dist;
    }
    
    static Vec2 GetParticleBoxCollisionNormal(Vec2 p, Vec2 b)
    {
        const Vec2 d = { std::abs(p.x) - b.x, std::abs(p.y) - b.y };
    
        // Inside the box, the shortest separation direction is perpendicular to the nearest edge.
        Vec2 n = (d.x > d.y) ? Vec2{1.f, 0.f} : Vec2{0.f, 1.f};
    
        // Map to the quadrant where point p is located.
        if (p.x < 0.f) n.x = -n.x;
        if (p.y < 0.f) n.y = -n.y;

        return n;
    }
    
    static void CheckRigidParticleCollision(PhysicsWorld& world, uint32_t query_obj_idx, uint32_t particle_idx);
    
    friend CollisionDetectionSystem;
};
