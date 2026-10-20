#pragma once
#include <span>
#include <vector>

struct Aabb;
struct BvhNode;
struct SoftBodyHandle;
struct XpbdParticleEdgeCollisionConstraint;
class CollisionDetectionSystem;
class PhysicsWorld;

class SoftSoftCollision
{
private:
    static void UpdateParticleEdgeAabbs(PhysicsWorld& world, const std::pmr::vector<std::pair<uint32_t, uint32_t>>& soft_soft_contacts);
    
    static void CheckParticleEdgeCollision(const PhysicsWorld& world, const SoftBodyHandle& soft_body_a, const SoftBodyHandle& soft_body_b,
        std::span<const Aabb> body_b_edge_aabbs, std::span<const Aabb> body_a_particle_aabbs,
        std::span<BvhNode> body_a_surface_particle_bvh, std::vector<XpbdParticleEdgeCollisionConstraint>& out_contacts);
    
    static void CheckSoftSoftCollision(PhysicsWorld& world, const std::pmr::vector<std::pair<uint32_t, uint32_t>>& pairs);
    
    friend CollisionDetectionSystem;
};
