#pragma once
#include <span>

struct Aabb;
class PhysicsWorld;

class CollisionDetectionSystem
{
private:
    static void GenerateContacts( PhysicsWorld& world, std::span<const Aabb> world_aabbs, std::pmr::vector<std::pair<uint32_t, uint32_t>>& rigid_rigid_candidates,
                                  std::pmr::vector<std::pair<uint32_t, uint32_t>>& rigid_soft_contacts, std::pmr::vector<std::pair<uint32_t, uint32_t>>& soft_soft_contacts);
    
    static void CheckCollision(PhysicsWorld& world);
    
    friend PhysicsWorld;
};
