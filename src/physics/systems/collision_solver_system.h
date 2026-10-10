#pragma once

struct Manifold;
class PhysicsWorld;

class CollisionSolverSystem
{
public:
    static void SolveCollision(PhysicsWorld& world);
    
private:
    static void InitializeContact(const PhysicsWorld& world, Manifold& manifold, uint32_t contact_idx);
    static void WarmStartContact(PhysicsWorld& world, const Manifold& manifold, uint32_t contact_idx);
    static void SolveVelocityConstraint(PhysicsWorld& world, Manifold& manifold, uint32_t contact_idx);
    static void SolvePositionConstraint(PhysicsWorld& world, Manifold& manifold, uint32_t contact_idx);
    
    friend PhysicsWorld;
};
