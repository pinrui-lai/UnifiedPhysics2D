#pragma once

struct Manifold;
class PhysicsWorld;

class CollisionSolverSystem
{
private:
    static void InitializeContacts(PhysicsWorld& world);
    static void WarmStartContacts(PhysicsWorld& world);
    static void SolveVelocityConstraints(PhysicsWorld& world);
    static void SolvePositionConstraints(PhysicsWorld& world);
    
    friend PhysicsWorld;
};
