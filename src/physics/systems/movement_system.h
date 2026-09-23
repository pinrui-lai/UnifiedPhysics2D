#pragma once

class PhysicsWorld;

class MovementSystem
{
public:
    static void IntegrateVelocity(PhysicsWorld& world, float dt);
    static void IntegratePosition(PhysicsWorld& world, float dt);
    
};
