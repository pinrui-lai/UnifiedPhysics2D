#pragma once

class PhysicsWorld;

class CollisionDetectionSystem
{
private:
    static void CheckCollision(PhysicsWorld& world);
    
    friend PhysicsWorld;
};
