#pragma once

class PhysicsWorld;

class XpbdSolver
{
private:
    static void PredictParticles(PhysicsWorld& world, float sdt);
    static void SolveDistanceConstraints(PhysicsWorld& world, float inv_sdt_sq);
    static void SolveCollisionConstraints(PhysicsWorld& world, float inv_sdt_sq);
    static void SolveTriangleAreaConstraints(PhysicsWorld& world, float inv_sdt_sq);
    static void SolveGlobalAreaConstraints(PhysicsWorld& world, float inv_sdt_sq);
    static void FinalizeParticles(PhysicsWorld& world, float inv_sdt);
    
    friend PhysicsWorld;
};
