#pragma once
#include "types.h"

class PhysicsWorld;

class XpbdSolver
{
private:
    static void PredictParticles(PhysicsWorld& world, float sdt);
    static void SolveDistanceConstraint(PhysicsWorld& world, float inv_sdt_sq, XpbdDistanceConstraint& c);
    static void SolveTriangleAreaConstraint(PhysicsWorld& world, float inv_sdt_sq, XpbdTriangleAreaConstraint& c);
    static void SolveGlobalAreaConstraints(PhysicsWorld& world, float inv_sdt_sq);
    static void SolveCollisionConstraints(PhysicsWorld& world, float inv_sdt_sq, float sub_dt);
    static void FinalizeParticles(PhysicsWorld& world, float inv_sdt);
    
    friend PhysicsWorld;
};
