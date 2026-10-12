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
    static void SolveRigidParticleCollisionConstraints(PhysicsWorld& world, float sub_dt, float inv_sub_dt);
    static void SolveParticleEdgeCollisionConstraints(PhysicsWorld& world);
    static void SolveRigidParticleJointConstraints(PhysicsWorld& world, float inv_sub_dt);
    static void SolveRigidParticleAngularConstraints(PhysicsWorld& world, float inv_sub_dt);
    static void FinalizeParticles(PhysicsWorld& world, float inv_sdt);
    
    friend PhysicsWorld;
};
