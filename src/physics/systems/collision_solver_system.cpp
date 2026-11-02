#include <algorithm>

#include "physics/physics_world.h"
#include "physics/systems/collision_solver_system.h"

void CollisionSolverSystem::InitializeContacts(PhysicsWorld& world)
{
    for (auto& manifold : world.manifolds_)
    {
        for (int i = 0; i < manifold.contact_count; ++i)
        {
            Contact& contact = manifold.contacts[i];
            contact.pseudo_impulse_sum = 0.f;

            // Effective Mass on normal: K = 1/mA + 1/mB + (rA x n)^2 / IA + (rB x n)^2 / IB
            const Vec2 ra = contact.collision_point_a - world.positions_[manifold.a];
            const Vec2 rb = contact.collision_point_b - world.positions_[manifold.b];

            const RigidBodyComponent& rigid_body_a = world.rigid_bodies_[manifold.a];
            const RigidBodyComponent& rigid_body_b = world.rigid_bodies_[manifold.b];

            const float ra_n = ra.Cross(manifold.normal);
            const float rb_n = rb.Cross(manifold.normal);
            const float k_normal = rigid_body_a.inv_mass + rigid_body_b.inv_mass
                                    + rigid_body_a.inv_inertia * ra_n * ra_n
                                    + rigid_body_b.inv_inertia * rb_n * rb_n;
            contact.mass_normal = (k_normal > 0.f)? 1.f / k_normal : 0.f;

            // Effective Mass on tangent
            const Vec2 tangent = manifold.normal.PerpCcw();
            const float ra_t = ra.Cross(tangent);
            const float rb_t = rb.Cross(tangent);
            const float k_tangent = rigid_body_a.inv_mass + rigid_body_b.inv_mass
                                    + rigid_body_a.inv_inertia * ra_t * ra_t
                                    + rigid_body_b.inv_inertia * rb_t * rb_t;
            contact.mass_tangent = (k_tangent > 0.f)? 1.f / k_tangent : 0.f;

            // Bias for Solving Penetration of Split Impulse
            constexpr float beta = 0.2f;
            constexpr float slop = 0.01f; // Slight penetration is permissible to prevent shaking.
            const float penetration = std::max(0.f, contact.depth - slop);
            contact.bias = beta * penetration;

            // Target restitution velocity.
            const Vec2 velocity_a = world.velocities_[manifold.a] + Vec2::ZCross(world.angular_velocities_[manifold.a], ra);
            const Vec2 velocity_b = world.velocities_[manifold.b] + Vec2::ZCross(world.angular_velocities_[manifold.b], rb);
            const Vec2 rel_velocity = velocity_b - velocity_a;
            const float rel_vel_normal = rel_velocity.Dot(manifold.normal);

            const float restitution = std::min(rigid_body_a.restitution, rigid_body_b.restitution);
            contact.restitution_velocity = 0.f;
            if (rel_vel_normal < -0.1f)
            {
                contact.restitution_velocity = -restitution * rel_vel_normal;
            }
        }
    }
    
}

void CollisionSolverSystem::WarmStartContacts(PhysicsWorld& world)
{
    for (auto& manifold : world.manifolds_)
    {
        for (int i = 0; i < manifold.contact_count; ++i)
        {
            const Contact& contact = manifold.contacts[i];
            const Vec2 tangent = manifold.normal.PerpCcw();
    
            const Vec2 ra = contact.collision_point_a - world.positions_[manifold.a];
            const Vec2 rb = contact.collision_point_b - world.positions_[manifold.b];
    
            const RigidBodyComponent& rigid_body_a = world.rigid_bodies_[manifold.a];
            const RigidBodyComponent& rigid_body_b = world.rigid_bodies_[manifold.b];
    
            // Warm Starting
            const Vec2 impulse = manifold.normal * contact.normal_impulse_sum + tangent * contact.tangent_impulse_sum;
            if (rigid_body_a.inv_mass > 0.f)
            {
                world.velocities_[manifold.a] -= impulse * rigid_body_a.inv_mass;
                world.angular_velocities_[manifold.a] -= ra.Cross(impulse) * rigid_body_a.inv_inertia;
            }
            if (rigid_body_b.inv_mass > 0.f)
            {
                world.velocities_[manifold.b] += impulse * rigid_body_b.inv_mass;
                world.angular_velocities_[manifold.b] += rb.Cross(impulse) * rigid_body_b.inv_inertia;
            }
        }
    }
    
}

void CollisionSolverSystem::SolveVelocityConstraints(PhysicsWorld& world)
{
    for (auto& manifold : world.manifolds_)
    {
        for (int i = 0; i < manifold.contact_count; ++i)
        {
            Contact& contact = manifold.contacts[i];
            
            const RigidBodyComponent& rigid_body_a = world.rigid_bodies_[manifold.a];
            const RigidBodyComponent& rigid_body_b = world.rigid_bodies_[manifold.b];

            const Vec2 ra = contact.collision_point_a - world.positions_[manifold.a];
            const Vec2 rb = contact.collision_point_b - world.positions_[manifold.b];
            
            const Vec2 collision_point_vel_a = world.velocities_[manifold.a] + Vec2::ZCross(world.angular_velocities_[manifold.a], ra);
            const Vec2 collision_point_vel_b = world.velocities_[manifold.b] + Vec2::ZCross(world.angular_velocities_[manifold.b], rb);
            const Vec2 rel_vel = collision_point_vel_b - collision_point_vel_a;
            const float rel_vel_normal = rel_vel.Dot(manifold.normal);
            
            float delta_impulse_magnitude = contact.mass_normal * (contact.restitution_velocity - rel_vel_normal);
            
            // Clamp impulse magnitude to make sure only push
            const float old_impulse_magnitude = contact.normal_impulse_sum;
            contact.normal_impulse_sum = std::max(old_impulse_magnitude + delta_impulse_magnitude, 0.f);
            delta_impulse_magnitude = contact.normal_impulse_sum - old_impulse_magnitude;
            
            // Apply impulse, Jacobian Matrix of Penetration Constrain = [-(transpose n), -CrossProduct(ra, normal), transpose n, CrossProduct(ra, normal)] 
            if (rigid_body_a.inv_mass > 0.f)
            {
                world.velocities_[manifold.a] -= manifold.normal * delta_impulse_magnitude * rigid_body_a.inv_mass;
                world.angular_velocities_[manifold.a] -= ra.Cross(manifold.normal) * delta_impulse_magnitude * rigid_body_a.inv_inertia;
            }
            
            if (rigid_body_b.inv_mass > 0.f)
            {
                world.velocities_[manifold.b] += manifold.normal * delta_impulse_magnitude * rigid_body_b.inv_mass;
                world.angular_velocities_[manifold.b] += rb.Cross(manifold.normal) * delta_impulse_magnitude * rigid_body_b.inv_inertia;
            }
            
            // ============ Tangent Friction ================
            const Vec2 updated_vel_a = world.velocities_[manifold.a] + Vec2::ZCross(world.angular_velocities_[manifold.a], ra);
            const Vec2 updated_vel_b = world.velocities_[manifold.b] + Vec2::ZCross(world.angular_velocities_[manifold.b], rb);
            const Vec2 updated_rel_vel = updated_vel_b - updated_vel_a;
            
            const Vec2 tangent = manifold.normal.PerpCcw();
            const float rel_vel_tan = updated_rel_vel.Dot(tangent);
            
            float delta_impulse_magnitude_tan = contact.mass_tangent * -rel_vel_tan;
            
            // Coulomb's law of friction : |jt| <= mu * jn
            const float mu = std::sqrt(rigid_body_a.friction * rigid_body_b.friction);
            const float max_friction = mu * contact.normal_impulse_sum;
            const float old_impulse_tangent = contact.tangent_impulse_sum;
            
            contact.tangent_impulse_sum = std::clamp(old_impulse_tangent + delta_impulse_magnitude_tan, -max_friction, max_friction);
            delta_impulse_magnitude_tan = contact.tangent_impulse_sum - old_impulse_tangent;
            
            if (rigid_body_a.inv_mass > 0.f)
            {
                world.velocities_[manifold.a] -= tangent * delta_impulse_magnitude_tan * rigid_body_a.inv_mass;
                world.angular_velocities_[manifold.a] -= ra.Cross(tangent) * delta_impulse_magnitude_tan * rigid_body_a.inv_inertia;
            }
            
            if (rigid_body_b.inv_mass > 0.f)
            {
                world.velocities_[manifold.b] += tangent * delta_impulse_magnitude_tan * rigid_body_b.inv_mass;
                world.angular_velocities_[manifold.b] += rb.Cross(tangent) * delta_impulse_magnitude_tan * rigid_body_b.inv_inertia;
            }
        }
    }
    
    
}

void CollisionSolverSystem::SolvePositionConstraints(PhysicsWorld& world)
{
    for (auto& manifold : world.manifolds_)
    {
        for (int i = 0; i < manifold.contact_count; ++i)
        {
            Contact& contact = manifold.contacts[i];
    
            const RigidBodyComponent& rigid_body_a = world.rigid_bodies_[manifold.a];
            const RigidBodyComponent& rigid_body_b = world.rigid_bodies_[manifold.b];
            
            if (rigid_body_a.inv_mass == 0.f && rigid_body_b.inv_mass == 0.f) return;

            const Vec2 ra = contact.collision_point_a - world.positions_[manifold.a];
            const Vec2 rb = contact.collision_point_b - world.positions_[manifold.b];
            
            // Use pseudo velocities compute relative velocity
            const Vec2 collision_point_pseudo_vel_a = world.pseudo_velocities_[manifold.a] + Vec2::ZCross(world.pseudo_angular_velocities_[manifold.a], ra);
            const Vec2 collision_point_pseudo_vel_b = world.pseudo_velocities_[manifold.b] + Vec2::ZCross(world.pseudo_angular_velocities_[manifold.b], rb);

            const float rel_pseudo_vel_normal = (collision_point_pseudo_vel_b - collision_point_pseudo_vel_a).Dot(manifold.normal);
            
            float delta_pseudo_impulse_magnitude = contact.mass_normal * (-rel_pseudo_vel_normal + contact.bias);
            
            // Clamp impulse magnitude to make sure only push
            const float old_pseudo_impulse_magnitude = contact.pseudo_impulse_sum;
            contact.pseudo_impulse_sum = std::max(old_pseudo_impulse_magnitude + delta_pseudo_impulse_magnitude, 0.f);
            delta_pseudo_impulse_magnitude = contact.pseudo_impulse_sum - old_pseudo_impulse_magnitude;
            
            // Apply impulse, Jacobian Matrix of Penetration Constrain = [-(transpose n), -CrossProduct(ra, normal), transpose n, CrossProduct(ra, normal)] 
            if (rigid_body_a.inv_mass > 0.f)
            {
                world.pseudo_velocities_[manifold.a] -= manifold.normal * delta_pseudo_impulse_magnitude * rigid_body_a.inv_mass;
                world.pseudo_angular_velocities_[manifold.a] -= ra.Cross(manifold.normal) * delta_pseudo_impulse_magnitude * rigid_body_a.inv_inertia;
            }
            
            if (rigid_body_b.inv_mass > 0.f)
            {
                world.pseudo_velocities_[manifold.b] += manifold.normal * delta_pseudo_impulse_magnitude * rigid_body_b.inv_mass;
                world.pseudo_angular_velocities_[manifold.b] += rb.Cross(manifold.normal) * delta_pseudo_impulse_magnitude * rigid_body_b.inv_inertia;
            }
        }
    }
    
}
