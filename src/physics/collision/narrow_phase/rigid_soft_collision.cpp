#include "physics/collision/narrow_phase/rigid_soft_collision.h"
#include "physics/components.h"
#include "physics/physics_world.h"

namespace 
{
    template<class... Ts> struct Overload : Ts... { using Ts::operator()...; };    
}
void RigidSoftCollision::CheckRigidParticleCollision(PhysicsWorld& world, uint32_t query_obj_idx, uint32_t particle_idx)
{
    Shape& shape = world.shapes_[query_obj_idx];
    std::visit(Overload{
        [&](const BoxShape& b)
        {
            const float s = world.rots_[query_obj_idx].s;
            const float c = world.rots_[query_obj_idx].c;
         
            Vec2 p_pos_map_to_box = world.particle_positions_[particle_idx] - world.positions_[query_obj_idx].val;
            p_pos_map_to_box = {p_pos_map_to_box.x * c + p_pos_map_to_box.y * s , - p_pos_map_to_box.x * s + p_pos_map_to_box.y * c};
            const float depth = SdfBox(p_pos_map_to_box, {b.half_width, b.half_height});
            if (depth >= 0.f) return;
            
            const Vec2 local_normal = GetParticleBoxCollisionNormal(p_pos_map_to_box, {b.half_width, b.half_height});
            
            Vec2 contact_normal{local_normal.x * c - local_normal.y * s, local_normal.x * s + local_normal.y * c};
            Vec2 contact_point = world.particle_positions_[particle_idx] + contact_normal * -depth;
   
            world.rigid_particle_collision_constraints_.emplace_back(particle_idx, query_obj_idx, contact_normal, contact_point, -depth);
        },
        [&](const CircleShape& c)
        {
            const Vec2 p_pos_map_to_circle = world.particle_positions_[particle_idx] - world.positions_[query_obj_idx].val;
            const float depth = SdfCircle(p_pos_map_to_circle, c.radius);
            if (depth >= 0.f) return;
            
            Vec2 contact_normal = p_pos_map_to_circle.Normalized() ;
            Vec2 contact_point = world.particle_positions_[particle_idx] + contact_normal * -depth;
        
            world.rigid_particle_collision_constraints_.emplace_back(particle_idx, query_obj_idx, contact_normal, contact_point, -depth);
        },
        [&](const PolygonShape& p){},
    }, shape);
}
