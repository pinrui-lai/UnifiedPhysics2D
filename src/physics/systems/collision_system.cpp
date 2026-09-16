#include <algorithm>
#include <variant>

#include "collision_system.h"
#include "physics_world.h"

void CollisionSystem::SolveCollision(PhysicsWorld& world)
{
    world.debug_contacts_.clear();
    world.debug_incident_edges_.clear();
    world.debug_reference_edges_.clear();
    
    world.frame_arena_.release();
    
    // Use buffered memory.
    std::pmr::vector<Contact> contacts(&world.frame_arena_);
    contacts.reserve(1000);
    
    const size_t count = world.positions_.size();
    // Check Collision.
    for (size_t i = 0; i < count - 1; ++i)
    {
        for (size_t j = i + 1; j < count; ++j)
        {
            if (world.rigid_bodies_[i].mass == 0.0f && world.rigid_bodies_[j].mass == 0.0f) continue;
            
            std::array<std::optional<Contact>, 2> out_contacts = GJK(world, i, j);
            for (const auto& contact : out_contacts)
            {
                if (contact.has_value())
                {
                    if (world.show_debug_contact_points_)
                    {
                        world.debug_contacts_.push_back(*contact);    
                    }
                    contacts.push_back(*contact);
                }
            }
        }
    }
    
    // Solve Collision.
    for (auto& contact : contacts)
    {
        PreSolve(world, contact);
        for (int i = 0; i < 20; ++i)
        {
            SolveVelocityConstraint(world, contact);
            SolvePositionConstraint(world, contact);
        }
    }
}

std::array<std::optional<Contact>, 2> CollisionSystem::GJK(PhysicsWorld& world, size_t index_a, size_t index_b)
{
    const Vec2 position_a = world.positions_[index_a].val;
    const Vec2 position_b = world.positions_[index_b].val;
    Vec2 dir = (position_b - position_a).Normalized();
    
    // First support point of simplex.
    Vec2 minkowski_support = GetSupport(world, index_a, dir) - GetSupport(world, index_b, -dir);
    
    FixedSimplex simplex;
    simplex.push_back(minkowski_support);
    
    dir = -minkowski_support.Normalized(); // Next direction to search is 'minkowski_support point to origin'.
    
    while (true)
    {
        Vec2 new_minkowski_support = GetSupport(world, index_a, dir) - GetSupport(world, index_b, -dir);
        
        // The new_minkowski_support does not exceed the origin in the dir direction.
        // Dot(Origin point to new_minkowski_support, dir)
        if (new_minkowski_support.Dot(dir) < 0)
        {
            return {std::nullopt, std::nullopt};
        }
        
        simplex.push_back(new_minkowski_support);
        
        if (HandleSimplex(simplex, dir))
        {
            auto [collision_normal, depth] = EPA(world, simplex, index_a, index_b);
            const auto [fst, snd] = GetCollisionPoints(world, collision_normal, depth, index_a, index_b);  
            return {fst, snd};
        }
    }
}

bool CollisionSystem::HandleSimplex(FixedSimplex& simplex, Vec2& dir)
{
    const Vec2 a = simplex.back(); // Latest point in simplex.
    const Vec2 ao = -a;
    
    if (simplex.size() == 2)
    {
        const Vec2 ab = simplex[0] - a; // b - a
        dir = TripleProduct(ab, ao, ab);
        
        if (dir.LengthSquared() < 0.00001f)
        {
            // If the origin is on AB, use AB's normal as dir.
            dir = ab.NormalWithoutNormalized();
        }
    }
    else if (simplex.size() == 3)
    {
        const Vec2 ab = simplex[1] - a;
        const Vec2 ac = simplex[0] - a;

        const Vec2 ab_normal = TripleProduct(ac, ab, ab); // Get normal vector of AB pointing outside the triangle.
        const Vec2 ac_normal = TripleProduct(ab, ac, ac); // Get normal vector of AC pointing outside the triangle.
        
        if (ab_normal.Dot(ao) > 0)
        {
            // Origin is outside AB. 
            simplex.erase_at(0); // Erase C.
            dir = ab_normal; // The next direction to search.
        }
        else if (ac_normal.Dot(ao) > 0)
        {
            // Origin is outside AC.
            simplex.erase_at(1);
            dir = ac_normal;
        }
        else
        {
            // Origin is inside the triangle.
            return true;
        }
    }
    
    return false;
}

namespace 
{
    template<class... Ts> struct Overload : Ts... { using Ts::operator()...; };    
}

Vec2 CollisionSystem::GetSupport(const PhysicsWorld& world, const size_t i, const Vec2 dir)
{
    const Shape& s = world.shapes_[i];
    return std::visit(Overload{
        [&](const CircleShape& c){ return world.positions_[i].val + dir.Normalized() * c.radius; },
            [&](const BoxShape& b)
                {
                    const float sin_rotation = std::sin(world.rotations_[i].val);
                    const float cos_rotation = std::cos(world.rotations_[i].val);
                
                    // Rotate dir to box's local space (rotate `-world.rotations_[i]` )
                    // cos(-theta) == cos(theta)   sin(-theta) == -sin(theta)
                    const Vec2 local_dir{dir.x * cos_rotation + dir.y * sin_rotation, -dir.x * sin_rotation + dir.y * cos_rotation};
                
                   // Box's local vertices are half_width/half_height with sign.
                   // In local space, the vertex has same sign with local_dir is support point.
                   // std::copysign use Bitwise Operation copy local_d's sign.
                   const Vec2 local_support{std::copysign(b.half_width, local_dir.x)
                       , std::copysign(b.half_height, local_dir.y)};
                    
                   // Transform local_support to world coordinate (rotate + translate).
                   return Vec2{local_support.x * cos_rotation - local_support.y * sin_rotation + world.positions_[i].val.x
                       , local_support.x * sin_rotation + local_support.y * cos_rotation + world.positions_[i].val.y};
                },
        [&](const PolygonShape& p){ return Vec2{0.f, 0.f}; },
    }, s);
}

std::pair<Vec2, float> CollisionSystem::EPA(const PhysicsWorld& world, const FixedSimplex& simplex, const size_t index_a, const size_t index_b)
{
    FixedPolytope polytope;
    for (size_t i = 0; i < simplex.size(); ++i)
    {
        polytope.push_back(simplex[i]);
    }
    
    // Ensure the vertices of the polygon are arranged in a clockwise order.
    const Vec2 e0 = polytope[1] - polytope[0];
    const Vec2 e1 = polytope[2] - polytope[1];
    if (e0.Cross(e1) > 0) 
    {
        std::swap(polytope[0], polytope[1]);
    }
    
    while (true)
    {
        float min_distance = std::numeric_limits<float>::max();
        Vec2 min_normal;
        size_t min_insert_index = 0;
        
        // Find edge nearest to origin.
        for (size_t j = 0; j < polytope.size(); ++j)
        {
            const size_t k = (j + 1) % polytope.size();
            Vec2 vertex_a = polytope[j];
            Vec2 vertex_b = polytope[k];
            
            Vec2 edge = vertex_b - vertex_a;
            Vec2 normal = edge.Normal();
            
            float distance = vertex_a.Dot(normal);
            if (distance < 0)
            {
                distance = 0.f;
            }
            
            if (distance < min_distance)
            {
                min_distance = distance;
                min_normal = normal;
                min_insert_index = k;
            }
        }
        
        // Find new Minkowski Support Point.
        Vec2 new_point = GetSupport(world, index_a, min_normal) - GetSupport(world, index_b, -min_normal);
        
        // It is already the boundary of the complete Minkowski simplex.
        const float new_point_distance = new_point.Dot(min_normal);
        if (constexpr float tolerance = 0.00001f; new_point_distance - min_distance < tolerance || polytope.size() >= polytope.capacity())
        {
            return {min_normal, min_distance};
        }
        
        // Not yet expanded to the boundary.
        polytope.insert(min_insert_index, new_point);
    }
}

std::pair<Contact, std::optional<Contact>> CollisionSystem::GetCollisionPoints(PhysicsWorld& world, Vec2 collision_normal, const float depth, size_t index_a, size_t index_b)
{
    Shape shape_a = world.shapes_[index_a];
    Shape shape_b = world.shapes_[index_b];

    Contact contact1;
    contact1.depth = depth;
    contact1.normal = collision_normal;
    contact1.a = index_a;
    contact1.b = index_b;
    
    std::optional<Contact> contact2 = std::nullopt;
    
    std::visit([&](const auto& a, const auto& b)
    {
        using A = std::decay_t<decltype(a)>;
        using B = std::decay_t<decltype(b)>;
        
        if constexpr (std::is_same_v<A, CircleShape> && std::is_same_v<B, CircleShape>)
        {
            contact1.collision_point_a = world.positions_[index_a].val + collision_normal * a.radius;
            contact1.collision_point_b = world.positions_[index_b].val - collision_normal * b.radius;
        }
        else if constexpr (std::is_same_v<A, CircleShape>)
        {
            contact1.collision_point_a = world.positions_[index_a].val + collision_normal * a.radius;
            contact1.collision_point_b = contact1.collision_point_a - collision_normal * depth;
        }
        else if constexpr (std::is_same_v<B, CircleShape>)
        {
            contact1.collision_point_b = world.positions_[index_b].val - collision_normal * b.radius;
            contact1.collision_point_a = contact1.collision_point_b + collision_normal * depth;
        }
        else
        {
            // Decide incident edge and reference edge.
            std::array<Vec2, 2> edge_a;
            float edge_a_dot_normal;
            GetMostPerpEdge(GetVertices(world, a, index_a), collision_normal, edge_a, edge_a_dot_normal);
            
            std::array<Vec2, 2> edge_b;
            float edge_b_dot_normal;
            GetMostPerpEdge(GetVertices(world, b, index_b), -collision_normal, edge_b, edge_b_dot_normal);
            
            const bool is_ref_a = edge_a_dot_normal > edge_b_dot_normal;
            const std::array<Vec2, 2> reference_edge = edge_a_dot_normal > edge_b_dot_normal ? edge_a : edge_b;
            const std::array<Vec2, 2> incident_edge = edge_a_dot_normal > edge_b_dot_normal ? edge_b : edge_a;
            
            if (world.show_debug_clipping_)
            {
                world.debug_incident_edges_.push_back(incident_edge);
                world.debug_reference_edges_.push_back(reference_edge);
                
            }
            
            // Clip incident edge.
            std::array<Vec2, 2> clipped_incident_edge = incident_edge;
            
            const Vec2 ref_a_to_b = reference_edge[1] - reference_edge[0]; // normal of clip-face (ref edge).
            ClipEdge(ref_a_to_b, reference_edge[0], clipped_incident_edge);
            
            const Vec2 ref_b_to_a = reference_edge[0] - reference_edge[1];
            ClipEdge(ref_b_to_a, reference_edge[1], clipped_incident_edge);

            const Vec2 ref_a_to_b_normal = ref_a_to_b.Normal();
            
            // Keep point which penetrates reference edge.
            const float clipped_inc_a_dot_normal = (clipped_incident_edge[0] - reference_edge[0]).Dot(ref_a_to_b_normal);
            const float clipped_inc_b_dot_normal = (clipped_incident_edge[1] - reference_edge[0]).Dot(ref_a_to_b_normal);
            
            const bool is_a_deeper = clipped_inc_a_dot_normal < clipped_inc_b_dot_normal;
            const bool both_penetrating = (clipped_inc_a_dot_normal < 0.f) && (clipped_inc_b_dot_normal < 0.f);

            // Decide index of primary contact (contact) and secondary contact (new_contact)
            const int primary_idx   = is_a_deeper ? 0 : 1;
            const int secondary_idx = 1 - primary_idx;

            // According to incident_edge, set collision_point_a and collision_point_b.
            auto assign_collision_points = [&](Contact& target_contact, const auto& point) {
                Vec2& primary_pt   = (!is_ref_a) ? target_contact.collision_point_a : target_contact.collision_point_b;
                Vec2& secondary_pt = (!is_ref_a) ? target_contact.collision_point_b : target_contact.collision_point_a;
                
                primary_pt   = point;
                secondary_pt = point + ref_a_to_b_normal * target_contact.depth;
            };
            
            assign_collision_points(contact1, clipped_incident_edge[primary_idx]);

            // If both points are penetrated, create and push the second contact.
            if (both_penetrating)
            {
                contact2 = contact1;
                // If penetrating reference edge, `clipped point dot normal` is negative.
                contact2.value().depth   = is_a_deeper ? -clipped_inc_b_dot_normal : -clipped_inc_a_dot_normal;
                
                assign_collision_points(contact2.value(), clipped_incident_edge[secondary_idx]);
            }
        }
    }
    , shape_a, shape_b);
    
    return {contact1, contact2};
}

void CollisionSystem::GetMostPerpEdge(const std::span<const Vec2>& vertices, Vec2 dir, std::array<Vec2, 2>& out_edges,
    float& out_dot)
{
    float max_dot = std::numeric_limits<float>::lowest();
    std::array<Vec2, 2> max_edge;
    for (size_t i = 0; i < vertices.size(); ++i)
    {
        const size_t j = (i + 1) % vertices.size();
        Vec2 ij = vertices[j] - vertices[i];
        Vec2 ij_normal = ij.Normal();
        if (const float current_dot = ij_normal.Dot(dir); current_dot > max_dot)
        {
            max_dot = current_dot;
            max_edge = {vertices[i], vertices[j]};
        }
    }
    
    out_edges = max_edge;
    out_dot = max_dot;
}

void CollisionSystem::ClipEdge(const Vec2 clip_face_inner_normal, const Vec2 point_on_clip_face,
    std::array<Vec2, 2>& out_clipped_edges)
{
    // Project Incident Edge's points to 'normal of clip-face (Reference Edge)' (all vector start from one of Reference Edge point).
    const Vec2 ref_a_to_inc_a = out_clipped_edges[0] - point_on_clip_face;
    const Vec2 ref_a_to_inc_b = out_clipped_edges[1] - point_on_clip_face;

    const float inc_a_dot_ref = ref_a_to_inc_a.Dot(clip_face_inner_normal);
    const float inc_b_dot_ref = ref_a_to_inc_b.Dot(clip_face_inner_normal);
            
    // If dot results have one positive and one negative, the negative one is outside clip-face.
    // Clip by dot results ratio.
    const Vec2 inc_a_to_b = out_clipped_edges[1] - out_clipped_edges[0];
    if (inc_a_dot_ref < 0)
    {
        // Clip point a (index 0)
        const float clip_ratio = -inc_a_dot_ref / (-inc_a_dot_ref + inc_b_dot_ref);    
        out_clipped_edges[0] = out_clipped_edges[0] + inc_a_to_b * clip_ratio;
    }
    if (inc_b_dot_ref < 0)
    {
        // Clip point b (index 1)
        const float clip_ratio = -inc_b_dot_ref / (-inc_b_dot_ref + inc_a_dot_ref);    
        out_clipped_edges[1] = out_clipped_edges[1] - inc_a_to_b * clip_ratio;
    }
}

std::array<Vec2, 4> CollisionSystem::GetVertices(const PhysicsWorld& world, const BoxShape& box_shape, size_t i)
{
    // Make sure vertices are arranged in clockwise.
    std::array<Vec2, 4> vertices;
    vertices[0] = {-box_shape.half_width, box_shape.half_height};
    vertices[1] = {box_shape.half_width, box_shape.half_height};
    vertices[2] = {box_shape.half_width, -box_shape.half_height};
    vertices[3] = {-box_shape.half_width, -box_shape.half_height};
    
    std::ranges::transform(vertices, vertices.begin(), [&](const Vec2 v){ return Vec2{v.x * std::cos(world.rotations_[i].val) - v.y * std::sin(world.rotations_[i].val)
        , v.x * std::sin(world.rotations_[i].val) + v.y * std::cos(world.rotations_[i].val)} + world.positions_[i].val; });
    
    return vertices;
}

std::span<const Vec2> CollisionSystem::GetVertices(PhysicsWorld& world, const PolygonShape& polygon_shape,
    size_t i)
{
    std::span<Vec2> local_vertices = std::span(world.vertices_).subspan(polygon_shape.vertex_offset, polygon_shape.vertex_count);
    
    // TODO : transform to world space.
    
    return local_vertices;
}

void CollisionSystem::PreSolve(const PhysicsWorld& world, Contact& contact)
{
    // Effective Mass on normal: K = 1/mA + 1/mB + (rA x n)^2 / IA + (rB x n)^2 / IB
    const Vec2 ra = contact.collision_point_a - world.positions_[contact.a].val;
    const Vec2 rb = contact.collision_point_b - world.positions_[contact.b].val;
    
    const RigidBodyComponent& rigid_body_a = world.rigid_bodies_[contact.a];
    const RigidBodyComponent& rigid_body_b = world.rigid_bodies_[contact.b];
    
    const float ra_n = ra.Cross(contact.normal);
    const float rb_n = rb.Cross(contact.normal);
    const float k_normal = rigid_body_a.inv_mass + rigid_body_b.inv_mass
                            + rigid_body_a.inv_inertia * ra_n * ra_n
                            + rigid_body_b.inv_inertia * rb_n * rb_n;
    
    contact.mass_normal = (k_normal > 0.f)? 1.f / k_normal : 0.f;
    
    // Effective Mass on tangent
    const Vec2 tangent = contact.normal.NormalWithoutNormalized();
    const float ra_t = ra.Cross(tangent);
    const float rb_t = rb.Cross(tangent);
    const float k_tangent = rigid_body_a.inv_mass + rigid_body_b.inv_mass
                            + rigid_body_a.inv_inertia * ra_t * ra_t
                            + rigid_body_b.inv_inertia * rb_t * rb_t;
    contact.mass_tangent = (k_tangent > 0.f)? 1.f / k_tangent : 0.f;
    
    // Bias for Solving Penetration of Split Impulse
    constexpr float beta = 0.2f;
    constexpr float slop = 0.01f; // Slight penetration is permissible to prevent shaking.
    const float penetration = std::max(0.0f, contact.depth - slop);
    contact.bias = beta * penetration;
}

void CollisionSystem::SolveVelocityConstraint(PhysicsWorld& world, Contact& contact)
{
    const RigidBodyComponent& rigid_body_a = world.rigid_bodies_[contact.a];
    const RigidBodyComponent& rigid_body_b = world.rigid_bodies_[contact.b];

    const Vec2 ra = contact.collision_point_a - world.positions_[contact.a].val;
    const Vec2 rb = contact.collision_point_b - world.positions_[contact.b].val;
    
    const Vec2 collision_point_vel_a = world.velocities_[contact.a].val + Vec2::ZCross(world.angular_velocities_[contact.a].val, ra);
    const Vec2 collision_point_vel_b = world.velocities_[contact.b].val + Vec2::ZCross(world.angular_velocities_[contact.b].val, rb);

    const Vec2 rel_vel = collision_point_vel_b - collision_point_vel_a;
    const float rel_vel_normal = rel_vel.Dot(contact.normal);
    if (rel_vel_normal > 0.0f) return;
    
    const float restitution = std::min(rigid_body_a.restitution, rigid_body_b.restitution);
    float restitution_bias = 0.0f;
    if (std::abs(rel_vel_normal) > 0.1f) { // Relative Velocity Threshold
        restitution_bias = -restitution * rel_vel_normal;
    }
    
    float delta_impulse_magnitude = contact.mass_normal * (-rel_vel_normal + restitution_bias);
    
    // Clamp impulse magnitude to make sure only push
    const float old_impulse_magnitude = contact.impulse_sum;
    contact.impulse_sum = std::max(old_impulse_magnitude + delta_impulse_magnitude, 0.0f);
    delta_impulse_magnitude = contact.impulse_sum - old_impulse_magnitude;
    
    // Apply impulse, Jacobian Matrix of Penetration Constrain = [-(transpose n), -CrossProduct(ra, normal), transpose n, CrossProduct(ra, normal)] 
    if (rigid_body_a.mass > 0.f)
    {
        world.velocities_[contact.a].val -= contact.normal * delta_impulse_magnitude * rigid_body_a.inv_mass;
        world.angular_velocities_[contact.a].val -= ra.Cross(contact.normal) * delta_impulse_magnitude * rigid_body_a.inv_inertia;
    }
    
    if (rigid_body_b.mass > 0.f)
    {
        world.velocities_[contact.b].val += contact.normal * delta_impulse_magnitude * rigid_body_b.inv_mass;
        world.angular_velocities_[contact.b].val += rb.Cross(contact.normal) * delta_impulse_magnitude * rigid_body_b.inv_inertia;
    }
    
    // ============ Tangent Friction ================
    const Vec2 updated_vel_a = world.velocities_[contact.a].val + Vec2::ZCross(world.angular_velocities_[contact.a].val, ra);
    const Vec2 updated_vel_b = world.velocities_[contact.b].val + Vec2::ZCross(world.angular_velocities_[contact.b].val, rb);
    const Vec2 updated_rel_vel = updated_vel_b - updated_vel_a;
    
    const Vec2 tangent = contact.normal.NormalWithoutNormalized();
    const float rel_vel_tan = updated_rel_vel.Dot(tangent);
    
    const float mu = std::sqrt(rigid_body_a.friction * rigid_body_b.friction);
    float delta_impulse_magnitude_tan = contact.mass_tangent * -rel_vel_tan;
    
    // Coulomb's law of friction : |jt| <= mu * jn
    const float max_friction = mu * contact.impulse_sum;
    const float old_impulse_tangent = contact.impulse_sum_tan;
    
    contact.impulse_sum_tan = std::clamp(old_impulse_tangent + delta_impulse_magnitude_tan, -max_friction, max_friction);
    delta_impulse_magnitude_tan = contact.impulse_sum_tan - old_impulse_tangent;
    
    if (rigid_body_a.mass > 0.f)
    {
        world.velocities_[contact.a].val -= tangent * delta_impulse_magnitude_tan * rigid_body_a.inv_mass;
        world.angular_velocities_[contact.a].val -= ra.Cross(tangent) * delta_impulse_magnitude_tan * rigid_body_a.inv_inertia;
    }
    
    if (rigid_body_b.mass > 0.f)
    {
        world.velocities_[contact.b].val += tangent * delta_impulse_magnitude_tan * rigid_body_b.inv_mass;
        world.angular_velocities_[contact.b].val += rb.Cross(tangent) * delta_impulse_magnitude_tan * rigid_body_b.inv_inertia;
    }
}

void CollisionSystem::SolvePositionConstraint(PhysicsWorld& world, Contact& contact)
{
    const RigidBodyComponent& rigid_body_a = world.rigid_bodies_[contact.a];
    const RigidBodyComponent& rigid_body_b = world.rigid_bodies_[contact.b];
    
    if (rigid_body_a.mass == 0.0f && rigid_body_b.mass == 0.0f) return;

    const Vec2 ra = contact.collision_point_a - world.positions_[contact.a].val;
    const Vec2 rb = contact.collision_point_b - world.positions_[contact.b].val;
    
    // Use pseudo velocities compute relative velocity
    const Vec2 collision_point_pseudo_vel_a = world.pseudo_velocities_[contact.a].val + Vec2::ZCross(world.pseudo_angular_velocities_[contact.a].val, ra);
    const Vec2 collision_point_pseudo_vel_b = world.pseudo_velocities_[contact.b].val + Vec2::ZCross(world.pseudo_angular_velocities_[contact.b].val, rb);

    const float rel_pseudo_vel_normal = (collision_point_pseudo_vel_b - collision_point_pseudo_vel_a).Dot(contact.normal);
    
    float delta_pseudo_impulse_magnitude = contact.mass_normal * (-rel_pseudo_vel_normal + contact.bias);
    
    // Clamp impulse magnitude to make sure only push
    const float old_pseudo_impulse_magnitude = contact.pseudo_impulse_sum;
    contact.pseudo_impulse_sum = std::max(old_pseudo_impulse_magnitude + delta_pseudo_impulse_magnitude, 0.0f);
    delta_pseudo_impulse_magnitude = contact.pseudo_impulse_sum - old_pseudo_impulse_magnitude;
    
    // Apply impulse, Jacobian Matrix of Penetration Constrain = [-(transpose n), -CrossProduct(ra, normal), transpose n, CrossProduct(ra, normal)] 
    if (rigid_body_a.mass > 0.f)
    {
        world.pseudo_velocities_[contact.a].val -= contact.normal * delta_pseudo_impulse_magnitude * rigid_body_a.inv_mass;
        world.pseudo_angular_velocities_[contact.a].val -= ra.Cross(contact.normal) * delta_pseudo_impulse_magnitude * rigid_body_a.inv_inertia;
    }
    
    if (rigid_body_b.mass > 0.f)
    {
        world.pseudo_velocities_[contact.b].val += contact.normal * delta_pseudo_impulse_magnitude * rigid_body_b.inv_mass;
        world.pseudo_angular_velocities_[contact.b].val += rb.Cross(contact.normal) * delta_pseudo_impulse_magnitude * rigid_body_b.inv_inertia;
    }
}
