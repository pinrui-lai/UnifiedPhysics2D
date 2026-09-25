#include "collision_detection_system.h"

#include <algorithm>

#include "physics_world.h"

void CollisionDetectionSystem::CheckCollision(PhysicsWorld& world)
{
    world.debug_incident_edges_.clear();
    world.debug_reference_edges_.clear();
        
    // Mark all manifold's is_overlapping false.
    for (auto& manifold : world.manifolds_)
    {
        manifold.is_overlapping = false;
    }
    
    // Broad-Phase.
    world.bvh_tree_.Build(world);
    const size_t count = world.positions_.size();
    for (size_t i = 0; i < count; ++i)
    {
        world.bvh_tree_.Query(world, i);
    }
    
    // Remove manifold whose AABBs no longer overlap.
    std::erase_if(world.manifolds_, [](const Manifold& manifold)
    {
        return manifold.is_overlapping == false;
    });
    
    // Narrow-Phase.
    for (auto& manifold : world.manifolds_)
    {
        if (world.rigid_bodies_[manifold.a].mass == 0.0f && world.rigid_bodies_[manifold.b].mass == 0.0f) continue;
        
        // Store old contacts and contact_count from the previous frame.
        std::array<Contact, 2> old_contacts = manifold.contacts;
        const size_t old_contact_count = manifold.contact_count;
        
        // Write new contacts of the current frame in manifold.
        Gjk(world, manifold);
        
        // Match Feature ID and inherit old impulse.
        for (size_t i = 0; i < manifold.contact_count; ++i)
        {
            Contact& new_c = manifold.contacts[i];
        
            for (size_t j = 0; j < old_contact_count; ++j)
            {
                const Contact& old_c = old_contacts[j];
                // ID matching successful, inheriting the old impulse.
                if (new_c.id.key == old_c.id.key)
                {
                    new_c.normal_impulse_sum = old_c.normal_impulse_sum;
                    new_c.tangent_impulse_sum = old_c.tangent_impulse_sum;
                    break;
                }
            }
        }
    }
}


void CollisionDetectionSystem::Gjk(PhysicsWorld& world, Manifold& manifold)
{
    const size_t index_a = manifold.a;
    const size_t index_b = manifold.b;
    
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
            manifold.contact_count = 0;
            for (auto& contact : manifold.contacts)
            {
                contact.normal_impulse_sum = 0;
                contact.tangent_impulse_sum = 0;
            }
            return;
        }
        
        simplex.push_back(new_minkowski_support);
        
        // A collision has been confirmed.
        if (HandleSimplex(simplex, dir))
        {
            manifold.contact_count = 1;
            auto [collision_normal, depth] = Epa(world, simplex, manifold);
            GetCollisionPoints(world, collision_normal, depth, manifold);
            return;
        }
    }
}

bool CollisionDetectionSystem::HandleSimplex(FixedSimplex& simplex, Vec2& dir)
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

Vec2 CollisionDetectionSystem::GetSupport(const PhysicsWorld& world, const size_t i, const Vec2 dir)
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

std::pair<Vec2, float> CollisionDetectionSystem::Epa(const PhysicsWorld& world, const FixedSimplex& simplex, const Manifold& manifold)
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
        Vec2 new_point = GetSupport(world, manifold.a, min_normal) - GetSupport(world, manifold.b, -min_normal);
        
        // It is already the boundary of the complete Minkowski diff.
        const float new_point_distance = new_point.Dot(min_normal);
        if (constexpr float tolerance = 0.00001f; new_point_distance - min_distance < tolerance || polytope.size() >= polytope.capacity())
        {
            return {min_normal, min_distance};
        }
        
        // Not yet expanded to the boundary.
        polytope.insert(min_insert_index, new_point);
    }
}

void CollisionDetectionSystem::GetCollisionPoints(PhysicsWorld& world, Vec2 collision_normal, const float depth, Manifold& manifold)
{
    size_t index_a = manifold.a;
    size_t index_b = manifold.b;
    
    Shape shape_a = world.shapes_[index_a];
    Shape shape_b = world.shapes_[index_b];

    manifold.normal = collision_normal;
    Contact& contact1 = manifold.contacts[0];
    contact1.depth = depth;
    contact1.normal_impulse_sum = 0.f;
    contact1.tangent_impulse_sum = 0.f;
    
    contact1.id.key = 0;
    
    std::visit([&](const auto& a, const auto& b)
    {
        using A = std::decay_t<decltype(a)>;
        using B = std::decay_t<decltype(b)>;
        
        if constexpr (std::is_same_v<A, CircleShape> && std::is_same_v<B, CircleShape>)
        {
            contact1.collision_point_a = world.positions_[index_a].val + collision_normal * a.radius;
            contact1.collision_point_b = world.positions_[index_b].val - collision_normal * b.radius;
            contact1.id.key = 0;
        }
        else if constexpr (std::is_same_v<A, CircleShape>)
        {
            contact1.collision_point_a = world.positions_[index_a].val + collision_normal * a.radius;
            contact1.collision_point_b = contact1.collision_point_a - collision_normal * depth;
            
            contact1.id.features.type_a = 2;
            contact1.id.features.index_a = 0;
            
            std::array<Vec2, 2> edge_pos_b;
            std::array<size_t, 2> edge_ids_b;
            float edge_b_dot_normal;
            GetMostPerpEdge(GetVertices(world, b, index_b), -collision_normal, edge_pos_b, edge_b_dot_normal, edge_ids_b);
            
            contact1.id.features.type_b = 1;
            contact1.id.features.index_b = edge_ids_b[0];
        }
        else if constexpr (std::is_same_v<B, CircleShape>)
        {
            contact1.collision_point_b = world.positions_[index_b].val - collision_normal * b.radius;
            contact1.collision_point_a = contact1.collision_point_b + collision_normal * depth;
            
            contact1.id.features.type_b = 2;
            contact1.id.features.index_b = 0;
            
            std::array<Vec2, 2> edge_pos_a;
            std::array<size_t, 2> edge_ids_a;
            float edge_a_dot_normal;
            GetMostPerpEdge(GetVertices(world, a, index_a), collision_normal, edge_pos_a, edge_a_dot_normal, edge_ids_a);
            
            contact1.id.features.type_a = 1;
            contact1.id.features.index_a = edge_ids_a[0];
        }
        else
        {
            // Decide incident edge and reference edge.
            std::array<Vec2, 2> edge_pos_a;
            std::array<size_t, 2> edge_ids_a;
            float edge_a_dot_normal;
            GetMostPerpEdge(GetVertices(world, a, index_a), collision_normal, edge_pos_a, edge_a_dot_normal, edge_ids_a);
            
            std::array<Vec2, 2> edge_pos_b;
            std::array<size_t, 2> edge_ids_b;
            float edge_b_dot_normal;
            GetMostPerpEdge(GetVertices(world, b, index_b), -collision_normal, edge_pos_b, edge_b_dot_normal, edge_ids_b);
            
            const bool is_ref_a = edge_a_dot_normal > edge_b_dot_normal;
            const std::array<Vec2, 2> ref_edge_pos = edge_a_dot_normal > edge_b_dot_normal ? edge_pos_a : edge_pos_b;
            const std::array<Vec2, 2> inc_edge_pos = edge_a_dot_normal > edge_b_dot_normal ? edge_pos_b : edge_pos_a;
            const std::array<size_t, 2> ref_edge_ids = edge_a_dot_normal > edge_b_dot_normal ? edge_ids_a : edge_ids_b;
            const std::array<size_t, 2> inc_edge_ids = edge_a_dot_normal > edge_b_dot_normal ? edge_ids_b : edge_ids_a;
            
            if (world.show_debug_clipping_)
            {
                world.debug_incident_edges_.push_back(inc_edge_pos);
                world.debug_reference_edges_.push_back(ref_edge_pos);
            }
            
            // Clip incident edge.
            std::array<Vec2, 2> clipped_incident_edge = inc_edge_pos;
            std::array<int, 2> clipping_ref_vertex_ids{-1, -1}; // -1 : Not cut
            const Vec2 ref_a_to_b = ref_edge_pos[1] - ref_edge_pos[0]; // normal of clip-face (ref edge).
            ClipEdge(ref_a_to_b, ref_edge_pos[0], ref_edge_ids[0], clipped_incident_edge, clipping_ref_vertex_ids);
            
            const Vec2 ref_b_to_a = ref_edge_pos[0] - ref_edge_pos[1];
            ClipEdge(ref_b_to_a, ref_edge_pos[1], ref_edge_ids[1], clipped_incident_edge, clipping_ref_vertex_ids);

            const Vec2 ref_a_to_b_normal = ref_a_to_b.Normal();
            
            // Keep point which penetrates reference edge.
            const float clipped_inc_a_dot_normal = (clipped_incident_edge[0] - ref_edge_pos[0]).Dot(ref_a_to_b_normal);
            const float clipped_inc_b_dot_normal = (clipped_incident_edge[1] - ref_edge_pos[0]).Dot(ref_a_to_b_normal);
            
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
            
            auto assign_feature_id = [&](Contact& target_contact, const size_t inc_edge_ids_idx) {
                uint8_t& inc_index = is_ref_a ? target_contact.id.features.index_b : contact1.id.features.index_a;
                uint8_t& inc_type = is_ref_a ? target_contact.id.features.type_b : contact1.id.features.type_a;
                uint8_t& ref_index = is_ref_a ? target_contact.id.features.index_a : contact1.id.features.index_b;
                uint8_t& ref_type = is_ref_a ? contact1.id.features.type_a : contact1.id.features.type_b;
                
                // Incident edge Not cut : Incident edge Got cut
                inc_index = clipping_ref_vertex_ids[inc_edge_ids_idx] == -1 ? inc_edge_ids[inc_edge_ids_idx] : inc_edge_ids[0];
                inc_type = clipping_ref_vertex_ids[inc_edge_ids_idx] == -1 ? 0 : 1; // 0: Vertex, 1: Edge
                ref_index = clipping_ref_vertex_ids[inc_edge_ids_idx] == -1 ? ref_edge_ids[0] : clipping_ref_vertex_ids[inc_edge_ids_idx];
                ref_type = clipping_ref_vertex_ids[inc_edge_ids_idx] == -1 ? 1 : 0;
            };
            
            assign_collision_points(contact1, clipped_incident_edge[primary_idx]);
            assign_feature_id(contact1, primary_idx);

            // If both points are penetrated, create and push the second contact.
            if (both_penetrating)
            {
                manifold.contact_count = 2;
                Contact& contact2 = manifold.contacts[1];
                contact2.normal_impulse_sum = 0.f;
                contact2.tangent_impulse_sum = 0.f;
                
                // If penetrating reference edge, `clipped point dot normal` is negative.
                contact2.depth   = is_a_deeper ? -clipped_inc_b_dot_normal : -clipped_inc_a_dot_normal;
                
                assign_collision_points(contact2, clipped_incident_edge[secondary_idx]);
                assign_feature_id(contact2, secondary_idx);
            }
        }
    }
    , shape_a, shape_b);
}

void CollisionDetectionSystem::GetMostPerpEdge(const std::span<const Vec2>& vertices, Vec2 dir, std::array<Vec2, 2>& out_edge_pos,
    float& out_dot, std::array<size_t, 2>& out_edge_ids)
{
    float max_dot = std::numeric_limits<float>::lowest();
    std::array<Vec2, 2> max_edge_pos;
    std::array<size_t, 2> max_edge_ids;
    for (size_t i = 0; i < vertices.size(); ++i)
    {
        const size_t j = (i + 1) % vertices.size();
        Vec2 ij = vertices[j] - vertices[i];
        Vec2 ij_normal = ij.Normal();
        if (const float current_dot = ij_normal.Dot(dir); current_dot > max_dot)
        {
            max_dot = current_dot;
            max_edge_pos = {vertices[i], vertices[j]};
            max_edge_ids = {i, j};
        }
    }
    
    out_edge_pos = max_edge_pos;
    out_dot = max_dot;
    out_edge_ids = max_edge_ids;
}

void CollisionDetectionSystem::ClipEdge(const Vec2 clip_face_inner_normal, const Vec2 point_on_clip_face,
    const size_t clip_face_id, std::array<Vec2, 2>& out_clipped_edges, std::array<int, 2>& out_clip_face_ids)
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
        
        // Reference edge's vertex id on the clip-face.(which endpoint on the reference edge clip the incident edge)
        out_clip_face_ids[0] = clip_face_id; 
    }
    if (inc_b_dot_ref < 0)
    {
        // Clip point b (index 1)
        const float clip_ratio = -inc_b_dot_ref / (-inc_b_dot_ref + inc_a_dot_ref);    
        out_clipped_edges[1] = out_clipped_edges[1] - inc_a_to_b * clip_ratio;
        out_clip_face_ids[1] = clip_face_id;
    }
}

std::array<Vec2, 4> CollisionDetectionSystem::GetVertices(const PhysicsWorld& world, const BoxShape& box_shape, size_t i)
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

std::span<const Vec2> CollisionDetectionSystem::GetVertices(PhysicsWorld& world, const PolygonShape& polygon_shape,
    size_t i)
{
    std::span<Vec2> local_vertices = std::span(world.vertices_).subspan(polygon_shape.vertex_offset, polygon_shape.vertex_count);
    
    // TODO : transform to world space.
    
    return local_vertices;
}
