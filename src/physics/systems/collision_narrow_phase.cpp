#include <algorithm>
#include <execution>
#include <ranges>

#include "collision_narrow_phase.h"
#include "physics_world.h"

namespace 
{
    template<class... Ts> struct Overload : Ts... { using Ts::operator()...; };    
}
Vec2 CollisionNarrowPhase::GetSupport(const PhysicsWorld& world, const uint32_t i, const Vec2 dir)
{
    const Shape& s = world.shapes_[i];
    return std::visit(Overload{
        [&](const CircleShape& c){ return world.positions_[i].val + dir.Normalized() * c.radius; },
            [&](const BoxShape& b)
                {
                    // Rotate dir to box's local space (rotate `-world.rotations_[i]` )
                    const Vec2 local_dir{world.rots_[i].MulT(dir)};
                
                   // Box's local vertices are half_width/half_height with sign.
                   // In local space, the vertex has same sign with local_dir is support point.
                   // std::copysign use Bitwise Operation copy local_d's sign.
                   const Vec2 local_support{std::copysign(b.half_width, local_dir.x)
                       , std::copysign(b.half_height, local_dir.y)};
                    
                   // Transform local_support to world coordinate (rotate + translate).
                    return world.rots_[i].Mul(local_support) + world.positions_[i].val;
                },
        [&](const PolygonShape& p){ return Vec2{0.f, 0.f}; },
    }, s);
}

bool CollisionNarrowPhase::HandleSimplex(FixedSimplex& simplex, Vec2& dir)
{
    const Vec2 a = simplex.back(); // Latest point in simplex.
    const Vec2 ao = -a;
    
    if (simplex.size() == 2)
    {
        const Vec2 ab = simplex[0] - a; // b - a
        dir = TripleProduct(ab, ao, ab);
        
        if (dir.LengthSquared() < 1e-6f)
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

void CollisionNarrowPhase::Gjk(PhysicsWorld& world, Manifold& manifold)
{
    constexpr int max_iterations = 32; 
    const uint32_t index_a = manifold.a;
    const uint32_t index_b = manifold.b;
    
    const Vec2 position_a = world.positions_[index_a].val;
    const Vec2 position_b = world.positions_[index_b].val;
    Vec2 dir = (position_b - position_a).Normalized();
    
    // First support point of simplex.
    Vec2 minkowski_support = GetSupport(world, index_a, dir) - GetSupport(world, index_b, -dir);
    
    FixedSimplex simplex;
    simplex.push_back(minkowski_support);
    
    dir = -minkowski_support.Normalized(); // Next direction to search is 'minkowski_support point to origin'.
    
    auto reset_manifold = [&manifold]()
    {
        manifold.contact_count = 0;
        manifold.contacts[0].normal_impulse_sum = 0;
        manifold.contacts[0].tangent_impulse_sum = 0;
        manifold.contacts[1].normal_impulse_sum = 0;
        manifold.contacts[1].tangent_impulse_sum = 0;
    };
    
    for (int i = 0; i < max_iterations; ++i)
    {
        if (dir.LengthSquared() < 1e-6f)
        {
            reset_manifold();
            return;
        }
        
        Vec2 new_minkowski_support = GetSupport(world, index_a, dir) - GetSupport(world, index_b, -dir);
        
        // The new_minkowski_support does not exceed the origin in the dir direction.
        // Dot(Origin point to new_minkowski_support, dir)
        if (new_minkowski_support.Dot(dir) < 0)
        {
            reset_manifold();
            return;
        }
        
        simplex.push_back(new_minkowski_support);
        
        // A collision has been confirmed.
        if (HandleSimplex(simplex, dir))
        {
            manifold.contact_count = 1;
            auto [collision_normal, depth] = Epa(world, simplex, manifold);
            if (depth < 1e-6f){
                reset_manifold();
                return;
            }
            GetCollisionPoints(world, collision_normal, depth, manifold);
            return;
        }
    }
    
    reset_manifold();
}

std::pair<Vec2, float> CollisionNarrowPhase::Epa(const PhysicsWorld& world, const FixedSimplex& simplex,
    const Manifold& manifold)
{
    FixedPolytope polytope;
    for (uint8_t i = 0; i < simplex.size(); ++i)
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
        uint8_t min_insert_index = 0;
        
        // Find edge nearest to origin.
        for (uint8_t j = 0; j < polytope.size(); ++j)
        {
            const uint8_t k = (j + 1) % polytope.size();
            Vec2 vertex_a = polytope[j];
            Vec2 vertex_b = polytope[k];
            
            Vec2 edge = vertex_b - vertex_a;
            Vec2 normal = edge.Normal();
            
            float distance = vertex_a.Dot(normal);
            if (distance < 1e-6f)
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
        if (constexpr float tolerance = 1e-5f; new_point_distance - min_distance < tolerance || polytope.size() >= polytope.capacity())
        {
            return {min_normal, min_distance};
        }
        
        // Not yet expanded to the boundary.
        polytope.insert(min_insert_index, new_point);
    }
}

void CollisionNarrowPhase::GetMostPerpEdge(const std::span<const Vec2>& vertices, const Vec2 dir,
    std::array<Vec2, 2>& out_edge_pos, float& out_dot, std::array<uint32_t, 2>& out_edge_ids)
{
    float max_dot = std::numeric_limits<float>::lowest();
    std::array<Vec2, 2> max_edge_pos;
    std::array<uint32_t, 2> max_edge_ids;
    for (uint32_t i = 0; i < vertices.size(); ++i)
    {
        const uint32_t j = (i + 1) % vertices.size();
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

void CollisionNarrowPhase::ClipEdge(const Vec2 clip_face_inner_normal, const Vec2 point_on_clip_face, const uint32_t clip_face_id,
    std::array<Vec2, 2>& out_clipped_edges, std::array<int, 2>& out_clip_face_ids)
{
    // Project Incident Edge's points to 'normal of clip-face (Reference Edge)' (all vector start from one of Reference Edge point).
    const Vec2 ref_a_to_inc_a = out_clipped_edges[0] - point_on_clip_face;
    const Vec2 ref_a_to_inc_b = out_clipped_edges[1] - point_on_clip_face;

    const float inc_a_dot_ref = ref_a_to_inc_a.Dot(clip_face_inner_normal);
    const float inc_b_dot_ref = ref_a_to_inc_b.Dot(clip_face_inner_normal);
            
    // If dot results have one positive and one negative, the negative one is outside clip-face.
    // Clip by dot results ratio.
    const Vec2 inc_a_to_b = out_clipped_edges[1] - out_clipped_edges[0];
    
    if (inc_a_dot_ref < 0.f)
    {
        // Clip point a (index 0)
        const float clip_ratio = -inc_a_dot_ref / (-inc_a_dot_ref + inc_b_dot_ref);    
        out_clipped_edges[0] = out_clipped_edges[0] + inc_a_to_b * clip_ratio;
        
        // Reference edge's vertex id on the clip-face.(which endpoint on the reference edge clip the incident edge)
        out_clip_face_ids[0] = clip_face_id; 
    }
    if (inc_b_dot_ref < 0.f)
    {
        // Clip point b (index 1)
        const float clip_ratio = -inc_b_dot_ref / (-inc_b_dot_ref + inc_a_dot_ref);    
        out_clipped_edges[1] = out_clipped_edges[1] - inc_a_to_b * clip_ratio;
        out_clip_face_ids[1] = clip_face_id;
    }
}

void CollisionNarrowPhase::GetCollisionPoints(PhysicsWorld& world, Vec2 collision_normal, const float depth,
    Manifold& manifold)
{
    uint32_t index_a = manifold.a;
    uint32_t index_b = manifold.b;
    
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
            std::array<uint32_t, 2> edge_ids_b;
            float edge_b_dot_normal;
            GetMostPerpEdge(PhysicsWorld::GetBodyVertices(world, b, index_b), -collision_normal, edge_pos_b, edge_b_dot_normal, edge_ids_b);
            
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
            std::array<uint32_t, 2> edge_ids_a;
            float edge_a_dot_normal;
            GetMostPerpEdge(PhysicsWorld::GetBodyVertices(world, a, index_a), collision_normal, edge_pos_a, edge_a_dot_normal, edge_ids_a);
            
            contact1.id.features.type_a = 1;
            contact1.id.features.index_a = edge_ids_a[0];
        }
        else
        {
            // Decide incident edge and reference edge.
            std::array<Vec2, 2> edge_pos_a;
            std::array<uint32_t, 2> edge_ids_a;
            float edge_a_dot_normal;
            GetMostPerpEdge(PhysicsWorld::GetBodyVertices(world, a, index_a), collision_normal, edge_pos_a, edge_a_dot_normal, edge_ids_a);
            
            std::array<Vec2, 2> edge_pos_b;
            std::array<uint32_t, 2> edge_ids_b;
            float edge_b_dot_normal;
            GetMostPerpEdge(PhysicsWorld::GetBodyVertices(world, b, index_b), -collision_normal, edge_pos_b, edge_b_dot_normal, edge_ids_b);
            
            const bool is_ref_a = edge_a_dot_normal > edge_b_dot_normal + 1e-6f;
            const std::array<Vec2, 2> ref_edge_pos = is_ref_a ? edge_pos_a : edge_pos_b;
            const std::array<Vec2, 2> inc_edge_pos = is_ref_a ? edge_pos_b : edge_pos_a;
            const std::array<uint32_t, 2> ref_edge_ids = is_ref_a ? edge_ids_a : edge_ids_b;
            const std::array<uint32_t, 2> inc_edge_ids = is_ref_a ? edge_ids_b : edge_ids_a;
            
            if (world.show_debug_clipping_ && !world.is_multi_thread_){
                world.debug_incident_edges_.push_back(inc_edge_pos);
                world.debug_reference_edges_.push_back(ref_edge_pos);
            }
            
            // Clip incident edge.
            std::array<Vec2, 2> clipped_incident_edge = inc_edge_pos;
            std::array<int, 2> clipping_ref_vertex_ids{-1, -1}; // Endpoint id of reference edge, which clips incident edge. -1 : Not cut
            
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
            
            auto assign_feature_id = [&](Contact& target_contact, const uint32_t inc_edge_ids_idx) {
                uint8_t& inc_index = is_ref_a ? target_contact.id.features.index_b : target_contact.id.features.index_a;
                uint8_t& inc_type = is_ref_a ? target_contact.id.features.type_b : target_contact.id.features.type_a;
                uint8_t& ref_index = is_ref_a ? target_contact.id.features.index_a : target_contact.id.features.index_b;
                uint8_t& ref_type = is_ref_a ? target_contact.id.features.type_a : target_contact.id.features.type_b;
                
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

void CollisionNarrowPhase::CheckRigidParticleCollision(PhysicsWorld& world, uint32_t query_obj_idx,
    uint32_t particle_idx)
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
   
            world.particle_rigid_contacts_.emplace_back(particle_idx, query_obj_idx, contact_normal, contact_point, -depth);
        },
        [&](const CircleShape& c)
        {
            const Vec2 p_pos_map_to_circle = world.particle_positions_[particle_idx] - world.positions_[query_obj_idx].val;
            const float depth = SdfCircle(p_pos_map_to_circle, c.radius);
            if (depth >= 0.f) return;
            
            Vec2 contact_normal = p_pos_map_to_circle.Normalized() ;
            Vec2 contact_point = world.particle_positions_[particle_idx] + contact_normal * -depth;
        
            world.particle_rigid_contacts_.emplace_back(particle_idx, query_obj_idx, contact_normal, contact_point, -depth);
        },
        [&](const PolygonShape& p){},
    }, shape);
}

uint32_t CollisionNarrowPhase::CountParticleEdgeCollisions(const PhysicsWorld& world, const SoftBodyHandle& soft_body_a,
    const SoftBodyHandle& soft_body_b)
{
    uint32_t count = 0;
    constexpr float total_thickness_sq = physics_config::kTwoParticlesThickness * physics_config::kTwoParticlesThickness;

    for (uint32_t i = soft_body_a.start_surface_particle_idx; i < soft_body_a.start_surface_particle_idx + soft_body_a.surface_particle_count; ++i) 
    {
        const auto particle_idx = world.surface_particles_idx_[i];
        const Vec2& particle_pos = world.particle_positions_[particle_idx];
        
        for (uint16_t j = 0; j < soft_body_b.edge_count; ++j)
        {
            const auto edge_idx = soft_body_b.start_edge_idx + j;
            const auto& edge = world.surface_edges_[edge_idx];
            
            const Vec2& a = world.particle_positions_[edge.first];
            const Vec2& b = world.particle_positions_[edge.second];

            Vec2 ab = b - a;
            const float ab_len_sq = ab.LengthSquared();
            if (ab_len_sq < 1e-6f) continue;

            const float t = (particle_pos - a).Dot(ab) / ab_len_sq;
            if (t < 0.f || t > 1.f) continue;
            if (Vec2 proj = a + ab * t; (particle_pos - proj).LengthSquared() > total_thickness_sq) continue; 
            
            ++count;
        }
    }

    return count;
}

uint32_t CollisionNarrowPhase::CheckParticleEdgeCollision(const PhysicsWorld& world, const SoftBodyHandle& soft_body_a,
    const SoftBodyHandle& soft_body_b, XpbdParticleEdgeCollisionConstraint* out_contacts)
{
    uint32_t write_offset = 0;
    constexpr float total_thickness_sq = physics_config::kTwoParticlesThickness * physics_config::kTwoParticlesThickness;
    const bool is_b_closed = world.surface_edges_[soft_body_b.start_edge_idx + soft_body_b.edge_count - 1].second == world.surface_edges_[soft_body_b.start_edge_idx].first;
    
    for (uint32_t i = soft_body_a.start_surface_particle_idx; i < soft_body_a.start_surface_particle_idx + soft_body_a.surface_particle_count; ++i) 
    {
        const auto particle_idx = world.surface_particles_idx_[i];
        const Vec2& particle_pos = world.particle_positions_[particle_idx];
        
        for (uint16_t j = 0; j < soft_body_b.edge_count; ++j)
        {
            const auto edge_idx = soft_body_b.start_edge_idx + j;
            const auto& edge = world.surface_edges_[edge_idx];
            
            const Vec2& a_pos = world.particle_positions_[edge.first];
            const Vec2& b_pos = world.particle_positions_[edge.second];

            Vec2 ab = b_pos - a_pos;
            const float ab_len_sq = ab.LengthSquared();
            if (ab_len_sq < 1e-6f) continue;

            const float t = (particle_pos - a_pos).Dot(ab) / ab_len_sq;
            if (t < 0.f || t > 1.f) continue;
            
            Vec2 proj = a_pos + ab * t;
            Vec2 diff = particle_pos - proj;

            const float dist_sq = diff.LengthSquared();
            if (dist_sq > total_thickness_sq) continue;
            
            const float dist = std::sqrt(dist_sq);
            const Vec2 normal = is_b_closed ? ab.Normal() : dist > 1e-6f ? diff / dist : Vec2{0.f, 1.f};
            
            out_contacts[write_offset++] = XpbdParticleEdgeCollisionConstraint{normal, particle_idx, 
                edge.first, edge.second, t};
        }
    }
    
    return write_offset;
}

void CollisionNarrowPhase::CheckSoftSoftCollision(PhysicsWorld& world,
    const std::pmr::vector<std::pair<uint32_t, uint32_t>>& pairs)
{
    const size_t num_pairs = pairs.size();
    if (num_pairs == 0) return;
    
    auto index_range = std::views::iota(size_t{0}, num_pairs);
    std::pmr::vector<uint32_t> counts(&world.frame_arena_);
    counts.resize(num_pairs);
    std::ranges::fill(counts, 0);
    
    // Calculate how many contact each pair has.
    std::for_each(std::execution::par_unseq, index_range.begin(), index_range.end(), [&](const size_t i) {
        const auto& body_a_handle = world.soft_body_handles_[pairs[i].first];
        const auto& body_b_handle = world.soft_body_handles_[pairs[i].second];
        
        counts[i] = CountParticleEdgeCollisions(world, body_a_handle, body_b_handle) + CountParticleEdgeCollisions(world, body_b_handle, body_a_handle);
    });

    // Calculate the Prefix Sum to obtain the Offset for each Task written to `world.particle_edge_contacts_`.
    std::pmr::vector<uint32_t> offsets(&world.frame_arena_);
    offsets.resize(num_pairs);
    std::exclusive_scan(std::execution::par_unseq, counts.begin(), counts.end(), offsets.begin(), 0u);

    const uint32_t total_contacts = offsets.back() + counts.back();
    if (total_contacts == 0) return;
    
    world.particle_edge_contacts_.resize(total_contacts);

    auto fill_contacts = [&](const size_t i) {
        if (counts[i] <= 0) return;
        
        const auto& body_a_handle = world.soft_body_handles_[pairs[i].first];
        const auto& body_b_handle = world.soft_body_handles_[pairs[i].second];

        const uint32_t current_offset = offsets[i];
        XpbdParticleEdgeCollisionConstraint* write_ptr = world.particle_edge_contacts_.data() + current_offset;
        
        const uint32_t written_a_vs_b = CheckParticleEdgeCollision(world, body_a_handle, body_b_handle, write_ptr);
        CheckParticleEdgeCollision(world, body_b_handle, body_a_handle, write_ptr + written_a_vs_b);
    };
    
    // Parallel direct filling into the `world.particle_edge_contacts_`
    if (world.is_multi_thread_){
        std::for_each(std::execution::par_unseq, index_range.begin(), index_range.end(), fill_contacts);
    }
    else{
        std::for_each(std::execution::seq, index_range.begin(), index_range.end(), fill_contacts);
    }
}
