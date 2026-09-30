#include <algorithm>

#include "collision_detection_system.h"
#include "physics_world.h"

void CollisionDetectionSystem::CheckCollision(PhysicsWorld& world)
{
    using Clock = std::chrono::high_resolution_clock;
    using Duration = std::chrono::duration<double, std::milli>;
    
    world.debug_incident_edges_.clear();
    world.debug_reference_edges_.clear();
        
    // Mark all manifold's is_overlapping false.
    for (auto& manifold : world.manifolds_)
    {
        manifold.is_overlapping = false;
    }
    
    // ===== Broad-Phase =====
    auto start = Clock::now();
    const uint32_t rigid_count = world.positions_.size();
    const uint32_t soft_count = world.soft_body_handles_.size();
    const uint32_t total_count = rigid_count + soft_count;
    if (total_count == 0) return;
    
    std::pmr::vector<Aabb> world_aabbs(&world.frame_arena_);
    world_aabbs.reserve(total_count);
    
    std::pmr::vector<std::pair<uint32_t, uint32_t>> rigid_particle_contacts(&world.frame_arena_);
    rigid_particle_contacts.reserve(100);
    
    std::pmr::vector<std::pair<uint32_t, uint32_t>> soft_soft_contacts(&world.frame_arena_);
    soft_soft_contacts.reserve(100);
    
    BuildBvh(world, world_aabbs);
    
    // Query rigid bodies for rigid-rigid and rigid-soft contact.
    for (uint32_t i = 0; i < rigid_count; ++i)
    {
        QueryBvh(world, world_aabbs, i, rigid_particle_contacts, soft_soft_contacts);
    }
    
    // Query soft bodies for soft-soft contact.
    for (uint32_t i = 0; i < soft_count; ++i)
    {
        const uint32_t idx = rigid_count + i;
        QueryBvh(world, world_aabbs, idx, rigid_particle_contacts, soft_soft_contacts);
    }
    
    // Remove manifold whose AABBs no longer overlap.
    std::erase_if(world.manifolds_, [](const Manifold& manifold)
    {
        return manifold.is_overlapping == false;
    });
    world.profile_data_.broad_phase_ms += Duration(Clock::now() - start).count();
    
    // ===== Narrow-Phase =====
    start = Clock::now();
    
    // Rigid vs Soft Particles (SDF).
    for (const auto& [fst, snd] : rigid_particle_contacts)
    {
        CheckRigidParticleCollision(world, fst, snd);
    }
    
    // Soft vs Soft.
    for (const auto& [fst, snd] : soft_soft_contacts)
    {
        CheckSoftSoftCollision(world, fst, snd);
    }
    
    // Rigid vs Rigid (GJK + EPA + Clipping).
    for (auto& manifold : world.manifolds_)
    {
        if (world.rigid_bodies_[manifold.a].inv_mass == 0.0f && world.rigid_bodies_[manifold.b].inv_mass == 0.0f) continue;
        
        // Store old contacts and contact_count from the previous frame.
        std::array<Contact, 2> old_contacts = manifold.contacts;
        const uint8_t old_contact_count = manifold.contact_count;
        
        // Write new contacts of the current frame in manifold.
        Gjk(world, manifold);
        
        // Match Feature ID and inherit old impulse.
        for (uint8_t i = 0; i < manifold.contact_count; ++i)
        {
            Contact& new_c = manifold.contacts[i];
        
            for (uint8_t j = 0; j < old_contact_count; ++j)
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
    world.profile_data_.narrow_phase_ms += Duration(Clock::now() - start).count();
}

void CollisionDetectionSystem::Gjk(PhysicsWorld& world, Manifold& manifold)
{
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

Vec2 CollisionDetectionSystem::GetSupport(const PhysicsWorld& world, const uint32_t i, const Vec2 dir)
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

std::pair<Vec2, float> CollisionDetectionSystem::Epa(const PhysicsWorld& world, const FixedSimplex& simplex, const Manifold& manifold)
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
            std::array<uint32_t, 2> edge_ids_a;
            float edge_a_dot_normal;
            GetMostPerpEdge(GetVertices(world, a, index_a), collision_normal, edge_pos_a, edge_a_dot_normal, edge_ids_a);
            
            contact1.id.features.type_a = 1;
            contact1.id.features.index_a = edge_ids_a[0];
        }
        else
        {
            // Decide incident edge and reference edge.
            std::array<Vec2, 2> edge_pos_a;
            std::array<uint32_t, 2> edge_ids_a;
            float edge_a_dot_normal;
            GetMostPerpEdge(GetVertices(world, a, index_a), collision_normal, edge_pos_a, edge_a_dot_normal, edge_ids_a);
            
            std::array<Vec2, 2> edge_pos_b;
            std::array<uint32_t, 2> edge_ids_b;
            float edge_b_dot_normal;
            GetMostPerpEdge(GetVertices(world, b, index_b), -collision_normal, edge_pos_b, edge_b_dot_normal, edge_ids_b);
            
            const bool is_ref_a = edge_a_dot_normal > edge_b_dot_normal;
            const std::array<Vec2, 2> ref_edge_pos = edge_a_dot_normal > edge_b_dot_normal ? edge_pos_a : edge_pos_b;
            const std::array<Vec2, 2> inc_edge_pos = edge_a_dot_normal > edge_b_dot_normal ? edge_pos_b : edge_pos_a;
            const std::array<uint32_t, 2> ref_edge_ids = edge_a_dot_normal > edge_b_dot_normal ? edge_ids_a : edge_ids_b;
            const std::array<uint32_t, 2> inc_edge_ids = edge_a_dot_normal > edge_b_dot_normal ? edge_ids_b : edge_ids_a;
            
            if (world.show_debug_clipping_)
            {
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
    float& out_dot, std::array<uint32_t, 2>& out_edge_ids)
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

void CollisionDetectionSystem::ClipEdge(const Vec2 clip_face_inner_normal, const Vec2 point_on_clip_face,
    const uint32_t clip_face_id, std::array<Vec2, 2>& out_clipped_edges, std::array<int, 2>& out_clip_face_ids)
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

std::array<Vec2, 4> CollisionDetectionSystem::GetVertices(const PhysicsWorld& world, const BoxShape& box_shape, uint32_t i)
{
    // Make sure vertices are arranged in clockwise.
    std::array<Vec2, 4> vertices;
    vertices[0] = {-box_shape.half_width, box_shape.half_height};
    vertices[1] = {box_shape.half_width, box_shape.half_height};
    vertices[2] = {box_shape.half_width, -box_shape.half_height};
    vertices[3] = {-box_shape.half_width, -box_shape.half_height};
    
    std::ranges::transform(vertices, vertices.begin(), [&](const Vec2 v){ return world.rots_[i].Mul(v) + world.positions_[i].val; });
    
    return vertices;
}

std::span<const Vec2> CollisionDetectionSystem::GetVertices(PhysicsWorld& world, const PolygonShape& polygon_shape,
    uint32_t i)
{
    std::span<Vec2> local_vertices = std::span(world.vertices_).subspan(polygon_shape.vertex_offset, polygon_shape.vertex_count);
    
    // TODO : transform to world space.
    
    return local_vertices;
}

Aabb CollisionDetectionSystem::ComputeFastAabbFromLocalAabb(const Vec2& pos, const Rot rot, const Aabb& local_aabb)
{
    const Vec2 local_aabb_min{local_aabb.min_x, local_aabb.min_y};
    const Vec2 local_aabb_max{local_aabb.max_x, local_aabb.max_y};
    
    // Calculate the center and extent of local aabb.
    const Vec2 local_center = (local_aabb_min + local_aabb_max) * 0.5f;
    const Vec2 local_extent = (local_aabb_max - local_aabb_min) * 0.5f;

    // Transform center to world space.
    const Vec2 world_center = rot.Mul(local_center) + pos;

    const float abs_c = std::abs(rot.c);
    const float abs_s = std::abs(rot.s);

    // Calculate extent of AABB of rotated Local AABB(OBB)
    const Vec2 world_extent{
        abs_c * local_extent.x + abs_s * local_extent.y,
        abs_s * local_extent.x + abs_c * local_extent.y
    };

    // World AABB
    const Vec2 world_aabb_min = world_center - world_extent;
    const Vec2 world_aabb_max = world_center + world_extent;
    return Aabb{.min_x = world_aabb_min.x, .min_y = world_aabb_min.y, .max_x = world_aabb_max.x, .max_y = world_aabb_max.y };
}

uint32_t CollisionDetectionSystem::ExpandBits(uint32_t bits)
{
     // 00000000 00000000 11111111 11111111 -> 00000000 11111111 00000000 11111111 -> 00001111 00001111 00001111 00001111 ...
    bits = (bits | (bits << 8)) & 0x00FF00FF;
    bits = (bits | (bits << 4)) & 0x0F0F0F0F;
    bits = (bits | (bits << 2)) & 0x33333333;
    bits = (bits | (bits << 1)) & 0x55555555;
    return bits;
}

uint32_t CollisionDetectionSystem::GetMortonCode(const Vec2& pos, const float min_x, const float min_y, const float max_x, const float max_y)
{
    // Clamp to avoid floating-point precision issues.
    const float norm_x = std::clamp((pos.x - min_x) / (max_x - min_x), 0.0f, 1.0f);
    const float norm_y = std::clamp((pos.y - min_y) / (max_y - min_y), 0.0f, 1.0f);

    const uint32_t x = static_cast<uint32_t>(norm_x * 65535.f);
    const uint32_t y = static_cast<uint32_t>(norm_y * 65535.f);
    
    return (ExpandBits(y) << 1) | ExpandBits(x); 
}

void CollisionDetectionSystem::BuildBvh(PhysicsWorld& world, std::pmr::vector<Aabb>& world_aabbs)
{
    const uint32_t rigid_count = world.positions_.size();
    const uint32_t soft_count = world.soft_body_handles_.size();
    
    world.bvh_tree_nodes_.clear();
    
    // Rigid Body world AABBs.
    for (auto i = 0; i < world.local_aabbs_.size(); ++i)
    {
        std::visit([&](auto& s)
        {
            using S = std::decay_t<decltype(s)>;
            
            if constexpr (std::is_same_v<S, CircleShape>)
            {
                const Vec2 pos = world.positions_[i].val;
                auto [local_min_x, local_min_y, local_max_x, local_max_y] = world.local_aabbs_[i];
                world_aabbs.emplace_back(local_min_x + pos.x, local_min_y + pos.y,
                    local_max_x + pos.x, local_max_y + pos.y);
            }
            else
            {
                world_aabbs.push_back(ComputeFastAabbFromLocalAabb(world.positions_[i].val, world.rots_[i], world.local_aabbs_[i]));
            }
            
        }, world.shapes_[i]);
    }
    
    // XPBD Soft Body world AABBs.
    for (uint32_t i = 0; i < soft_count; ++i)
    {
        float min_x = std::numeric_limits<float>::max();
        float min_y = std::numeric_limits<float>::max();
        float max_x = std::numeric_limits<float>::min();
        float max_y = std::numeric_limits<float>::min();
        
        for (const auto& pos : world.GetSoftBodyParticlePositions(i))
        {
            min_x = std::min(min_x, pos.x);
            min_y = std::min(min_y, pos.y);
            max_x = std::max(max_x, pos.x);
            max_y = std::max(max_y, pos.y);
        }

        world_aabbs.emplace_back(min_x - 0.05f, min_y - 0.05f, max_x + 0.05f, max_y + 0.05f);
    }
    
    // Get Scene Bounding Box (boundary of all bodies in the world).
    Aabb scene_aabb = world_aabbs[0];
    for (const auto& world_aabb : world_aabbs) {
        scene_aabb = Aabb::Combine(scene_aabb, world_aabb);
    }

    // Get Morton Code and create sort item.
    std::pmr::vector<Item> items(&world.frame_arena_);
    items.reserve(world_aabbs.size());
    
    for (uint32_t i = 0; i < rigid_count; ++i)
    {
        items.emplace_back(i, GetMortonCode(world.positions_[i].val, scene_aabb.min_x, scene_aabb.min_y, scene_aabb.max_x, scene_aabb.max_y));
    }
    for (uint32_t i = 0; i < soft_count; ++i)
    {
        const uint32_t unified_idx = rigid_count + i;
        auto& [min_x, min_y, max_x, max_y] = world_aabbs[unified_idx];
        auto center = Vec2{(max_x - min_x) * 0.5f, (max_y - min_y) * 0.5f};
        items.emplace_back(unified_idx, GetMortonCode(center, scene_aabb.min_x, scene_aabb.min_y, scene_aabb.max_x, scene_aabb.max_y));
    }
    
    // Sort by morton code.
    std::ranges::sort(items, {}, &Item::morton_code);
    
    world.bvh_tree_root_index_ = BuildBvhRecursive(world, items, world_aabbs, 0, items.size());
}

void CollisionDetectionSystem::QueryBvh(PhysicsWorld& world, const std::pmr::vector<Aabb>& world_aabbs, uint32_t query_obj_idx,
    std::pmr::vector<std::pair<uint32_t, uint32_t>>& rigid_soft_contacts, std::pmr::vector<std::pair<uint32_t, uint32_t>>& soft_soft_contacts)
{
    if (world.bvh_tree_root_index_ == std::numeric_limits<uint32_t>::max()) return;
    
    const uint32_t rigid_count = world.positions_.size();
    
    std::pmr::vector<uint32_t> stack(&world.frame_arena_);
    stack.reserve(64);
    stack.push_back(world.bvh_tree_root_index_);

    while (!stack.empty()) {
        const uint32_t curr = stack.back();
        stack.pop_back();

        if (const BvhNode& node = world.bvh_tree_nodes_[curr]; Aabb::Overlap(node.aabb, world_aabbs[query_obj_idx])) 
        {
            if (node.IsLeaf()) 
            {
                // Rigid vs Rigid
                if (const uint32_t leaf_id = node.leaf_id; leaf_id < rigid_count)
                {
                    if (node.leaf_id == query_obj_idx || query_obj_idx >= rigid_count) continue;
                
                    auto it = std::ranges::find_if(world.manifolds_, [&](const Manifold& manifold)
                    {
                        return (manifold.a == query_obj_idx && manifold.b == node.leaf_id)
                            || (manifold.b == query_obj_idx && manifold.a == node.leaf_id);
                    });
                
                    if (it != world.manifolds_.end()) 
                    {
                        it->is_overlapping = true;
                    }
                    else
                    {
                        world.manifolds_.emplace_back(query_obj_idx, node.leaf_id);
                    }
                }
                else
                {
                    const uint32_t soft_idx = leaf_id - rigid_count;
                    
                    // Rigid vs Soft (Particle)
                    if (query_obj_idx < rigid_count)
                    {
                        const auto& soft_handle = world.soft_body_handles_[soft_idx];
                        for (uint32_t i = soft_handle.start_surface_particle_idx; i < soft_handle.start_surface_particle_idx + soft_handle.surface_particle_count; ++i)
                        {
                            const auto particle_idx = world.surface_particles_idx_[i]; // Only detect surface particle of soft body
                            rigid_soft_contacts.emplace_back(query_obj_idx, particle_idx);
                        }
                    }
                    // Soft vs Soft
                    else
                    {
                        const uint32_t query_soft_idx = query_obj_idx - rigid_count;
                        if (query_soft_idx == soft_idx) continue;
                        
                        soft_soft_contacts.emplace_back(query_soft_idx, soft_idx);
                    }
                }
            } 
            else
            {
                stack.push_back(node.left_child);
                stack.push_back(node.right_child);
            }
        }
    }
}

uint32_t CollisionDetectionSystem::BuildBvhRecursive(PhysicsWorld& world, std::pmr::vector<Item>& items, const std::pmr::vector<Aabb>& world_aabbs, const int start, const int end)
{
    // Index of current node.
    const uint32_t node_index = world.bvh_tree_nodes_.size();
    
    // Place a placeholder in bvh_tree_nodes_.
    world.bvh_tree_nodes_.emplace_back();

    const int count = end - start;
    
    // Leaf node.
    if (count == 1)
    {
        world.bvh_tree_nodes_[node_index].aabb = world_aabbs[items[start].id];
        world.bvh_tree_nodes_[node_index].leaf_id = items[start].id;
        return node_index;
    }
    
    // Find the median and divide in half.
    const int mid = start + count / 2;
    
    const uint32_t left_node_index = BuildBvhRecursive(world, items, world_aabbs, start, mid);
    const uint32_t right_node_index = BuildBvhRecursive(world, items, world_aabbs, mid, end);
    
    world.bvh_tree_nodes_[node_index].aabb = Aabb::Combine(world.bvh_tree_nodes_[left_node_index].aabb, world.bvh_tree_nodes_[right_node_index].aabb);
    world.bvh_tree_nodes_[node_index].left_child = left_node_index;
    world.bvh_tree_nodes_[node_index].right_child = right_node_index;
    
    return node_index;
}

void CollisionDetectionSystem::CheckRigidParticleCollision(PhysicsWorld& world, const uint32_t query_obj_idx, const uint32_t particle_idx)
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
   
            world.soft_rigid_contacts_.emplace_back(particle_idx, query_obj_idx, contact_normal, contact_point, -depth);
        },
        [&](const CircleShape& c)
        {
            const Vec2 p_pos_map_to_circle = world.particle_positions_[particle_idx] - world.positions_[query_obj_idx].val;
            const float depth = SdfCircle(p_pos_map_to_circle, c.radius);
            if (depth >= 0.f) return;
            
            Vec2 contact_normal = p_pos_map_to_circle.Normalized() ;
            Vec2 contact_point = world.particle_positions_[particle_idx] + contact_normal * -depth;
        
            world.soft_rigid_contacts_.emplace_back(particle_idx, query_obj_idx, contact_normal, contact_point, -depth);
        },
        [&](const PolygonShape& p){},
    }, shape);
}

void CollisionDetectionSystem::CheckParticleEdgeCollision(PhysicsWorld& world, const float total_thickness, const SoftBodyHandle& soft_body_a, const SoftBodyHandle& soft_body_b)
{
    for (uint32_t i = soft_body_a.start_surface_particle_idx; i < soft_body_a.start_surface_particle_idx + soft_body_a.surface_particle_count; ++i) 
    {
        const auto particle_idx = world.surface_particles_idx_[i]; // Only detect surface particles.
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

            // Calculate the projection ratio t of P onto line AB.
            float t = (particle_pos - a).Dot(ab) / ab_len_sq;
            if (t < 0.f || t > 1.f) continue;
            
            Vec2 proj = a + ab * t;
            Vec2 diff = particle_pos - proj;

            // The distance is less than the thickness of the two particles.
            if (const float dist = diff.Length(); dist < total_thickness) 
            {
                const Vec2 normal = (dist > 1e-6f) ? diff / dist : Vec2{0.0f, 1.0f};
                world.particle_edge_collision_constraints_.emplace_back(normal, particle_idx, edge.first, edge.second, -(dist - total_thickness), t);
            }
        }
    }
}

void CollisionDetectionSystem::CheckSoftSoftCollision(PhysicsWorld& world, const uint32_t soft_idx1, const uint32_t soft_idx2)
{
    CheckParticleEdgeCollision(world, physics_config::kTwoParticlesThickness, world.soft_body_handles_[soft_idx1], world.soft_body_handles_[soft_idx2]);
    CheckParticleEdgeCollision(world, physics_config::kTwoParticlesThickness, world.soft_body_handles_[soft_idx2], world.soft_body_handles_[soft_idx1]);
}

float CollisionDetectionSystem::SdfBox(const Vec2 p, const Vec2 b)
{
    // Map the point to the first quadrant.
    const Vec2 d = { std::abs(p.x) - b.x, std::abs(p.y) - b.y };

    // External distance (point outside the rectangle corner or boundary)
    const Vec2 outside_vec = { std::max(d.x, 0.0f), std::max(d.y, 0.0f) };
    const float outside_dist = outside_vec.Length();

    // Interior distance (negative value if the point is inside the rectangle)
    const float inside_dist = std::min(std::max(d.x, d.y), 0.0f);

    return outside_dist + inside_dist;
}

Vec2 CollisionDetectionSystem::GetParticleBoxCollisionNormal(const Vec2 p, const Vec2 b)
{
    const Vec2 d = { std::abs(p.x) - b.x, std::abs(p.y) - b.y };
    
    // Inside the box, the shortest separation direction is perpendicular to the nearest edge.
    Vec2 n = (d.x > d.y) ? Vec2{1.0f, 0.0f} : Vec2{0.0f, 1.0f};
    
    // Map to the quadrant where point p is located.
    if (p.x < 0.0f) n.x = -n.x;
    if (p.y < 0.0f) n.y = -n.y;

    return n;
}
