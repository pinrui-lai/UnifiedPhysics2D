#include <algorithm>

#include "physics_world.h"

Aabb MortonBvh::ComputeFastAabbFromLocalAabb(const Vec2& pos, const float rot, const Aabb& local_aabb)
{
    const Vec2 local_aabb_min{local_aabb.min_x, local_aabb.min_y};
    const Vec2 local_aabb_max{local_aabb.max_x, local_aabb.max_y};
    
    // Calculate the center and extent of local aabb.
    const Vec2 local_center = (local_aabb_min + local_aabb_max) * 0.5f;
    const Vec2 local_extent = (local_aabb_max - local_aabb_min) * 0.5f;

    // Transform center to world space.
    const float c = std::cos(rot);
    const float s = std::sin(rot);
    const Vec2 world_center = Vec2{local_center.x * c - local_center.y * s, local_center.x * s + local_center.y * c} + pos;

    const float abs_c = std::abs(c);
    const float abs_s = std::abs(s);

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

uint32_t MortonBvh::ExpandBits(uint32_t bits)
{
     // 00000000 00000000 11111111 11111111 -> 00000000 11111111 00000000 11111111 -> 00001111 00001111 00001111 00001111
     // -> 00110011 00110011 00110011 00110011 -> 01010101 01010101 01010101 01010101
    bits = (bits | (bits << 8)) & 0x00FF00FF;
    bits = (bits | (bits << 4)) & 0x0F0F0F0F;
    bits = (bits | (bits << 2)) & 0x33333333;
    bits = (bits | (bits << 1)) & 0x55555555;
    return bits;
}

uint32_t MortonBvh::GetMortonCode(const Vec2& pos, const float min_x, const float min_y, const float max_x, const float max_y)
{
    // Clamp to avoid floating-point precision issues.
    const float norm_x = std::clamp((pos.x - min_x) / (max_x - min_x), 0.0f, 1.0f);
    const float norm_y = std::clamp((pos.y - min_y) / (max_y - min_y), 0.0f, 1.0f);

    const uint32_t x = static_cast<uint32_t>(norm_x * 65535.f);
    const uint32_t y = static_cast<uint32_t>(norm_y * 65535.f);
    
    return (ExpandBits(y) << 1) | ExpandBits(x); 
}

void MortonBvh::Build(PhysicsWorld& world)
{
    const size_t rigid_count = world.positions_.size();
    const size_t soft_count = world.soft_body_handles_.size();
    const size_t total_count = rigid_count + soft_count;
    
    if (total_count == 0) return;
    
    nodes_.clear();
    world_aabbs_.clear();
    world_aabbs_.reserve(total_count);
    
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
                world_aabbs_.emplace_back(local_min_x + pos.x, local_min_y + pos.y,
                    local_max_x + pos.x, local_max_y + pos.y);
            }
            else
            {
                world_aabbs_.push_back(ComputeFastAabbFromLocalAabb(world.positions_[i].val, world.rotations_[i].val, world.local_aabbs_[i]));
            }
            
        }, world.shapes_[i]);
    }
    
    // XPBD Soft Body world AABBs.
    for (size_t i = 0; i < soft_count; ++i)
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

        world_aabbs_.emplace_back(min_x - 0.05f, min_y - 0.05f, max_x + 0.05f, max_y + 0.05f);
    }
    
    // Get Scene Bounding Box (boundary of all objects in the world).
    Aabb scene_aabb = world_aabbs_[0];
    for (const auto& world_aabb : world_aabbs_) {
        scene_aabb = Aabb::Combine(scene_aabb, world_aabb);
    }

    // Get Morton Code and create sort item.
    std::pmr::vector<Item> items(&world.frame_arena_);
    items.reserve(total_count);
    
    for (size_t i = 0; i < rigid_count; ++i)
    {
        items.emplace_back(i, GetMortonCode(world.positions_[i].val, scene_aabb.min_x, scene_aabb.min_y, scene_aabb.max_x, scene_aabb.max_y));
    }
    for (size_t i = 0; i < soft_count; ++i)
    {
        const size_t unified_idx = rigid_count + i;
        auto& [min_x, min_y, max_x, max_y] = world_aabbs_[unified_idx];
        auto center = Vec2{(max_x - min_x) * 0.5f, (max_y - min_y) * 0.5f};
        items.emplace_back(unified_idx, GetMortonCode(center, scene_aabb.min_x, scene_aabb.min_y, scene_aabb.max_x, scene_aabb.max_y));
    }
    
    // Sort by morton code.
    std::ranges::sort(items, {}, &Item::morton_code);
    
    root_index_ = BuildRecursive(items, world_aabbs_, 0, items.size());
}

void MortonBvh::Query(PhysicsWorld& world, size_t query_obj_idx)
{
    if (root_index_ == std::numeric_limits<size_t>::max()) return;
    
    const size_t rigid_count = world.positions_.size();
    if (query_obj_idx >= rigid_count) return; // No Soft to Soft query
    
    std::pmr::vector<size_t> stack(&world.frame_arena_);
    stack.reserve(64);
    stack.push_back(root_index_);

    while (!stack.empty()) {
        const size_t curr = stack.back();
        stack.pop_back();

        const BvhNode& node = nodes_[curr];
        if (Aabb::Overlap(node.aabb, world_aabbs_[query_obj_idx])) {
            if (node.IsLeaf()) {
                const size_t leaf_id = node.leaf_id;
                
                // Rigid vs Rigid
                if (leaf_id < rigid_count)
                {
                    if (node.leaf_id == query_obj_idx) continue;
                
                    auto it = std::ranges::find_if(world.manifolds_, [&](const Manifold& manifold)
                    {
                        return (manifold.a == query_obj_idx && manifold.b == node.leaf_id)
                            || (manifold.b == query_obj_idx && manifold.a == node.leaf_id);
                    });
                
                    if (it != world.manifolds_.end()) {
                        it->is_overlapping = true;
                    }
                    else{
                        world.manifolds_.emplace_back(query_obj_idx, node.leaf_id);
                    }
                }
                // Rigid bs Soft
                else
                {
                    const size_t soft_idx = leaf_id - rigid_count;
                    for (size_t i = 0; i < world.soft_body_handles_[soft_idx].particle_count; ++i)
                    {
                        size_t particle_idx = world.soft_body_handles_[soft_idx].start_particle_idx + i;
                        
                        // Call Narrow-Phase for Rigid vs Soft particles(SDF).
                        XpbdContactConstraint contact;
                        if (CheckRigidParticleCollision(world, query_obj_idx, particle_idx, contact)){
                            world.soft_rigid_contacts_.push_back(contact);
                        }
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

size_t MortonBvh::BuildRecursive(std::pmr::vector<Item>& items, const std::vector<Aabb>& world_aabbs, const size_t start, const size_t end)
{
    // Index of current node.
    const size_t node_index = nodes_.size();
    
    // Place a placeholder in nodes_.
    nodes_.emplace_back();

    const int count = end - start;
    
    // Leaf node.
    if (count == 1)
    {
        nodes_[node_index].aabb = world_aabbs[items[start].id];
        nodes_[node_index].leaf_id = items[start].id;
        return node_index;
    }
    
    // Find the median and divide in half.
    const int mid = start + count / 2;
    
    const size_t left_node_index = BuildRecursive(items, world_aabbs, start, mid);
    const size_t right_node_index = BuildRecursive(items, world_aabbs, mid, end);
    
    nodes_[node_index].aabb = Aabb::Combine(nodes_[left_node_index].aabb, nodes_[right_node_index].aabb);
    nodes_[node_index].left_child = left_node_index;
    nodes_[node_index].right_child = right_node_index;
    
    return node_index;
}

namespace 
{
    template<class... Ts> struct Overload : Ts... { using Ts::operator()...; };    
}
bool MortonBvh::CheckRigidParticleCollision(PhysicsWorld& world, const size_t query_obj_idx, const size_t particle_idx,
    XpbdContactConstraint& out_constraint)
{
    Shape& shape = world.shapes_[query_obj_idx];
    return std::visit(Overload{
        [&](const BoxShape& b)
        {
            const float s = std::sin(world.rotations_[query_obj_idx].val);
            const float c = std::cos(world.rotations_[query_obj_idx].val);
            
            Vec2 p_pos_map_to_box = world.particle_positions_[particle_idx] - world.positions_[query_obj_idx].val;
            p_pos_map_to_box = {p_pos_map_to_box.x * c + p_pos_map_to_box.y * s , - p_pos_map_to_box.x * s + p_pos_map_to_box.y * c};
            const float depth = SdfBox(p_pos_map_to_box, {b.half_width, b.half_height});
            if (depth >= 0.f) return false;
            
            out_constraint.particle_idx = particle_idx;
            out_constraint.body_idx = query_obj_idx;
            out_constraint.depth = -depth;

            const Vec2 local_normal = GetBoxNormal(p_pos_map_to_box, {b.half_width, b.half_height});
            out_constraint.contact_normal = Vec2{local_normal.x * c - local_normal.y * s, local_normal.x * s + local_normal.y * c} ;
            out_constraint.contact_point = world.particle_positions_[particle_idx] + out_constraint.contact_normal * out_constraint.depth;
            
            return true;
        },
        [&](const CircleShape& c)
        {
            const Vec2 p_pos_map_to_circle = world.particle_positions_[particle_idx] - world.positions_[query_obj_idx].val;
            const float depth = SdfCircle(p_pos_map_to_circle, c.radius);
            if (depth >= 0.f) return false;
            
            out_constraint.particle_idx = particle_idx;
            out_constraint.body_idx = query_obj_idx;
            out_constraint.depth = -depth;
            out_constraint.contact_normal = p_pos_map_to_circle.Normalized() ;
            out_constraint.contact_point = world.particle_positions_[particle_idx] + out_constraint.contact_normal * out_constraint.depth;
        
            return true;
        },
        [&](const PolygonShape& p){ return false; },
    }, shape);
}

float MortonBvh::SdfBox(const Vec2 p, const Vec2 b) const
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

Vec2 MortonBvh::GetBoxNormal(const Vec2 p, const Vec2 b)
{
    const Vec2 d = { std::abs(p.x) - b.x, std::abs(p.y) - b.y };
    // Inside the box, the shortest separation direction is perpendicular to the nearest edge.
    Vec2 n = (d.x > d.y) ? Vec2{1.0f, 0.0f} : Vec2{0.0f, 1.0f};
    
    // Map to the quadrant where point p is located.
    if (p.x < 0.0f) n.x = -n.x;
    if (p.y < 0.0f) n.y = -n.y;

    return n;
}
