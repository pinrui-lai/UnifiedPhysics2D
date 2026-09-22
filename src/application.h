#pragma once
#include "types.h"

class PhysicsWorld;

class Application
{
public:
    void CreateInitialObjects(SDL_Window* window, PhysicsWorld& world);
    void Input(SDL_Window* window, bool &is_running, PhysicsWorld& world);
    void Update(PhysicsWorld& world, const float dt);
    void Render(SDL_Renderer* renderer, SDL_Window* window, PhysicsWorld& world);
    void RenderPhysicsControlPanel(PhysicsWorld& world);
    static ImU32 GetDepthColor(int depth, float alpha = 1.0f);
    static void DrawBvhNode(ImDrawList* draw_list, const std::vector<BvhNode>& nodes, int node_idx, int current_depth, int max_render_depth, int& hovered_node_idx, float window_height);
    static void RenderBvhDebug(const std::vector<BvhNode>& nodes, int root_idx, int max_render_depth, float window_height);
    static ImVec2 GetAabbCenter(const Aabb& aabb) { return ImVec2((aabb.min_x + aabb.max_x) * 0.5f, (aabb.min_y + aabb.max_y) * 0.5f); }
    static bool AabbContains(const Aabb& aabb, const ImVec2 p) { return p.x >= aabb.min_x && p.x <= aabb.max_x && p.y >= aabb.min_y && p.y <= aabb.max_y; }
    
private:
    BodyHandle floating_circle_{0xFFFFFFFF};
    BodyHandle floating_circle2_{0xFFFFFFFF};
    BodyHandle floating_box_{0xFFFFFFFF};
    BodyHandle floating_wide_box_1_{0xFFFFFFFF};
    BodyHandle floating_wide_box_2_{0xFFFFFFFF};
    BodyHandle ceil_{0xFFFFFFFF};
    BodyHandle floor_{0xFFFFFFFF};
    BodyHandle left_wall_{0xFFFFFFFF};
    BodyHandle right_wall_{0xFFFFFFFF};
    BodyHandle debug_circle_{0xFFFFFFFF};
    BodyHandle debug_box_{0xFFFFFFFF};
    
    std::array<BodyHandle*, 4> boundaries_{&ceil_, &floor_, &left_wall_, &right_wall_ };
    std::array<BodyHandle*, 5> floatings_{ &floating_circle_, &floating_circle2_, &floating_box_, &floating_wide_box_1_, &floating_wide_box_2_};
    
    float floating_objects_angular_vel_{0.5f};
    float floating_objects_restitution_{0.2f};
    float floating_objects_friction_{0.2f};
    
    float boundary_restitution_{0.2f};
    float boundary_friction_{0.2f};
    
    float circle_spawn_mass_{5.f};
    float circle_spawn_radius_{1.f};
    float circle_spawn_restitution_{0.2f};
    float circle_spawn_friction_{0.2f};
    
    float box_spawn_mass_{5.f};
    float box_spawn_half_width_{1.f};
    float box_spawn_half_height_{1.f};
    float box_spawn_restitution_{0.2f};
    float box_spawn_friction_{0.2f};
    
    float arrow_key_force_{500.f};
    bool draw_bvh_{false};
    int bvh_draw_max_depth_{5};
};
