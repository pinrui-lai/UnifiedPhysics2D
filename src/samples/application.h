#pragma once
#include <span>

#include "scene_builder.h"
#include "physics/physics_config.h"
#include "physics/types.h"

class PhysicsWorld;

enum class SpawnType {
    kRigidCircle = 0,
    kRigidBox,
    kSoftRope,
    kSoftBox,
    kSoftCircleArea,
    kSoftCirclePressure,
    kCar
};

class Application
{
public:
    void Render(SDL_Renderer* renderer, SDL_Window* window, PhysicsWorld& world);
    void CreateInitialBodies(SDL_Window* window, PhysicsWorld& world, scenes::PresetScene preset = scenes::PresetScene::kDefaultDemo);
    void Input(SDL_Window* window, bool &is_running, PhysicsWorld& world);
    static void Update(PhysicsWorld& world, float dt);
    
private:
    void RenderPhysicsControlPanel(SDL_Window* window, PhysicsWorld& world, int window_width);
    void SpawnBodyAt(PhysicsWorld& world, SpawnType type, Vec2 spawn_pos) const;
    void RenderSpawnTypeConfigUi(SpawnType type);
    
    static ImU32 GetDepthColor(int depth, float alpha = 1.f);
    static void DrawBvhNode(ImDrawList* draw_list, std::span<const BvhNode> nodes, int node_idx, int current_depth, int max_render_depth, int& hovered_node_idx, float window_height);
    static void RenderBvhDebug(std::span<const BvhNode> nodes, int root_idx, int max_render_depth, float window_height);
    static ImVec2 GetAabbCenter(const Aabb& aabb) { return {(aabb.min_x + aabb.max_x) * 0.5f, (aabb.min_y + aabb.max_y) * 0.5f}; }
    static bool AabbContains(const Aabb& aabb, const ImVec2 p) { return p.x >= aabb.min_x && p.x <= aabb.max_x && p.y >= aabb.min_y && p.y <= aabb.max_y; }
    
    double render_time_ms_{0};
    scenes::SceneObjects scene_objects_;
    
    // ===== Handles =====
    std::array<BodyHandle*, 4> boundaries_{&scene_objects_.ceiling, &scene_objects_.floor, &scene_objects_.left_wall, &scene_objects_.right_wall};
    std::array<BodyHandle*, 4> floatings_{ &scene_objects_.floating_circle, &scene_objects_.floating_circle2, &scene_objects_.floating_wide_box1, &scene_objects_.floating_wide_box2};

    // ===== Scene Bodies Setting =====
    float floating_bodies_angular_vel_{0.5f};
    float floating_bodies_restitution_{0.2f};
    float floating_bodies_friction_{0.5f};
    
    float boundary_restitution_{0.2f};
    float boundary_friction_{0.5f};
    
    // ===== Mouse Spawn Settings =====
    SpawnType left_click_spawn_type_{SpawnType::kRigidBox};
    SpawnType right_click_spawn_type_{SpawnType::kSoftCirclePressure};
    
    // Rigid Circle
    float circle_spawn_mass_{5.f};
    float circle_spawn_radius_{1.f};
    float circle_spawn_restitution_{0.2f};
    float circle_spawn_friction_{0.5f};
    
    // Rigid Box
    float box_spawn_mass_{5.f};
    float box_spawn_half_width_{1.f};
    float box_spawn_half_height_{1.f};
    float box_spawn_restitution_{0.2f};
    float box_spawn_friction_{0.3f};
    
    // Soft Rope
    bool fix_start_{true};
    bool fix_end_{false};
    bool is_horizontal_{false};
    float rope_spawn_length_{5.f};
    int rope_max_spawn_num_segments_{static_cast<int>(rope_spawn_length_ * physics_config::kParticleDensityLenght)};
    int rope_spawn_num_segments_{20};
    float rope_spawn_mass_{5.f};
    float rope_spawn_friction_{0.5f};
    float rope_spawn_compliance_{0.0005f};

    // Soft Box (Triangle Area)
    float soft_box_spawn_width_{2.f};
    float soft_box_spawn_height_{2.f};
    int soft_box_max_spawn_nx_{static_cast<int>(soft_box_spawn_width_ * physics_config::kParticleDensityLenght)};
    int soft_box_max_spawn_ny_{static_cast<int>(soft_box_spawn_height_ * physics_config::kParticleDensityLenght)};
    int soft_box_spawn_nx_{5};
    int soft_box_spawn_ny_{5};
    float soft_box_spawn_mass_{5.f};
    float soft_box_spawn_friction_{0.5f};
    float soft_box_spawn_compliance_{0.0005f};

    // Soft Circle (Triangle Area)
    float soft_circle_spawn_radius_{1.4f};
    int soft_circle_max_spawn_num_segments_{static_cast<int>(soft_circle_spawn_radius_ * physics_config::kParticleDensityRadius)};
    int soft_circle_spawn_num_segments_{20};
    float soft_circle_spawn_mass_{10.f};
    float soft_circle_spawn_friction_{0.5f};
    float soft_circle_spawn_compliance_{0.0005f};

    // Soft Circle (Pressure)
    float pressure_circle_spawn_radius_{1.4f};
    int pressure_circle_max_spawn_num_segments_{static_cast<int>(pressure_circle_spawn_radius_ * physics_config::kParticleDensityRadius)};
    int pressure_circle_spawn_num_segments_{20};
    float pressure_circle_spawn_mass_{10.f};
    float pressure_circle_spawn_friction_{0.5f};
    float pressure_circle_spawn_compliance_{0.f};
    
    // Car
    float wheel_radius_{0.7f};
    float wheel_mass_{20.f};
    float wheel_friction_{0.7f};
    float wheel_restitution_{0.05f};
    float suspension_height_{1.5f};
    float suspension_mass_{3.5f};
    float suspension_compliance_{0.0005f};
    float car_body_width_{3.f};
    float car_body_height_{1.f};
    float car_body_mass_{10.f};
    float car_body_restitution_{0.f};
    float car_body_friction_{0.5f};
    
    // ===== Others =====
    float arrow_key_force_{800.f};
    int bvh_draw_max_depth_{5};
    bool draw_bvh_{false};
    bool draw_particle_{false};
    bool draw_surface_particle_{false};
    bool draw_local_bvh_{false};
};
