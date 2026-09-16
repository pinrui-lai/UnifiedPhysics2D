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
    
private:
    BodyHandle floating_circle_{0xFFFFFFFF};
    BodyHandle floating_box_{0xFFFFFFFF};
    BodyHandle floating_wide_box_1_{0xFFFFFFFF};
    BodyHandle floating_wide_box_2_{0xFFFFFFFF};
    BodyHandle floor_{0xFFFFFFFF};
    BodyHandle left_wall_{0xFFFFFFFF};
    BodyHandle right_wall_{0xFFFFFFFF};
    BodyHandle debug_circle_{0xFFFFFFFF};
    BodyHandle debug_box_{0xFFFFFFFF};
    
    std::array<BodyHandle*, 3> boundaries_{ &floor_, &left_wall_, &right_wall_ };
    std::array<BodyHandle*, 4> floatings_{ &floating_circle_, &floating_box_, &floating_wide_box_1_, &floating_wide_box_2_};
    
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
};
