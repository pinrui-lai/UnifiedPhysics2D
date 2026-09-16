#include <iostream>
#include <array>
#include <numbers>
#include <SDL3/SDL.h>
#include <imgui_impl_sdl3.h>
#include <imgui_impl_sdlrenderer3.h>

#include "Application.h"
#include "physics_config.h"
#include "physics/systems/collision_system.h"
#include "physics/systems/movement_system.h"
#include "physics/physics_world.h"

void Application::CreateInitialObjects(SDL_Window* window, PhysicsWorld& world)
{
    int w, h;
    SDL_GetWindowSize(window, &w, &h);
    
    // Small object for debug (check contact info).
    // debug_circle_ = world.CreateCircle(Vec2{0.f, 0.f}, 0.f, {0.f, 0.f}, 0.0f, 0.f, 1.f, 0.2f, 1.f);
    // debug_box_ = world.CreateBox(Vec2{0, 0 } * physics_config::kMpp, 1.f, {0.f, 0.f}, 0.0f, 0.f, 1.f, 1.f, 1.f);

    // Floating obstacle objects.
    const Vec2 floating_circle_pos = Vec2{static_cast<float>(w) / 4.f * 3.f, static_cast<float>(h) / 2.f } * physics_config::kMpp;
    floating_circle_ = world.CreateCircle(floating_circle_pos, 0.0f,
        {0.f, 0.f}, floating_objects_angular_vel_,
        0.f, 2.f, floating_objects_restitution_, floating_objects_friction_);
    
    const Vec2 floating_box_pos = Vec2{static_cast<float>(w) / 4.f, static_cast<float>(h) / 2.f } * physics_config::kMpp;
    floating_box_ = world.CreateBox(floating_box_pos, 0.0f,
        {0.f, 0.f}, floating_objects_angular_vel_,
        0.f, 2.5f, 2.5f, floating_objects_restitution_, floating_objects_friction_);
    
    const Vec2 floating_wide_box_pos = Vec2{static_cast<float>(w) / 2.f, static_cast<float>(h) / 2.f } * physics_config::kMpp;
    floating_wide_box_1_ = world.CreateBox(floating_wide_box_pos, 0.0f,
        {0.f, 0.f}, floating_objects_angular_vel_,
        0.f, 3.5f, 0.4f, floating_objects_restitution_, floating_objects_friction_);
    
    floating_wide_box_2_ = world.CreateBox(floating_wide_box_pos, std::numbers::pi / 2,
        {0.f, 0.f}, floating_objects_angular_vel_,
        0.f, 3.5f, 0.4f, floating_objects_restitution_, floating_objects_friction_);
    
    // Floor, Left wall, Right wall.
    const Vec2 floor_pos = Vec2{static_cast<float>(w) / 2.f, 50.f } * physics_config::kMpp;
    floor_ = world.CreateBox(floor_pos, 0.f,
        {0.f, 0.f}, 0.0f,
        0.f, static_cast<float>(w) / 2 * physics_config::kMpp, 1.f, boundary_restitution_, boundary_friction_);
    
    const Vec2 left_all_pos = Vec2{50, static_cast<float>(h) / 2.f } * physics_config::kMpp;
    left_wall_ = world.CreateBox(left_all_pos, 0.f,
        {0.f, 0.f}, 0.0f,
        0.f, 1.f, static_cast<float>(h) / 2 * physics_config::kMpp, boundary_restitution_, boundary_friction_);
    
    const Vec2 right_wall_pos = Vec2{static_cast<float>(w) - 50, static_cast<float>(h) / 2.f } * physics_config::kMpp;
    right_wall_ = world.CreateBox( right_wall_pos, 0.f,
        {0.f, 0.f}, 0.0f,
        0.f, 1.f, static_cast<float>(h) / 2 * physics_config::kMpp, boundary_restitution_, boundary_friction_);
}

void Application::Input(SDL_Window* window, bool& is_running, PhysicsWorld& world)
{
    int w, h;
    SDL_GetWindowSize(window, &w, &h);
    const float float_h = static_cast<float>(h);

    const ImGuiIO& io = ImGui::GetIO();
    SDL_Event event;
    
    while (SDL_PollEvent(&event))
    {
        ImGui_ImplSDL3_ProcessEvent(&event);
        
        if (event.type == SDL_EVENT_QUIT) {
            is_running = false;
            break;
        }
        
        // Physical logic is only executed if ImGui does not capture input.
        // Keyboard event.
        if (!io.WantCaptureKeyboard && event.type == SDL_EVENT_KEY_DOWN)
        {
            for (size_t i = 0; i < world.GetPositions().size(); i++)
            {
                switch (event.key.key) {
                case SDLK_ESCAPE:
                    is_running = false;
                    break;
                case SDLK_RIGHT:
                    world.ApplyForce({i}, {arrow_key_force_, 0.f});
                    break;
                case SDLK_LEFT:
                    world.ApplyForce({i}, {-arrow_key_force_, 0.f});
                    break;
                case SDLK_UP:
                    world.ApplyForce({i}, {0.f, arrow_key_force_});
                    break;
                case SDLK_DOWN:
                    world.ApplyForce({i}, {0.f, -arrow_key_force_});
                    break;
                default: break;
                }
            }
            
        }
        
        // Mouse button event.
        if (!io.WantCaptureMouse && event.type == SDL_EVENT_MOUSE_BUTTON_DOWN)
        {
            if (event.button.button == SDL_BUTTON_LEFT)
            {
                world.CreateCircle(
                    Vec2{ event.button.x, float_h - event.button.y } * physics_config::kMpp,
                    1.f, {0.f, 0.f}, 0.f, 
                    circle_spawn_mass_, circle_spawn_radius_, 
                    circle_spawn_restitution_, circle_spawn_friction_
                );
            } 
            else if (event.button.button == SDL_BUTTON_RIGHT) 
            {
                world.CreateBox(
                    Vec2{ event.button.x, float_h - event.button.y } * physics_config::kMpp,
                    0.f, {0.f, 0.f}, 0.0f,
                    box_spawn_mass_, box_spawn_half_width_, box_spawn_half_height_,
                    box_spawn_restitution_, box_spawn_friction_
                );
            }
        }
        
        // Mouse motion event.
        if (!io.WantCaptureMouse && event.type == SDL_EVENT_MOUSE_MOTION)
        {
            if (debug_box_.IsValid())
            {
                world.SetPosition({debug_box_}, Vec2{(event.motion.x), float_h - event.motion.y} * physics_config::kMpp);    
            }
            if (debug_circle_.IsValid())
            {
                world.SetPosition({debug_circle_}, Vec2{(event.motion.x), float_h - event.motion.y} * physics_config::kMpp);    
            }
        }
        
        // Mouse wheel event.
        if (!io.WantCaptureMouse && event.type == SDL_EVENT_MOUSE_WHEEL)
        {
            if (event.wheel.y > 0.0f) {
                std::cout << "Scroll up. ";
            } else if (event.wheel.y < 0.0f) {
                std::cout << "Scroll down. ";
            }
        }
    }
}

void Application::Update(PhysicsWorld& world, const float dt)
{
    MovementSystem::ApplyGravity(world);
    MovementSystem::IntegrateVelocity(world, dt);
    CollisionSystem::SolveCollision(world);
    MovementSystem::IntegratePosition(world, dt);
}

namespace 
{
    template<class... Ts> struct Overload : Ts... { using Ts::operator()...; };    
}

void Application::Render(SDL_Renderer* renderer, SDL_Window* window, PhysicsWorld& world)
{
    // ImGui New Frame.
    ImGui_ImplSDLRenderer3_NewFrame();
    ImGui_ImplSDL3_NewFrame();
    ImGui::NewFrame();
    
    RenderPhysicsControlPanel(world);
    
    int w, h;
    SDL_GetWindowSize(window, &w, &h);
    const float float_h = static_cast<float>(h);
    
    ImDrawList* draw_list = ImGui::GetBackgroundDrawList();

    const auto& positions = world.GetPositions();
    const auto& rotations = world.GetRotations();
    // const auto& vertices = world.GetVertices();
    const auto& shapes = world.GetShapes();
    
    for (int i = 0; i < shapes.size(); i++)
    {
        std::visit(Overload {
        [&](const CircleShape& c)
        {
            const ImVec2 center(positions[i].val.x * physics_config::kPpm, float_h - positions[i].val.y * physics_config::kPpm);
            const float radius = c.radius * physics_config::kPpm;
            
            draw_list->AddCircle(center, radius, IM_COL32(255, 0, 0, 255), 0, 2.f);
            
            // Draw a rotation indicator line (line from center to circumference).
            const float rot = -rotations[i].val;
            const ImVec2 rad_line(center.x + radius * std::cos(rot), center.y + radius * std::sin(rot));
            draw_list->AddLine(center, rad_line, IM_COL32(255, 0, 0, 255), 1.5f);
        },
        [&](const BoxShape& b)
        {
            const std::array<Vec2, 4> local_vertices{{
                    {-b.half_width, b.half_height}, 
                    {b.half_width, b.half_height}, 
                    {b.half_width, -b.half_height}, 
                    {-b.half_width, -b.half_height}
                }};
            
            std::array<ImVec2, 4> draw_vertices;
                const float cos_r = std::cos(rotations[i].val);
                const float sin_r = std::sin(rotations[i].val);

                for (size_t idx = 0; idx < 4; ++idx) {
                    const Vec2 v = local_vertices[idx];
                    const float wx = (v.x * cos_r - v.y * sin_r + positions[i].val.x) * physics_config::kPpm;
                    const float wy = float_h - (v.x * sin_r + v.y * cos_r + positions[i].val.y) * physics_config::kPpm;
                    draw_vertices[idx] = ImVec2(wx, wy);
                }

                draw_list->AddPolyline(draw_vertices.data(), 4, IM_COL32(255, 0, 0, 255), ImDrawFlags_Closed, 2.0f);
        },
        [](const PolygonShape& p) {  }
            }, shapes[i]);
    }
    
    for (const auto& contact : world.debug_contacts_)
    {
        ImVec2 pt_a(contact.collision_point_a.x * physics_config::kPpm, float_h - contact.collision_point_a.y * physics_config::kPpm);
        ImVec2 pt_b(contact.collision_point_b.x * physics_config::kPpm, float_h - contact.collision_point_b.y * physics_config::kPpm);
        
        // Line collision_point_a to collision_point_b.
        draw_list->AddLine(pt_a, pt_b, IM_COL32(255, 255, 0, 255), 5.0f);
     
        // Draw collision point a.
        draw_list->AddCircleFilled(pt_a, 5.0f, IM_COL32(0, 255, 0, 255));
    }
    
    for (const auto& incident_edge : world.debug_incident_edges_)
    {
        ImVec2 p0(incident_edge[0].x * physics_config::kPpm, float_h - incident_edge[0].y * physics_config::kPpm);
        ImVec2 p1(incident_edge[1].x * physics_config::kPpm, float_h - incident_edge[1].y * physics_config::kPpm);
        draw_list->AddLine(p0, p1, IM_COL32(255, 0, 255, 255), 5.0f);
    }
    
    for (const auto& reference_edge : world.debug_reference_edges_)
    {
        ImVec2 p0(reference_edge[0].x * physics_config::kPpm, float_h - reference_edge[0].y * physics_config::kPpm);
        ImVec2 p1(reference_edge[1].x * physics_config::kPpm, float_h - reference_edge[1].y * physics_config::kPpm);
        draw_list->AddLine(p0, p1, IM_COL32(0, 255, 255, 255), 5.0f);
    }
    
    ImGui::Render();
    SDL_SetRenderDrawColor(renderer, 20, 20, 20, 255);
    SDL_RenderClear(renderer);
    
    ImGui_ImplSDLRenderer3_RenderDrawData(ImGui::GetDrawData(), renderer);
    SDL_RenderPresent(renderer);
}

void Application::RenderPhysicsControlPanel(PhysicsWorld& world)
{
    ImGui::Begin("Physics Engine Control");
    
    ImGui::Text("Current FPS: %.1f", ImGui::GetIO().Framerate);
    ImGui::Separator();
    
    ImGui::Text("Boundaries:");
    if (ImGui::SliderFloat("Restitution##bound", &boundary_restitution_, 0.0f, 1.0f))
    {
        for (const auto& obj : boundaries_)
        {
            if (!obj->IsValid()) continue;
            world.SetRestitution(*obj, boundary_restitution_);
        }
    }
    if (ImGui::SliderFloat("Friction##bound", &boundary_friction_, 0.0f, 1.0f))
    {
        for (const auto& obj : boundaries_)
        {
            if (!obj->IsValid()) continue;
            world.SetFriction(*obj, boundary_friction_);
        }
    }
    ImGui::Separator();
    
    ImGui::Text("Floating Obstacles:");
    if (ImGui::Button("Reset Angular Velocity##floating"))
    {
        floating_objects_angular_vel_ = 0.f;
        for (const auto& obj : floatings_)
        {
            if (!obj->IsValid()) continue;
            world.SetAngularVelocity(*obj, 0.f);
        }
    }
    if (ImGui::SliderFloat("Angular Velocity##floating", &floating_objects_angular_vel_, -6.0f, 6.0f))
    {
        for (const auto& obj : floatings_)
        {
            if (!obj->IsValid()) continue;
            world.SetAngularVelocity(*obj, floating_objects_angular_vel_);
        }
    }
    if (ImGui::SliderFloat("Restitution##floating", &floating_objects_restitution_, 0.0f, 1.0f))
    {
        for (const auto& obj : floatings_)
        {
            if (!obj->IsValid()) continue;
            world.SetRestitution(*obj, floating_objects_restitution_);
        }
    }
    if (ImGui::SliderFloat("Friction##floating", &floating_objects_friction_, 0.0f, 1.0f))
    {
        for (const auto& obj : floatings_)
        {
            if (!obj->IsValid()) continue;
            world.SetFriction(*obj, floating_objects_friction_);
        }
    }
    ImGui::Separator();
    
    ImGui::Text("Next Spawned Circle (Left Mouse Button):");
    if (ImGui::SliderFloat("Mass##c", &circle_spawn_mass_, 0.0f, 20.0f)) {}
    if (ImGui::SliderFloat("Radius##c", &circle_spawn_radius_, 0.5f, 3.0f)) {}
    if (ImGui::SliderFloat("Restitution##c", &circle_spawn_restitution_, 0.0f, 1.0f)) {}
    if (ImGui::SliderFloat("friction##c", &circle_spawn_friction_, 0.0f, 1.0f)) {}
    ImGui::Separator();
    
    ImGui::Text("Next Spawned Box (Right Mouse Button):");
    if (ImGui::SliderFloat("Mass##b", &box_spawn_mass_, 0.0f, 20.0f)) {}
    if (ImGui::SliderFloat("Half-Width##b", &box_spawn_half_width_, 0.1f, 3.0f)) {}
    if (ImGui::SliderFloat("Half-Height##b", &box_spawn_half_height_, 0.1f, 3.0f)) {}
    if (ImGui::SliderFloat("Restitution##b", &box_spawn_restitution_, 0.0f, 1.0f)) {}
    if (ImGui::SliderFloat("friction##b", &box_spawn_friction_, 0.0f, 1.0f)) {}
    ImGui::Separator();
    
    ImGui::Text("Debug Options:");
    ImGui::Checkbox("Draw Contact Info", &world.show_debug_contact_points_);
    ImGui::Checkbox("Draw Incident Edge & Reference Edge", &world.show_debug_clipping_);
    
    ImGui::End();
}
