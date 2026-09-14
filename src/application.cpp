#include <iostream>
#include <array>
#include <SDL3/SDL.h>
#include <imgui_impl_sdl3.h>
#include <imgui_impl_sdlrenderer3.h>

#include "Application.h"
#include "physics_config.h"
#include "physics/systems/collision_system.h"
#include "physics/systems/movement_system.h"
#include "physics/physics_world.h"

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
                float arrow_key_force = 500.f;
                switch (event.key.key) {
                case SDLK_ESCAPE:
                    is_running = false;
                    break;
                case SDLK_RIGHT:
                    world.ApplyForce({i}, {arrow_key_force, 0.f});
                    break;
                case SDLK_LEFT:
                    world.ApplyForce({i}, {-arrow_key_force, 0.f});
                    break;
                case SDLK_UP:
                    world.ApplyForce({i}, {0.f, arrow_key_force});
                    break;
                case SDLK_DOWN:
                    world.ApplyForce({i}, {0.f, -arrow_key_force});
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
                    1.f, {0.f, 0.f}, 0.f, 5.f, 1.f, 0.5f, 1.f
                );
            } 
            else if (event.button.button == SDL_BUTTON_RIGHT) 
            {
                world.CreateBox(
                    Vec2{ event.button.x, float_h - event.button.y } * physics_config::kMpp,
                    0.f, {0.f, 0.f}, 0.0f, 5.f, 1.f, 1.f, 0.f, 1.f
                );
            }
        }
        
        // Mouse motion event.
        if (!io.WantCaptureMouse && event.type == SDL_EVENT_MOUSE_MOTION)
        {
            // `event.motion.x, event.motion.y` are current positions.
            // `event.motion.xrel, event.motion.yrel` are relative move distance.
            // world.SetPosition({0}, Vec2{static_cast<float>(event.motion.x), float_h - event.motion.y} * physics_config::kMpp);
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
    
    int w, h;
    SDL_GetWindowSize(window, &w, &h);
    const float float_h = static_cast<float>(h);
    
    ImDrawList* draw_list = ImGui::GetBackgroundDrawList();

    const auto& positions = world.GetPositions();
    const auto& rotations = world.GetRotations();
    const auto& vertices = world.GetVertices();
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
        
        // Line collision_point_a to collision_point_a - normal * depth. 
        // Vec2 a_to_normal_depth = contact.collision_point_a - contact.normal * contact.depth;
        // ImVec2 pt_depth(a_to_normal_depth.x * physics_config::kPpm, float_h - a_to_normal_depth.y * physics_config::kPpm);
        // draw_list->AddLine(pt_a, pt_depth, IM_COL32(0, 0, 255, 255), 5.0f);
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
