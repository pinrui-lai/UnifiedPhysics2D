#include <iostream>
#include <ranges>
#include <array>
#include <SDL2_gfxPrimitives.h>

#include "Application.h"
#include "graphics.h"
#include "physics_config.h"
#include "physics/systems/collision_system.h"
#include "physics/systems/movement_system.h"
#include "physics/physics_world.h"

void Application::Input(SDL_Event& event, SDL_Window* window, bool& is_running, PhysicsWorld& world)
{
    int w, h;
    SDL_GetWindowSize(window, &w, &h);
    float float_h = static_cast<float>(h);
    
    while (SDL_PollEvent(&event))
    {
        float arrow_key_force = 500.f;
        switch (event.type) {
            case SDL_QUIT:
                is_running = false;
                break;
            
            /** Keyboard Down */
            case SDL_KEYDOWN:
                if (event.key.keysym.sym == SDLK_ESCAPE)
                {
                    is_running = false;
                }
                else if (event.key.keysym.sym == SDLK_RIGHT)
                {
                    world.ApplyForce({0}, {arrow_key_force, 0.f});
                }
                else if (event.key.keysym.sym == SDLK_LEFT)
                {
                    world.ApplyForce({0}, {-arrow_key_force, 0.f});
                }
                else if (event.key.keysym.sym == SDLK_UP)
                {
                    world.ApplyForce({0}, {0.f, arrow_key_force});
                }
                else if (event.key.keysym.sym == SDLK_DOWN)
                {
                    world.ApplyForce({0}, {0.f, -arrow_key_force});
                }
                break;
            
            /** Mouse Motion */
            case SDL_MOUSEMOTION:
                // `event.motion.x, event.motion.y` are current positions.
                // `event.motion.xrel, event.motion.yrel` are relative move distance.
                // world.SetPosition({0}, Vec2{static_cast<float>(event.motion.x), float_h - event.motion.y} * physics_config::kMpp);
                break;
            
            /** Mouse Down or Up*/
            case SDL_MOUSEBUTTONDOWN:
                if (event.button.button == SDL_BUTTON_LEFT)
                {
                    world.CreateCircle(Vec2{static_cast<float>(event.button.x), float_h - event.button.y } * physics_config::kMpp
                , 1.f, {0.f, 0.f}, 0.f, 5.f, 1.f, 0.5f, 1.f);
                    
                } else if (event.button.button == SDL_BUTTON_RIGHT) {
                    world.CreateBox(Vec2{static_cast<float>(event.button.x), float_h - event.button.y } * physics_config::kMpp
                        , 0.f, {0.f, 0.f}, 0.0f, 5.f, 1.f, 1.f, 0.f, 1.f);
                }
                break;

            case SDL_MOUSEBUTTONUP:
                break;
            
            /** Mouse Wheel */
            case SDL_MOUSEWHEEL:
                if (event.wheel.y > 0) {
                    std::cout << "Scroll up. ";
                } else if (event.wheel.y < 0) {
                    std::cout << "Scroll down. ";
                }
                break;
            
            default: ;
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
    // Render screen to dark gray.
    SDL_SetRenderDrawColor(renderer, 20, 20, 20, 255);
    SDL_RenderClear(renderer);
    
    const auto& positions = world.GetPositions();
    const auto& rotations = world.GetRotations();
    const auto& vertices = world.GetVertices();
    const auto& shapes = world.GetShapes();
    
    int w, h;
    SDL_GetWindowSize(window, &w, &h);
    const float float_h = static_cast<float>(h);
    
    for (int i = 0; i < shapes.size(); i++)
    {
        std::visit(Overload {
        [&](const CircleShape& c) { Graphics::DrawRotatedCircle(renderer, Vec2{positions[i].val.x * physics_config::kPpm, float_h - positions[i].val.y * physics_config::kPpm}, c.radius * physics_config::kPpm, -rotations[i].val, 255, 0, 0, 255); },
        [&](const BoxShape& b)
        {
            std::vector<Vec2> local_vertices{{-b.half_width, b.half_height}, {b.half_width, b.half_height}, {b.half_width, -b.half_height}, {-b.half_width, -b.half_height}};
            
            // Transform to world coordinate and scale by PPM.
            auto world_vertices_render_view = local_vertices | std::views::transform([rotations, i, positions, h](Vec2 v)
            {
                return Vec2{(v.x * std::cos(rotations[i].val) - v.y * std::sin(rotations[i].val) + positions[i].val.x) * physics_config::kPpm, 
                h - (v.x * std::sin(rotations[i].val) + v.y * std::cos(rotations[i].val) + positions[i].val.y) * physics_config::kPpm};
            });
            
            std::vector<Vec2> world_vertices(world_vertices_render_view.begin(), world_vertices_render_view.end());
            
            Graphics::DrawPolygon(renderer, world_vertices, 255, 0, 0, 255);
            
        },
        [](const PolygonShape& p) {  }
            }, shapes[i]);
    }
    
    for (const auto& contact : world.debug_contacts_)
    {
        // Line collision_point_a to collision_point_b.
        thickLineRGBA(renderer, contact.collision_point_a.x * physics_config::kPpm, float_h - contact.collision_point_a.y * physics_config::kPpm, contact.collision_point_b.x * physics_config::kPpm, float_h - contact.collision_point_b.y * physics_config::kPpm, 5, 255, 255, 0, 255);
     
        // Line collision_point_a to collision_point_a + normal * depth. 
        Vec2 b = contact.collision_point_a - contact.normal * contact.depth;
        thickLineRGBA(renderer, contact.collision_point_a.x * physics_config::kPpm, float_h - contact.collision_point_a.y * physics_config::kPpm, b.x * physics_config::kPpm, float_h - b.y * physics_config::kPpm, 5, 0, 0, 255, 255);
        
        // Draw collision point a.
        filledCircleRGBA(renderer, contact.collision_point_a.x * physics_config::kPpm, float_h - contact.collision_point_a.y * physics_config::kPpm, 5, 0, 255, 0, 255);
    }
    
    for (const auto& incident_edge : world.debug_incident_edges_)
    {
        thickLineRGBA(renderer, incident_edge[0].x * physics_config::kPpm, float_h - incident_edge[0].y * physics_config::kPpm, incident_edge[1].x * physics_config::kPpm, float_h - incident_edge[1].y * physics_config::kPpm, 5, 255, 0, 255, 255);
    }
    
    for (const auto& reference_edge : world.debug_reference_edges_)
    {
        thickLineRGBA(renderer, reference_edge[0].x * physics_config::kPpm, float_h - reference_edge[0].y * physics_config::kPpm, reference_edge[1].x * physics_config::kPpm, float_h - reference_edge[1].y * physics_config::kPpm, 5, 0, 255, 255, 255);
    }
    
    SDL_RenderPresent(renderer);
}
