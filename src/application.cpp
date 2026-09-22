#include <iostream>
#include <array>
#include <numbers>
#include <SDL3/SDL.h>
#include <imgui_impl_sdl3.h>
#include <imgui_impl_sdlrenderer3.h>

#include "Application.h"
#include "collision_detection_system.h"
#include "collision_solver_system.h"
#include "physics_config.h"
#include "movement_system.h"
#include "physics_world.h"

void Application::CreateInitialObjects(SDL_Window* window, PhysicsWorld& world)
{
    int w, h;
    SDL_GetWindowSize(window, &w, &h);
    
    // Small object for debug (check contact info).
    // debug_circle_ = world.CreateCircle(Vec2{0.f, 0.f}, 0.f, {0.f, 0.f}, 0.0f, 0.f, 1.f, 0.2f, 1.f);
    // debug_box_ = world.CreateBox(Vec2{0.f, 0.f} * physics_config::kMpp, 1.f, {0.f, 0.f}, 0.0f, 0.f, 1.f, 1.f, 1.f);

    // Floating obstacle objects.
    const Vec2 floating_circle_pos = Vec2{static_cast<float>(w) / 4.f * 3.f, static_cast<float>(h) / 2.f } * physics_config::kMpp;
    floating_circle_ = world.CreateCircle(floating_circle_pos, 0.0f,
        {0.f, 0.f}, floating_objects_angular_vel_,
        0.f, 2.f, floating_objects_restitution_, floating_objects_friction_);
    
    floating_circle2_ = world.CreateCircle({floating_circle_pos.x + 3, floating_circle_pos.y}, 0.0f,
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
    
    // Ceil, Floor, Left wall, Right wall.
    const Vec2 ceil_pos = Vec2{static_cast<float>(w) / 2.f, static_cast<float>(h) } * physics_config::kMpp;
    floor_ = world.CreateBox(ceil_pos, 0.f,
        {0.f, 0.f}, 0.0f,
        0.f, static_cast<float>(w) / 2 * physics_config::kMpp, 1.f, boundary_restitution_, boundary_friction_);
    
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
    // CollisionSystem::SolveCollision(world);
    CollisionDetectionSystem::CheckCollision(world);
    CollisionSolverSystem::SolveCollision(world);
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
    
    if (world.show_debug_contact_points_)
    {
        for (const auto& manifold : world.GetManifolds())
        {
            if (manifold.contact_count == 0) continue;
        
            for (size_t i = 0; i < manifold.contact_count; ++i)
            {
                ImVec2 pt_a(manifold.contacts[i].collision_point_a.x * physics_config::kPpm, float_h - manifold.contacts[i].collision_point_a.y * physics_config::kPpm);
                ImVec2 pt_b(manifold.contacts[i].collision_point_b.x * physics_config::kPpm, float_h - manifold.contacts[i].collision_point_b.y * physics_config::kPpm);
        
                // Line collision_point_a to collision_point_b.
                draw_list->AddLine(pt_a, pt_b, IM_COL32(255, 255, 0, 255), 5.0f);
            
                // Draw collision point a.
                draw_list->AddCircleFilled(pt_a, 5.0f, IM_COL32(0, 255, 0, 255));
            }
        }    
    }
    
    if (!world.GetBvhNodes().empty() && draw_bvh_) {
        RenderBvhDebug(world.GetBvhNodes(), 0, bvh_draw_max_depth_, float_h);
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
    
    ImGui::Text("Current FPS: %.1f\nNum Objects: %d, Num Manifolds: %d",
        ImGui::GetIO().Framerate, world.GetPositions().size(), world.GetManifolds().size());
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
    ImGui::Separator();
    
    ImGui::Text("Next Spawned Circle (Left Mouse Button):");
    ImGui::SliderFloat("Restitution##c", &circle_spawn_restitution_, 0.0f, 1.0f);
    ImGui::SliderFloat("friction##c", &circle_spawn_friction_, 0.0f, 1.0f);
    ImGui::SliderFloat("Mass##c", &circle_spawn_mass_, 0.0f, 20.0f);
    ImGui::SliderFloat("Radius##c", &circle_spawn_radius_, 0.5f, 3.0f);
    
    ImGui::Separator();
    
    ImGui::Text("Next Spawned Box (Right Mouse Button):");
    ImGui::SliderFloat("Restitution##b", &box_spawn_restitution_, 0.0f, 1.0f);
    ImGui::SliderFloat("friction##b", &box_spawn_friction_, 0.0f, 1.0f);
    ImGui::SliderFloat("Mass##b", &box_spawn_mass_, 0.0f, 20.0f);
    ImGui::SliderFloat("Half-Width##b", &box_spawn_half_width_, 0.1f, 3.0f);
    ImGui::SliderFloat("Half-Height##b", &box_spawn_half_height_, 0.1f, 3.0f);
    ImGui::Separator();
    
    ImGui::Text("Debug Options:");
    ImGui::Checkbox("Draw Contact Info", &world.show_debug_contact_points_);
    ImGui::Checkbox("Draw Incident Edge & Reference Edge", &world.show_debug_clipping_);
    ImGui::Text("BVH :");
    ImGui::Checkbox("Draw BVH", &draw_bvh_);
    ImGui::SliderInt("BVH Draw Depth", &bvh_draw_max_depth_, 0, 20);
    
    ImGui::End();
}

ImU32 Application::GetDepthColor(const int depth, const float alpha)
{
    const float hue = std::fmod(0.15f * static_cast<float>(depth), 1.0f); // Each depth has a different color.
    float r, g, b;
    ImGui::ColorConvertHSVtoRGB(hue, 0.8f, 0.9f, r, g, b);
    return ImColor(r, g, b, alpha);
}

void Application::DrawBvhNode(ImDrawList* draw_list, const std::vector<BvhNode>& nodes, const int node_idx, const int current_depth,
    const int max_render_depth, int& hovered_node_idx, const float window_height)
{
    if (node_idx >= static_cast<int>(nodes.size())) return;
    if (current_depth > max_render_depth) return;

    const BvhNode& node = nodes[node_idx];
    
    // Shrink by depth to avoid border overlap.
    const float padding = 1.5f * static_cast<float>(current_depth);
    const ImVec2 p_min = ImVec2(node.aabb.min_x * physics_config::kPpm + padding, window_height - node.aabb.min_y * physics_config::kPpm - padding);
    const ImVec2 p_max = ImVec2(node.aabb.max_x * physics_config::kPpm - padding, window_height - node.aabb.max_y * physics_config::kPpm + padding);
    
    // If the mouse is hovering over the current AABB.
    ImVec2 mouse_pos = ImGui::GetIO().MousePos;
    mouse_pos = {mouse_pos.x * physics_config::kMpp, (window_height - mouse_pos.y) * physics_config::kMpp};
    if (AabbContains(node.aabb, mouse_pos)) {
        hovered_node_idx = node_idx;
    }

    // Draw borders and fill with light colors
    const ImU32 stroke_color = GetDepthColor(current_depth, 0.8f);
    const ImU32 fill_color   = GetDepthColor(current_depth, 0.05f);
    
    draw_list->AddRectFilled(p_min, p_max, fill_color);
    draw_list->AddRect(p_min, p_max, stroke_color, 0.0f, 0, 1.5f);

    // Recursively draw subtree
    if (!node.IsLeaf()) {
        if (node.left_child != std::numeric_limits<size_t>::max())
        {
            DrawBvhNode(draw_list, nodes, node.left_child, current_depth + 1, max_render_depth, hovered_node_idx, window_height);
        }
        if (node.right_child != std::numeric_limits<size_t>::max())
        {
            DrawBvhNode(draw_list, nodes, node.right_child, current_depth + 1, max_render_depth, hovered_node_idx, window_height);    
        }
    }
}

void Application::RenderBvhDebug(const std::vector<BvhNode>& nodes, const int root_idx, const int max_render_depth, const float window_height)
{
    ImDrawList* draw_list = ImGui::GetBackgroundDrawList();
    
    int hovered_node_idx = -1;

    // Draw all AABB.
    DrawBvhNode(draw_list, nodes, root_idx, 0, max_render_depth, hovered_node_idx, window_height);

    // When the mouse hovers over an AABB, highlight and draw the children connection line.
    if (hovered_node_idx != -1) {
        const BvhNode& target = nodes[hovered_node_idx];
        ImVec2 target_center = GetAabbCenter(target.aabb);
        target_center = {target_center.x * physics_config::kPpm, window_height - target_center.y * physics_config::kPpm};

        // Highlight the AABB of the current hover (thick yellow border).
        draw_list->AddRect(
            ImVec2(target.aabb.min_x * physics_config::kPpm, window_height - target.aabb.min_y * physics_config::kPpm),
            ImVec2(target.aabb.max_x * physics_config::kPpm, window_height - target.aabb.max_y * physics_config::kPpm),
            IM_COL32(255, 255, 0, 255), 0.0f, 0, 3.0f
        );

        // Connect red line to the Left Child and mark it with a red dot.
        if (target.left_child != std::numeric_limits<size_t>::max()) {
            ImVec2 left_center = GetAabbCenter(nodes[target.left_child].aabb);
            left_center = {left_center.x * physics_config::kPpm, window_height - left_center.y * physics_config::kPpm};
            draw_list->AddLine(target_center, left_center, IM_COL32(255, 50, 50, 255), 2.0f);
            draw_list->AddCircleFilled(left_center, 4.0f, IM_COL32(255, 50, 50, 255));
        }

        // Right Child blue.
        if (target.right_child != std::numeric_limits<size_t>::max()) {
            ImVec2 right_center = GetAabbCenter(nodes[target.right_child].aabb);
            right_center = {right_center.x * physics_config::kPpm, window_height - right_center.y * physics_config::kPpm};
            draw_list->AddLine(target_center, right_center, IM_COL32(50, 150, 255, 255), 2.0f);
            draw_list->AddCircleFilled(right_center, 4.0f, IM_COL32(50, 150, 255, 255));
        }
    }
}
