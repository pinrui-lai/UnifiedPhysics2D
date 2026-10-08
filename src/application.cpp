#include <array>
#include <numbers>
#include <SDL3/SDL.h>
#include <imgui_impl_sdl3.h>
#include <imgui_impl_sdlrenderer3.h>

#include "Application.h"
#include "physics_config.h"
#include "physics_world.h"

void Application::CreateInitialBodies(SDL_Window* window, PhysicsWorld& world)
{
    int w, h;
    SDL_GetWindowSize(window, &w, &h);
    
    // Small body for debug (check contact info).
    // debug_circle_ = world.CreateCircle(Vec2{0.f, 0.f}, 0.f, {0.f, 0.f}, 0.0f, 0.f, 1.f, 0.2f, 1.f);
    // debug_box_ = world.CreateBox(Vec2{0.f, 0.f} * physics_config::kMpp, 1.f, {0.f, 0.f}, 0.0f, 0.f, 1.f, 1.f, 1.f);

    // XPBD soft rope.
    Vec2 rope_start_pos = Vec2{static_cast<float>(w) / 2.f, static_cast<float>(h) / 2.f} * physics_config::kMpp;
    rope_start_pos.x -= 9.f;
    rope_start_pos.y += 5.f;
    const Vec2 rope_end_pos{rope_start_pos.x + 12.f, rope_start_pos.y};
    world.CreateRope(rope_start_pos, rope_end_pos, physics_config::kSubBvhThreshold + 1,  20, 0.5f, 0.00001f, true, true);
    
    // Floating obstacle Bodies.
    const Vec2 floating_circle_pos = Vec2{static_cast<float>(w) - 700.f, static_cast<float>(h) / 2.f } * physics_config::kMpp;
    floating_circle_ = world.CreateCircle(floating_circle_pos, 0.0f,
        {0.f, 0.f}, floating_bodies_angular_vel_,
        0.f, 1.75f, floating_bodies_restitution_, floating_bodies_friction_);
    
    floating_circle2_ = world.CreateCircle({floating_circle_pos.x + 2.5f, floating_circle_pos.y}, 0.0f,
        {0.f, 0.f}, floating_bodies_angular_vel_,
        0.f, 1.75f, floating_bodies_restitution_, floating_bodies_friction_);
    
    const Vec2 floating_wide_box_pos = Vec2{400.f, static_cast<float>(h) / 2.f } * physics_config::kMpp;
    floating_wide_box_1_ = world.CreateBox(floating_wide_box_pos, 0.0f,
        {0.f, 0.f}, floating_bodies_angular_vel_,
        0.f, 3.2f, 0.3f, floating_bodies_restitution_, floating_bodies_friction_);
    
    floating_wide_box_2_ = world.CreateBox(floating_wide_box_pos, static_cast<float>(std::numbers::pi / 2),
        {0.f, 0.f}, floating_bodies_angular_vel_,
        0.f, 3.2f, 0.3f, floating_bodies_restitution_, floating_bodies_friction_);
    
    // Ceil, Floor, Left wall, Right wall.
    const Vec2 ceil_pos = Vec2{static_cast<float>(w) / 2.f, static_cast<float>(h) + 15.f } * physics_config::kMpp;
    ceil_ = world.CreateBox(ceil_pos, 0.f,
        {0.f, 0.f}, 0.0f,
        0.f, static_cast<float>(w) / 2 * physics_config::kMpp, 1.f, boundary_restitution_, boundary_friction_);
    
    const Vec2 floor_pos = Vec2{static_cast<float>(w) / 2.f, 0.f } * physics_config::kMpp;
    floor_ = world.CreateBox(floor_pos, 0.f,
        {0.f, 0.f}, 0.0f,
        0.f, static_cast<float>(w) / 2 * physics_config::kMpp, 1.f, boundary_restitution_, boundary_friction_);
    
    const Vec2 left_all_pos = Vec2{10.f, static_cast<float>(h) / 2.f } * physics_config::kMpp;
    left_wall_ = world.CreateBox(left_all_pos, 0.f,
        {0.f, 0.f}, 0.0f,
        0.f, 1.f, static_cast<float>(h) / 2 * physics_config::kMpp, boundary_restitution_, boundary_friction_);
    
    const Vec2 right_wall_pos = Vec2{static_cast<float>(w) - 10.f, static_cast<float>(h) / 2.f } * physics_config::kMpp;
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
        // ===== Keyboard event =====
        if (!io.WantCaptureKeyboard && event.type == SDL_EVENT_KEY_DOWN)
        {
            switch (event.key.key) {
            case SDLK_ESCAPE:
                is_running = false;
                break;
            case SDLK_RIGHT:
                world.ApplyForceToAll({arrow_key_force_, 0.f});
                break;
            case SDLK_LEFT:
                world.ApplyForceToAll({-arrow_key_force_, 0.f});
                break;
            case SDLK_UP:
                world.ApplyForceToAll({0.f, arrow_key_force_});
                break;
            case SDLK_DOWN:
                world.ApplyForceToAll({0.f, -arrow_key_force_});
                break;
            default: break;
            }
        }
        
        // ===== Mouse button event =====
        if (!io.WantCaptureMouse && event.type == SDL_EVENT_MOUSE_BUTTON_DOWN)
        {
            const Vec2 mouse_pos = Vec2{ event.button.x, float_h - event.button.y } * physics_config::kMpp;
            if (event.button.button == SDL_BUTTON_LEFT)
            {
                SpawnBodyAt(world, left_click_spawn_type_, mouse_pos);
            } 
            else if (event.button.button == SDL_BUTTON_RIGHT) 
            {
                SpawnBodyAt(world, right_click_spawn_type_, mouse_pos);
            }
        }
        
        // ===== Mouse motion event =====
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
    }
}

void Application::Update(PhysicsWorld& world, const float dt)
{
    world.Step(dt);
}

namespace 
{
    template<class... Ts> struct Overload : Ts... { using Ts::operator()...; };    
}

void Application::Render(SDL_Renderer* renderer, SDL_Window* window, PhysicsWorld& world)
{
    using Clock = std::chrono::high_resolution_clock;
    using Duration = std::chrono::duration<double, std::milli>;
    const auto start = Clock::now();
    
    // ImGui New Frame.
    ImGui_ImplSDLRenderer3_NewFrame();
    ImGui_ImplSDL3_NewFrame();
    ImGui::NewFrame();
    
    int w, h;
    SDL_GetWindowSize(window, &w, &h);
    const float float_h = static_cast<float>(h);
    
    ImDrawList* draw_list = ImGui::GetBackgroundDrawList();

    const auto& positions = world.GetPositions();
    const auto& rots = world.GetRots();
    // const auto& vertices = world.GetVertices();
    const auto& shapes = world.GetShapes();
    const auto& particle_positions = world.GetParticlePositions();
    const auto& surface_particles_idx = world.GetSurfaceParticlesIdx();
    
    // ==== Rigid Body ====
    for (int i = 0; i < shapes.size(); i++)
    {
        std::visit(Overload {
        [&](const CircleShape& c)
        {
            const ImVec2 center(positions[i].val.x * physics_config::kPpm, float_h - positions[i].val.y * physics_config::kPpm);
            const float radius = c.radius * physics_config::kPpm;
            
            draw_list->AddCircle(center, radius, IM_COL32(255, 0, 0, 255), 0, 2.f);
            
            // Draw a rotation indicator line (line from center to circumference).
            const ImVec2 rad_line(center.x + radius * rots[i].c, center.y + radius * (-rots[i].s));
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
                for (uint8_t idx = 0; idx < 4; ++idx) {
                    const Vec2 v = local_vertices[idx];
                    const float wx = (v.x * rots[i].c - v.y * rots[i].s + positions[i].val.x) * physics_config::kPpm;
                    const float wy = float_h - (v.x * rots[i].s + v.y * rots[i].c + positions[i].val.y) * physics_config::kPpm;
                    draw_vertices[idx] = ImVec2(wx, wy);
                }

                draw_list->AddPolyline(draw_vertices.data(), 4, IM_COL32(255, 0, 0, 255), ImDrawFlags_Closed, 2.0f);
        },
        [](const PolygonShape& p) {  }
            }, shapes[i]);
    }
    
    // ==== Soft Body ====
    for (const auto& particle_position : particle_positions)
    {
        ImVec2 pos{particle_position.x * physics_config::kPpm, float_h - particle_position.y * physics_config::kPpm};
        draw_list->AddCircleFilled(pos, 5.0f, IM_COL32(255, 0, 0, 255));
    }
    
    for (const auto& constraint : world.GetDistanceConstraints())
    {
        ImVec2 p0(particle_positions[constraint.p1].x * physics_config::kPpm, float_h - particle_positions[constraint.p1].y * physics_config::kPpm);
        ImVec2 p1(particle_positions[constraint.p2].x * physics_config::kPpm, float_h - particle_positions[constraint.p2].y * physics_config::kPpm);
        draw_list->AddLine(p0, p1, IM_COL32(255, 0, 0, 255), 2.0f);
    }
    
    if (draw_surface_particle_)
    {
        for (const auto& idx : surface_particles_idx)
        {
            ImVec2 pos{particle_positions[idx].x * physics_config::kPpm, float_h - particle_positions[idx].y * physics_config::kPpm};
            draw_list->AddCircleFilled(pos, 5.0f, IM_COL32(255, 127, 0, 255));
        }
    }
    
    // ==== Debug Option ====
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
        
            for (uint8_t i = 0; i < manifold.contact_count; ++i)
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
    
    if (draw_first_rope_bvh_) {
        RenderBvhDebug(world.GetInitRopeBvhNodes(), 0, bvh_draw_max_depth_, float_h);
    }
    
    RenderPhysicsControlPanel(window, world, w, Duration(Clock::now() - start).count());
    
    ImGui::Render();
    SDL_SetRenderDrawColor(renderer, 20, 20, 20, 255);
    SDL_RenderClear(renderer);
    
    ImGui_ImplSDLRenderer3_RenderDrawData(ImGui::GetDrawData(), renderer);
    SDL_RenderPresent(renderer);
}

void Application::RenderPhysicsControlPanel(SDL_Window* window, PhysicsWorld& world, const int window_width, const double render_time_ms)
{
    ImGui::SetNextWindowPos(ImVec2(static_cast<float>(window_width) - 415.f, 10), ImGuiCond_Appearing);
    ImGui::SetNextWindowSize(ImVec2(415, 1060), ImGuiCond_Appearing);
    ImGui::SetNextWindowCollapsed(false, ImGuiCond_Appearing);
    
    ImGui::Begin("Physics Engine Control");

    ImGui::Text("FPS: %.1f, Render: %.5f ms\nNum Bodies: %d, Num Manifolds: %d\nNum Particle-Rigid Contacts: %d\nNum Particle-Edge Contacts: %d",
        ImGui::GetIO().Framerate, render_time_ms, world.GetPositions().size() + world.GetSoftBodyHandles().size(), world.GetManifolds().size(), world.GetParticleRigidContactsSize(), world.GetParticleEdgeContactsSize());
    
    ImGui::Spacing();
    ImGui::Text("Integrate Rigid Vel: %.5f ms\nPredict Particle Pos: %.5f ms",
        world.profile_data_.integrate_rigid_vel, world.profile_data_.predict_particle_pos);
    
    ImGui::Spacing();
    ImGui::Text("Broad-Phase: %.5f ms\n  Build Bvh: %.5f ms\n  Query Bvh: %.5f ms\n  Erase Manifolds: %.5f ms",
        world.profile_data_.broad_phase_ms, world.profile_data_.build_bvh_ms, world.profile_data_.query_bvh_ms, world.profile_data_.erase_manifold_ms);
    
    ImGui::Spacing();
    ImGui::Text("Narrow-Phase: %.5f ms\n  Rigid - Rigid: %.5f ms\n  Rigid - Soft: %.5f ms\n  Soft - Soft: %.5f ms",
        world.profile_data_.narrow_phase_ms, world.profile_data_.rigid_rigid_ms, world.profile_data_.rigid_soft_ms, world.profile_data_.soft_soft_ms);
    
    ImGui::Spacing();
    ImGui::Text("Solve XPBD Constraint: %.5f ms\nSolve Sequential Impulse Constraint: %.5f ms",
        world.profile_data_.solve_xpbd_constraints_ms, world.profile_data_.solve_si_constraints_ms);
    
    ImGui::Spacing();
    ImGui::Text("Integrate Rigid Pos: %.5f ms\nUpdate Particle Vel: %.5f ms",
        world.profile_data_.integrate_rigid_pos, world.profile_data_.update_particle_vel);
    
    ImGui::Separator();
    if (ImGui::Button("  Reset World  ")){
        ResetWorld(window, world);
    }
    constexpr float indent_len{8.f};
    
    // ===== Scene Boundaries =====
    ImGui::Spacing();
    ImGui::SeparatorText("Boundaries:");
    ImGui::Indent(indent_len);
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
    ImGui::Unindent(indent_len);
    
    // ===== Scene Floating Bodies =====
    ImGui::Spacing();
    ImGui::SeparatorText("Floating Bodies:");
    ImGui::Indent(indent_len);
    if (ImGui::SliderFloat("Restitution##floating", &floating_bodies_restitution_, 0.0f, 1.0f))
    {
        for (const auto& obj : floatings_)
        {
            if (!obj->IsValid()) continue;
            world.SetRestitution(*obj, floating_bodies_restitution_);
        }
    }
    if (ImGui::SliderFloat("Friction##floating", &floating_bodies_friction_, 0.0f, 1.0f))
    {
        for (const auto& obj : floatings_)
        {
            if (!obj->IsValid()) continue;
            world.SetFriction(*obj, floating_bodies_friction_);
        }
    }
    ImGui::Spacing();
    
    if (ImGui::Button("Reset Angular Velocity##floating"))
    {
        floating_bodies_angular_vel_ = 0.f;
        for (const auto& obj : floatings_)
        {
            if (!obj->IsValid()) continue;
            world.SetAngularVelocity(*obj, 0.f);
        }
    }
    if (ImGui::SliderFloat("Angular Velocity##floating", &floating_bodies_angular_vel_, -6.0f, 6.0f))
    {
        for (const auto& obj : floatings_)
        {
            if (!obj->IsValid()) continue;
            world.SetAngularVelocity(*obj, floating_bodies_angular_vel_);
        }
    }
    ImGui::Unindent(indent_len);
    
    // ===== Mouse Button Spawned Bodies =====
    ImGui::Spacing();
    ImGui::SeparatorText("Mouse Spawn Configuration:");
    ImGui::Indent(indent_len);
    const char* spawn_type_names[] = {
        "Rigid Circle",
        "Rigid Box",
        "Soft Rope",
        "Soft Box",
        "Triangle Area Soft Circle",
        "Pressure Soft Circle"
    };
    
    int left_idx = static_cast<int>(left_click_spawn_type_);
    if (ImGui::Combo("Left Mouse", &left_idx, spawn_type_names, IM_ARRAYSIZE(spawn_type_names)))
    {
        left_click_spawn_type_ = static_cast<SpawnType>(left_idx);
    }
    
    int right_idx = static_cast<int>(right_click_spawn_type_);
    if (ImGui::Combo("Right Mouse", &right_idx, spawn_type_names, IM_ARRAYSIZE(spawn_type_names)))
    {
        right_click_spawn_type_ = static_cast<SpawnType>(right_idx);
    }
    ImGui::Spacing();
    
    ImGui::Text("Left Mouse:");
    ImGui::Indent(indent_len);
    RenderSpawnTypeConfigUi(left_click_spawn_type_);
    ImGui::Spacing();
    ImGui::Unindent(indent_len);
    
    ImGui::Text("Right Mouse:");
    // Avoid selecting the same type with left and right mouse buttons.
    if (left_click_spawn_type_ == right_click_spawn_type_)
    {
        ImGui::TextDisabled("(Same as Left Mouse settings)");
    }
    else
    {
        ImGui::Indent(indent_len);
        RenderSpawnTypeConfigUi(right_click_spawn_type_);
        ImGui::Unindent(indent_len);
    }
    ImGui::Unindent(indent_len);
    
    // ===== Debug Info Render Options =====
    ImGui::Spacing();
    ImGui::SeparatorText("Debug Options:");
    ImGui::Indent(indent_len);
    ImGui::Checkbox("Is Multi-Thread", &world.is_multi_thread_);
    ImGui::Checkbox("Draw Contact Info", &world.show_debug_contact_points_);
    ImGui::Checkbox("Draw IncidentEdge & ReferenceEdge(multi-thread only)", &world.show_debug_clipping_);
    ImGui::Text("BVH :");
    ImGui::Checkbox("Draw Global BVH", &draw_bvh_);
    ImGui::Checkbox("Draw first Rope Particle BVH(Particle BVH is only\nupdated when it collides with another soft body)", &draw_first_rope_bvh_);
    ImGui::SliderInt("BVH Draw Depth", &bvh_draw_max_depth_, 0, 20);
    ImGui::Checkbox("Draw Surface Particle of Soft Body", &draw_surface_particle_);
    ImGui::Unindent(indent_len);
    
    ImGui::End();
}

void Application::SpawnBodyAt(PhysicsWorld& world, const SpawnType type, const Vec2 spawn_pos) const
{
    switch (type)
    {
    case SpawnType::RigidCircle:
        world.CreateCircle(
            spawn_pos, 0.0f, {0.f, 0.f}, 0.0f,
            circle_spawn_mass_, circle_spawn_radius_,
            circle_spawn_restitution_, circle_spawn_friction_
        );
        break;

    case SpawnType::RigidBox:
        world.CreateBox(
            spawn_pos, 0.0f, {0.f, 0.f}, 0.0f,
            box_spawn_mass_, box_spawn_half_width_, box_spawn_half_height_,
            box_spawn_restitution_, box_spawn_friction_
        );
        break;

    case SpawnType::SoftRope:
        {
            const Vec2 end_pos = is_horizontal_ ? spawn_pos + Vec2{rope_spawn_length_, 0.0f} : spawn_pos - Vec2{0.0f, rope_spawn_length_};
            world.CreateRope(spawn_pos, end_pos, static_cast<uint16_t>(rope_spawn_num_segments_), rope_spawn_mass_, rope_spawn_friction_, rope_spawn_compliance_, fix_start_, fix_end_);
            break;
        }

    case SpawnType::SoftBox:
        world.CreateSoftBox(
            spawn_pos, soft_box_spawn_width_, soft_box_spawn_height_,
            static_cast<uint16_t>(soft_box_spawn_nx_), static_cast<uint16_t>(soft_box_spawn_ny_),
            soft_box_spawn_mass_, soft_box_spawn_friction_, soft_box_spawn_compliance_
        );
        break;

    case SpawnType::SoftCircleArea:
        world.CreateSoftCircle(
            spawn_pos, soft_circle_spawn_radius_, static_cast<uint16_t>(soft_circle_spawn_num_segments_),
            soft_circle_spawn_mass_, soft_circle_spawn_friction_, soft_circle_spawn_compliance_
        );
        break;

    case SpawnType::SoftCirclePressure:
        world.CreatePressureSoftCircle(
            spawn_pos, pressure_circle_spawn_radius_, static_cast<uint16_t>(pressure_circle_spawn_num_segments_),
            pressure_circle_spawn_mass_, pressure_circle_spawn_friction_, pressure_circle_spawn_compliance_
        );
        break;
    }
}

void Application::RenderSpawnTypeConfigUi(const SpawnType type)
{
    switch (type)
    {
    case SpawnType::RigidCircle:
        ImGui::SameLine();
        ImGui::Text("Rigid Circle");
        ImGui::SliderFloat("Restitution##c", &circle_spawn_restitution_, 0.0f, 1.0f);
        ImGui::SliderFloat("Friction##c", &circle_spawn_friction_, 0.0f, 1.0f);
        ImGui::SliderFloat("Mass##c", &circle_spawn_mass_, 0.1f, 50.0f);
        ImGui::SliderFloat("Radius##c", &circle_spawn_radius_, 0.1f, 5.0f);
        break;

    case SpawnType::RigidBox:
        ImGui::SameLine();
        ImGui::Text("Rigid Box");
        ImGui::SliderFloat("Restitution##b", &box_spawn_restitution_, 0.0f, 1.0f);
        ImGui::SliderFloat("Friction##b", &box_spawn_friction_, 0.0f, 1.0f);
        ImGui::SliderFloat("Mass##b", &box_spawn_mass_, 0.1f, 50.0f);
        ImGui::SliderFloat("Half-Width##b", &box_spawn_half_width_, 0.1f, 5.0f);
        ImGui::SliderFloat("Half-Height##b", &box_spawn_half_height_, 0.1f, 5.0f);
        break;

    case SpawnType::SoftRope:
        ImGui::SameLine();
        ImGui::Text("Soft Rope");
        ImGui::Checkbox("Fix Start", &fix_start_);
        ImGui::SameLine();
        ImGui::Checkbox("Fix End", &fix_end_);
        ImGui::SameLine();
        ImGui::Checkbox("Horizontal", &is_horizontal_);
        ImGui::SliderFloat("Length##rope", &rope_spawn_length_, 1.0f, 30.0f);
        ImGui::SliderInt("Segments##rope", &rope_spawn_num_segments_, 1, 128);
        ImGui::SliderFloat("Mass##rope", &rope_spawn_mass_, 0.1f, 100.0f);
        ImGui::SliderFloat("Friction##rope", &rope_spawn_friction_, 0.0f, 1.0f);
        ImGui::SliderFloat("Compliance##rope", &rope_spawn_compliance_, 0.0f, 0.005f, "%.7f");
        break;

    case SpawnType::SoftBox:
        ImGui::SameLine();
        ImGui::Text("Soft Box");
        ImGui::SliderFloat("Width##sbox", &soft_box_spawn_width_, 0.5f, 10.0f);
        ImGui::SliderFloat("Height##sbox", &soft_box_spawn_height_, 0.5f, 10.0f);
        ImGui::SliderInt("Grid Nx##sbox", &soft_box_spawn_nx_, 2, 20);
        ImGui::SliderInt("Grid Ny##sbox", &soft_box_spawn_ny_, 2, 20);
        ImGui::SliderFloat("Mass##sbox", &soft_box_spawn_mass_, 0.1f, 100.0f);
        ImGui::SliderFloat("Friction##sbox", &soft_box_spawn_friction_, 0.0f, 1.0f);
        ImGui::SliderFloat("Compliance##sbox", &soft_box_spawn_compliance_, 0.0f, 0.005f, "%.7f");
        break;

    case SpawnType::SoftCircleArea:
        ImGui::SameLine();
        ImGui::Text("Triangle Area Soft Circle");
        ImGui::SliderFloat("Radius##scircle", &soft_circle_spawn_radius_, 0.5f, 10.0f);
        ImGui::SliderInt("Segments##scircle", &soft_circle_spawn_num_segments_, 3, 128);
        ImGui::SliderFloat("Mass##scircle", &soft_circle_spawn_mass_, 0.1f, 100.0f);
        ImGui::SliderFloat("Friction##scircle", &soft_circle_spawn_friction_, 0.0f, 1.0f);
        ImGui::SliderFloat("Compliance##scircle", &soft_circle_spawn_compliance_, 0.0f, 0.005f, "%.7f");
        break;

    case SpawnType::SoftCirclePressure:
        ImGui::SameLine();
        ImGui::Text("Pressure Soft Circle");
        ImGui::SliderFloat("Radius##pcircle", &pressure_circle_spawn_radius_, 0.5f, 10.0f);
        ImGui::SliderInt("Segments##pcircle", &pressure_circle_spawn_num_segments_, 3, 128);
        ImGui::SliderFloat("Mass##pcircle", &pressure_circle_spawn_mass_, 0.0f, 100.0f);
        ImGui::SliderFloat("Friction##pcircle", &pressure_circle_spawn_friction_, 0.0f, 1.0f);
        ImGui::SliderFloat("Compliance##pcircle", &pressure_circle_spawn_compliance_, 0.0f, 0.001f, "%.7f");
        break;
    }
}

void Application::ResetWorld(SDL_Window* window, PhysicsWorld& world)
{
    world.ResetWorld();
    CreateInitialBodies(window, world);
}

ImU32 Application::GetDepthColor(const int depth, const float alpha)
{
    const float hue = std::fmod(0.15f * static_cast<float>(depth), 1.0f); // Each depth has a different color.
    float r, g, b;
    ImGui::ColorConvertHSVtoRGB(hue, 0.8f, 0.9f, r, g, b);
    return ImColor(r, g, b, alpha);
}

void Application::DrawBvhNode(ImDrawList* draw_list, const std::span<BvhNode> nodes, const int node_idx, const int current_depth,
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
        if (node.left_child != std::numeric_limits<uint32_t>::max())
        {
            DrawBvhNode(draw_list, nodes, (uint32_t)node.left_child, current_depth + 1, max_render_depth, hovered_node_idx, window_height);
        }
        if (node.right_child != std::numeric_limits<uint32_t>::max())
        {
            DrawBvhNode(draw_list, nodes, (uint32_t)node.right_child, current_depth + 1, max_render_depth, hovered_node_idx, window_height);    
        }
    }
}

void Application::RenderBvhDebug(const std::span<BvhNode> nodes, const int root_idx, const int max_render_depth, const float window_height)
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
        if (target.left_child != std::numeric_limits<uint32_t>::max()) {
            ImVec2 left_center = GetAabbCenter(nodes[target.left_child].aabb);
            left_center = {left_center.x * physics_config::kPpm, window_height - left_center.y * physics_config::kPpm};
            draw_list->AddLine(target_center, left_center, IM_COL32(255, 50, 50, 255), 2.0f);
            draw_list->AddCircleFilled(left_center, 4.0f, IM_COL32(255, 50, 50, 255));
        }

        // Right Child blue.
        if (target.right_child != std::numeric_limits<uint32_t>::max()) {
            ImVec2 right_center = GetAabbCenter(nodes[target.right_child].aabb);
            right_center = {right_center.x * physics_config::kPpm, window_height - right_center.y * physics_config::kPpm};
            draw_list->AddLine(target_center, right_center, IM_COL32(50, 150, 255, 255), 2.0f);
            draw_list->AddCircleFilled(right_center, 4.0f, IM_COL32(50, 150, 255, 255));
        }
    }
}
