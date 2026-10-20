#include <algorithm>
#include <array>
#include <numbers>
#include <SDL3/SDL.h>
#include <imgui_impl_sdl3.h>
#include <imgui_impl_sdlrenderer3.h>

#include "application.h"
#include "physics/physics_config.h"
#include "physics/physics_world.h"

void Application::CreateInitialBodies(SDL_Window* window, PhysicsWorld& world, scenes::PresetScene preset)
{
    int w = 0;
    int h = 0;

    SDL_GetWindowSize(window, &w, &h);

    const scenes::SceneSettings settings{
        .floating_angular_velocity = floating_bodies_angular_vel_,
        .floating_restitution = floating_bodies_restitution_,
        .floating_friction = floating_bodies_friction_,
        .boundary_restitution = boundary_restitution_,
        .boundary_friction = boundary_friction_
    };

    scene_objects_ = scenes::SceneBuilder::Create(preset,
        world, {.width = w, .height = h}, settings);
}

void Application::Input(SDL_Window* window, bool& is_running, PhysicsWorld& world)
{
    int w, h;
    SDL_GetWindowSize(window, &w, &h);
    const float float_h = static_cast<float>(h);

    const ImGuiIO& io = ImGui::GetIO();
    SDL_Event event;
    constexpr float mpp = physics_config::kMpp;
    
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
            const Vec2 mouse_pos = Vec2{ event.button.x, float_h - event.button.y } * mpp;
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
            if (scene_objects_.debug_box.IsValid())
            {
                world.SetPosition({scene_objects_.debug_box}, Vec2{(event.motion.x), float_h - event.motion.y} * mpp);    
            }
            if (scene_objects_.debug_circle.IsValid())
            {
                world.SetPosition({scene_objects_.debug_circle}, Vec2{(event.motion.x), float_h - event.motion.y} * mpp);    
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
    constexpr  float ppm = physics_config::kPpm;
    
    ImDrawList* draw_list = ImGui::GetBackgroundDrawList();

    const auto& positions = world.GetPositions();
    const auto& rots = world.GetRots();
    const auto& shapes = world.GetShapes();
    const auto& particle_positions = world.GetParticlePositions();
    const auto& soft_body_handles = world.GetSoftBodyHandles();
    // ==== Rigid Body ====
    for (int i = 0; i < shapes.size(); i++)
    {
        std::visit(Overload {
        [&](const CircleShape& c)
        {
            const ImVec2 center(positions[i].val.x * ppm, float_h - positions[i].val.y * ppm);
            const float radius = c.radius * ppm;
            
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
                    const float wx = (v.x * rots[i].c - v.y * rots[i].s + positions[i].val.x) * ppm;
                    const float wy = float_h - (v.x * rots[i].s + v.y * rots[i].c + positions[i].val.y) * ppm;
                    draw_vertices[idx] = ImVec2(wx, wy);
                }

                draw_list->AddPolyline(draw_vertices.data(), 4, IM_COL32(255, 0, 0, 255), ImDrawFlags_Closed, 2.f);
        },
        [](const PolygonShape& p) {  }
            }, shapes[i]);
    }
    
    // ==== Soft Body ====
    if (draw_particle_)
    {
        for (const auto& particle_position : particle_positions)
        {
            ImVec2 pos{particle_position.x * ppm, float_h - particle_position.y * ppm};
            draw_list->AddCircleFilled(pos, 4.f, IM_COL32(255, 0, 0, 255));
        }    
    }
    
    for (const auto& constraint : world.GetDistanceConstraints())
    {
        ImVec2 p0(particle_positions[constraint.p1].x * ppm, float_h - particle_positions[constraint.p1].y * ppm);
        ImVec2 p1(particle_positions[constraint.p2].x * ppm, float_h - particle_positions[constraint.p2].y * ppm);
        draw_list->AddLine(p0, p1, IM_COL32(255, 0, 0, 255), 2.f);
    }
    
    if (draw_surface_particle_)
    {
        for (const auto& soft_body_handle : soft_body_handles)
        {
            for (uint16_t i = 0; i < soft_body_handle.surface_particle_count; i++)
            {
                const uint32_t idx = soft_body_handle.start_surface_particle_idx + i;
                ImVec2 pos{particle_positions[idx].x * ppm, float_h - particle_positions[idx].y * ppm};
                draw_list->AddCircleFilled(pos, 5.f, IM_COL32(255, 127, 0, 255));
            }
        }
    }
    
    // ==== Debug Option ====
    for (const auto& incident_edge : world.debug_incident_edges_)
    {
        ImVec2 p0(incident_edge[0].x * ppm, float_h - incident_edge[0].y * ppm);
        ImVec2 p1(incident_edge[1].x * ppm, float_h - incident_edge[1].y * ppm);
        draw_list->AddLine(p0, p1, IM_COL32(20, 200, 10, 255), 5.f);
    }
    
    for (const auto& reference_edge : world.debug_reference_edges_)
    {
        ImVec2 p0(reference_edge[0].x * ppm, float_h - reference_edge[0].y * ppm);
        ImVec2 p1(reference_edge[1].x * ppm, float_h - reference_edge[1].y * ppm);
        draw_list->AddLine(p0, p1, IM_COL32(0, 160, 240, 255), 5.f);
    }
    
    if (world.show_debug_contact_points_)
    {
        for (const auto& manifold : world.GetManifolds())
        {
            if (manifold.contact_count == 0) continue;
        
            for (uint8_t i = 0; i < manifold.contact_count; ++i)
            {
                ImVec2 pt_a(manifold.contacts[i].collision_point_a.x * ppm, float_h - manifold.contacts[i].collision_point_a.y * ppm);
                ImVec2 pt_b(manifold.contacts[i].collision_point_b.x * ppm, float_h - manifold.contacts[i].collision_point_b.y * ppm);
        
                // Line collision_point_a to collision_point_b.
                draw_list->AddLine(pt_a, pt_b, IM_COL32(255, 255, 0, 255), 5.f);
            
                // Draw collision point a.
                draw_list->AddCircleFilled(pt_a, 5.f, IM_COL32(190, 190, 190, 255));
            }
        }    
    }
    
    if (!world.GetBvhNodes().empty() && draw_bvh_) {
        RenderBvhDebug(world.GetBvhNodes(), 0, bvh_draw_max_depth_, float_h);
    }
    
    if (draw_sub_bvh_) {
        for (const auto& sub_bvh : world.GetSubBvhs())
        {
            RenderBvhDebug(sub_bvh, 0, bvh_draw_max_depth_, float_h);    
        }
    }
    
    draw_list->AddText(ImGui::GetFont(), 30.f, ImVec2(static_cast<float>(w) * 0.4f, float_h * 0.05f), IM_COL32(255, 255, 255, 255),
        "Arrow Key to apply force to all bodies\nLeft/Right Mouse to spawn body" );
    
    RenderPhysicsControlPanel(window, world, w);
    
    ImGui::Render();
    SDL_SetRenderDrawColor(renderer, 20, 20, 20, 255);
    SDL_RenderClear(renderer);
    
    ImGui_ImplSDLRenderer3_RenderDrawData(ImGui::GetDrawData(), renderer);
    SDL_RenderPresent(renderer);
    render_time_ms_ = Duration(Clock::now() - start).count();
}

void Application::RenderPhysicsControlPanel(SDL_Window* window, PhysicsWorld& world, const int window_width)
{
    ImGui::SetNextWindowPos(ImVec2(static_cast<float>(window_width) - 415.f, 10), ImGuiCond_Appearing);
    ImGui::SetNextWindowSize(ImVec2(415, 1060), ImGuiCond_Appearing);
    ImGui::SetNextWindowCollapsed(false, ImGuiCond_Appearing);
    
    ImGui::Begin("Physics Engine Control");
    constexpr float indent_len{8.f};

    ImGui::Text("FPS: %.1f, Render: %.5f ms",
        ImGui::GetIO().Framerate, render_time_ms_);
    
    // ===== Counts =====
    ImGui::Separator();
    ImGui::Text("Counts:");
    
    ImGui::Indent(indent_len);
    ImGui::Text("Rigid Bodies: %zu, Soft Bodies: %zu, Particles: %zu",
        world.GetPositions().size(), world.GetSoftBodyHandles().size(), world.GetParticlePositions().size());
    
    ImGui::Text("Manifolds: %zu\nParticle-Rigid Contacts: %zu\nParticle-Edge Contacts: %zu",
        world.GetManifolds().size(), world.GetParticleRigidConstraintsSize(), world.GetParticleEdgeConstraintsSize());
    ImGui::Unindent(indent_len);
    
    // ===== Profiler =====
    ImGui::Separator();
    ImGui::Text("Integrate Rigid Vel: %.5f ms\nPredict Particle Pos: %.5f ms",
        world.profile_data_.integrate_rigid_vel, world.profile_data_.predict_particle_pos);
    
    ImGui::Spacing();
    ImGui::Text("Broad-Phase: %.5f ms\n  Build Bvh: %.5f ms\n  Query Bvh: %.5f ms\n  Erase Manifolds: %.5f ms",
        world.profile_data_.broad_phase_ms, world.profile_data_.build_bvh_ms, world.profile_data_.query_bvh_ms, world.profile_data_.erase_manifold_ms);
    
    ImGui::Spacing();
    ImGui::Text("Narrow-Phase: %.5f ms\n  Rigid - Rigid: %.5f ms\n  Rigid - Soft: %.5f ms\n  Soft - Soft: %.5f ms"
                "\n    Update Local AABB & BVH: %.5f ms\n    Detection: %.5f ms",
                world.profile_data_.narrow_phase_ms, world.profile_data_.rigid_rigid_ms, world.profile_data_.rigid_soft_ms,
                world.profile_data_.soft_soft_ms, world.profile_data_.ss_update_local_aabb_bvh, world.profile_data_.ss_detection);
    
    ImGui::Spacing();
    ImGui::Text("Solve XPBD Constraint: %.5f ms\nSolve Sequential Impulse Constraint: %.5f ms",
        world.profile_data_.solve_xpbd_constraints_ms, world.profile_data_.solve_si_constraints_ms);
    
    ImGui::Spacing();
    ImGui::Text("Integrate Rigid Pos: %.5f ms\nUpdate Particle Vel: %.5f ms",
        world.profile_data_.integrate_rigid_pos, world.profile_data_.update_particle_vel);
    
    // ===== Preset Scenes =====
    ImGui::Separator();
    ImGui::Text("Preset Scenes(click to reset scene):");
    
    if (ImGui::Button(" Default ")){
        world.ClearWorld();
        CreateInitialBodies(window, world, scenes::PresetScene::kDefaultDemo);
    }
    
    ImGui::SameLine();
    if (ImGui::Button(" Box Pyramid ")){
        world.ClearWorld();
        CreateInitialBodies(window, world, scenes::PresetScene::kBoxPyramid);
    }
    
    ImGui::SameLine();
    if (ImGui::Button(" Box Stack ")){
        world.ClearWorld();
        CreateInitialBodies(window, world, scenes::PresetScene::kBoxStack);
    }
    
    ImGui::SameLine();
    if (ImGui::Button(" Soft Demo ")){
        world.ClearWorld();
        CreateInitialBodies(window, world, scenes::PresetScene::kSoftBodyDemo);
    }
    
    if (ImGui::Button(" Box Stress Test ")){
        world.ClearWorld();
        CreateInitialBodies(window, world, scenes::PresetScene::kBoxStressTest);
    }
    
    ImGui::SameLine();
    if (ImGui::Button(" Pressure Circle Stress Test ")){
        world.ClearWorld();
        CreateInitialBodies(window, world, scenes::PresetScene::kPressureCircleStressTest);
    }
    
    if (ImGui::Button(" Rope Stress Test ")){
        world.ClearWorld();
        CreateInitialBodies(window, world, scenes::PresetScene::kRopeStressTest);
    }
    
    // ===== Scene Boundaries =====
    ImGui::Spacing();
    ImGui::SeparatorText("Boundaries:");
    ImGui::Indent(indent_len);
    if (ImGui::SliderFloat("Restitution##bound", &boundary_restitution_, 0.f, 1.f))
    {
        for (const auto& obj : boundaries_)
        {
            if (!obj->IsValid()) continue;
            world.SetRestitution(*obj, boundary_restitution_);
        }
    }
    if (ImGui::SliderFloat("Friction##bound", &boundary_friction_, 0.f, 1.f))
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
    if (ImGui::SliderFloat("Restitution##floating", &floating_bodies_restitution_, 0.f, 1.f))
    {
        for (const auto& obj : floatings_)
        {
            if (!obj->IsValid()) continue;
            world.SetRestitution(*obj, floating_bodies_restitution_);
        }
    }
    if (ImGui::SliderFloat("Friction##floating", &floating_bodies_friction_, 0.f, 1.f))
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
    if (ImGui::SliderFloat("Angular Velocity##floating", &floating_bodies_angular_vel_, -6.f, 6.f))
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
        "Pressure Soft Circle",
        "Car",
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
    ImGui::Checkbox("Use Single-Thread(Rigid-Rigid&Soft-Soft Narrow-Phase)", &world.use_single_thread_);
    ImGui::Checkbox("Use Local BVH in Soft-Soft Narrow-Phase", &world.use_local_bvh_);
    ImGui::Checkbox("Draw Contact Info", &world.show_debug_contact_points_);
    ImGui::Checkbox("Draw Incident/Reference Edge(single-thread only)", &world.show_debug_clipping_);
    ImGui::Checkbox("Draw Particle of Soft Body", &draw_particle_);
    ImGui::Checkbox("Draw Surface Particle of Soft Body", &draw_surface_particle_);
    ImGui::Spacing();
    ImGui::Text("BVH(Deeper nodes' AABBs shrink more when rendered) :");
    ImGui::Checkbox("Draw Global BVH", &draw_bvh_);
    ImGui::Checkbox("Draw Soft Body Local BVH (Only updated\nwhen it collides with another soft body)", &draw_sub_bvh_);
    ImGui::SliderInt("BVH Draw Depth", &bvh_draw_max_depth_, 0, 20);
    ImGui::Spacing();
    ImGui::Text("Arrow Key to apply force to all bodies :");
    ImGui::SliderFloat("Arrow Key Force", &arrow_key_force_, 0, 2000.f);
    ImGui::Unindent(indent_len);
    
    ImGui::End();
}

void Application::SpawnBodyAt(PhysicsWorld& world, const SpawnType type, const Vec2 spawn_pos) const
{
    switch (type)
    {
    case SpawnType::kRigidCircle:
        world.CreateCircle(
            spawn_pos, 0.f, {0.f, 0.f}, 0.f,
            circle_spawn_mass_, circle_spawn_radius_,
            circle_spawn_restitution_, circle_spawn_friction_
        );
        break;

    case SpawnType::kRigidBox:
        world.CreateBox(
            spawn_pos, 0.f, {0.f, 0.f}, 0.f,
            box_spawn_mass_, box_spawn_half_width_, box_spawn_half_height_,
            box_spawn_restitution_, box_spawn_friction_
        );
        break;

    case SpawnType::kSoftRope:
        {
            const Vec2 end_pos = is_horizontal_ ? spawn_pos + Vec2{rope_spawn_length_, 0.f} : spawn_pos - Vec2{0.f, rope_spawn_length_};
            world.CreateRope(spawn_pos, end_pos, static_cast<uint16_t>(rope_spawn_num_segments_), rope_spawn_mass_, rope_spawn_friction_, rope_spawn_compliance_, fix_start_, fix_end_);
            break;
        }

    case SpawnType::kSoftBox:
        world.CreateSoftBox(
            spawn_pos, soft_box_spawn_width_, soft_box_spawn_height_,
            static_cast<uint16_t>(soft_box_spawn_nx_), static_cast<uint16_t>(soft_box_spawn_ny_),
            soft_box_spawn_mass_, soft_box_spawn_friction_, soft_box_spawn_compliance_
        );
        break;

    case SpawnType::kSoftCircleArea:
        world.CreateSoftCircle(
            spawn_pos, soft_circle_spawn_radius_, static_cast<uint16_t>(soft_circle_spawn_num_segments_),
            soft_circle_spawn_mass_, soft_circle_spawn_friction_, soft_circle_spawn_compliance_
        );
        break;

    case SpawnType::kSoftCirclePressure:
        world.CreatePressureSoftCircle(
            spawn_pos, pressure_circle_spawn_radius_, static_cast<uint16_t>(pressure_circle_spawn_num_segments_),
            pressure_circle_spawn_mass_, pressure_circle_spawn_friction_, pressure_circle_spawn_compliance_
        );
        break;
        
    case SpawnType::kCar:
        world.CreateCar(spawn_pos, wheel_radius_, wheel_mass_, wheel_friction_, wheel_restitution_, suspension_height_,
                        suspension_mass_, suspension_compliance_, car_body_width_,
                        car_body_height_, car_body_mass_, car_body_restitution_, car_body_friction_);
        break;
    }
}

void Application::RenderSpawnTypeConfigUi(const SpawnType type)
{
    switch (type)
    {
    case SpawnType::kRigidCircle:
        ImGui::SameLine();
        ImGui::Text("Rigid Circle");
        ImGui::SliderFloat("Restitution##c", &circle_spawn_restitution_, 0.f, 1.f);
        ImGui::SliderFloat("Friction##c", &circle_spawn_friction_, 0.f, 1.f);
        ImGui::SliderFloat("Mass##c", &circle_spawn_mass_, 0.1f, 50.f);
        ImGui::SliderFloat("Radius##c", &circle_spawn_radius_, 0.1f, 5.f);
        break;

    case SpawnType::kRigidBox:
        ImGui::SameLine();
        ImGui::Text("Rigid Box");
        ImGui::SliderFloat("Restitution##b", &box_spawn_restitution_, 0.f, 1.f);
        ImGui::SliderFloat("Friction##b", &box_spawn_friction_, 0.f, 1.f);
        ImGui::SliderFloat("Mass##b", &box_spawn_mass_, 0.1f, 50.f);
        ImGui::SliderFloat("Half-Width##b", &box_spawn_half_width_, 0.1f, 5.f);
        ImGui::SliderFloat("Half-Height##b", &box_spawn_half_height_, 0.1f, 5.f);
        break;

    case SpawnType::kSoftRope:
        ImGui::SameLine();
        ImGui::Text("Soft Rope");
        ImGui::Checkbox("Fix Start", &fix_start_);
        ImGui::SameLine();
        ImGui::Checkbox("Fix End", &fix_end_);
        ImGui::SameLine();
        ImGui::Checkbox("Horizontal", &is_horizontal_);
        if (ImGui::SliderFloat("Length##rope", &rope_spawn_length_, 1.f, 30.f))
        {
            rope_max_spawn_num_segments_ = static_cast<int>(rope_spawn_length_ * physics_config::kParticleDensityLenght);
            rope_spawn_num_segments_ = std::clamp(rope_spawn_num_segments_, 3, rope_max_spawn_num_segments_);
        }
        ImGui::SliderInt("Segments##rope", &rope_spawn_num_segments_, 1, rope_max_spawn_num_segments_);
        ImGui::SliderFloat("Mass##rope", &rope_spawn_mass_, 0.1f, 100.f);
        ImGui::SliderFloat("Friction##rope", &rope_spawn_friction_, 0.f, 1.f);
        ImGui::SliderFloat("Compliance##rope", &rope_spawn_compliance_, 0.f, 0.005f, "%.7f");
        break;

    case SpawnType::kSoftBox:
        ImGui::SameLine();
        ImGui::Text("Soft Box");
        if (ImGui::SliderFloat("Width##sbox", &soft_box_spawn_width_, 0.5f, 10.f))
        {
            soft_box_max_spawn_nx_ = static_cast<int>(soft_box_spawn_width_ * physics_config::kParticleDensityLenght);
            soft_box_spawn_nx_ = std::clamp(soft_box_spawn_nx_, 2, soft_box_max_spawn_nx_);
        }
        if (ImGui::SliderFloat("Height##sbox", &soft_box_spawn_height_, 0.5f, 10.f))
        {
            soft_box_max_spawn_ny_ = static_cast<int>(soft_box_spawn_height_ * physics_config::kParticleDensityLenght);
            soft_box_spawn_ny_ = std::clamp(soft_box_spawn_ny_, 2, soft_box_max_spawn_ny_);
        }
        ImGui::SliderInt("Grid Nx##sbox", &soft_box_spawn_nx_, 2, soft_box_max_spawn_nx_);
        ImGui::SliderInt("Grid Ny##sbox", &soft_box_spawn_ny_, 2, soft_box_max_spawn_ny_);
        ImGui::SliderFloat("Mass##sbox", &soft_box_spawn_mass_, 0.1f, 100.f);
        ImGui::SliderFloat("Friction##sbox", &soft_box_spawn_friction_, 0.f, 1.f);
        ImGui::SliderFloat("Compliance##sbox", &soft_box_spawn_compliance_, 0.f, 0.005f, "%.7f");
        break;

    case SpawnType::kSoftCircleArea:
        ImGui::SameLine();
        ImGui::Text("Triangle Area Soft Circle");
        if (ImGui::SliderFloat("Radius##scircle", &soft_circle_spawn_radius_, 0.5f, 10.f))
        {
            soft_circle_max_spawn_num_segments_ = static_cast<int>(soft_circle_spawn_radius_ * physics_config::kParticleDensityRadius);
            soft_circle_spawn_num_segments_ = std::clamp(soft_circle_spawn_num_segments_, 3, soft_circle_max_spawn_num_segments_);
        }
        ImGui::SliderInt("Segments##scircle", &soft_circle_spawn_num_segments_, 3, soft_circle_max_spawn_num_segments_);
        ImGui::SliderFloat("Mass##scircle", &soft_circle_spawn_mass_, 0.1f, 100.f);
        ImGui::SliderFloat("Friction##scircle", &soft_circle_spawn_friction_, 0.f, 1.f);
        ImGui::SliderFloat("Compliance##scircle", &soft_circle_spawn_compliance_, 0.f, 0.005f, "%.7f");
        break;

    case SpawnType::kSoftCirclePressure:
        ImGui::SameLine();
        ImGui::Text("Pressure Soft Circle");
        if (ImGui::SliderFloat("Radius##pcircle", &pressure_circle_spawn_radius_, 0.5f, 10.f))
        {
            pressure_circle_max_spawn_num_segments_ = static_cast<int>(pressure_circle_spawn_radius_ * physics_config::kParticleDensityRadius);
            pressure_circle_spawn_num_segments_ = std::clamp(pressure_circle_spawn_num_segments_, 3, pressure_circle_max_spawn_num_segments_);
        }
        ImGui::SliderInt("Segments##pcircle", &pressure_circle_spawn_num_segments_, 3, pressure_circle_max_spawn_num_segments_);
        ImGui::SliderFloat("Mass##pcircle", &pressure_circle_spawn_mass_, 0.f, 100.f);
        ImGui::SliderFloat("Friction##pcircle", &pressure_circle_spawn_friction_, 0.f, 1.f);
        ImGui::SliderFloat("Compliance##pcircle", &pressure_circle_spawn_compliance_, 0.f, 0.001f, "%.7f");
        break;
        
    case SpawnType::kCar:
        ImGui::SameLine();
        ImGui::Text("Car");
        ImGui::SliderFloat("Wheel Radius##car", &wheel_radius_, 0.5f, 2.f);
        ImGui::SliderFloat("Wheel Mass##car", &wheel_mass_, 0.1f, 50.f);
        ImGui::SliderFloat("Wheel Friction##car", &wheel_friction_, 0.f, 1.f);
        ImGui::SliderFloat("Wheel Restitution##car", &wheel_restitution_, 0.f, 1.f);
        ImGui::SliderFloat("Suspension Height##car", &suspension_height_, 0.5f, 5.f);
        ImGui::SliderFloat("Suspension Mass##car", &suspension_mass_, 0.1f, 50.f);
        ImGui::SliderFloat("Suspension Compliance##car", &suspension_compliance_, 0.f, 0.001f, "%.7f");
        ImGui::SliderFloat("Car Body Width##car", &car_body_width_, 1.f, 10.f);
        ImGui::SliderFloat("Car Body Height##car", &car_body_height_, 0.5f, 5.f);
        ImGui::SliderFloat("Car Body Mass##car", &car_body_mass_, 0.1f, 50.f);
        ImGui::SliderFloat("Car Body Restitution##car", &car_body_restitution_, 0.f, 1.f);
        ImGui::SliderFloat("Car Body Friction##car", &car_body_friction_, 0.f, 1.f);
        break;
    }
}

ImU32 Application::GetDepthColor(const int depth, const float alpha)
{
    const float hue = std::fmod(0.15f * static_cast<float>(depth), 1.f); // Each depth has a different color.
    float r, g, b;
    ImGui::ColorConvertHSVtoRGB(hue, 0.8f, 0.9f, r, g, b);
    return ImColor(r, g, b, alpha);
}

void Application::DrawBvhNode(ImDrawList* draw_list, std::span<const BvhNode> nodes, const int node_idx, const int current_depth,
                              const int max_render_depth, int& hovered_node_idx, const float window_height)
{
    if (node_idx >= static_cast<int>(nodes.size())) return;
    if (current_depth > max_render_depth) return;

    constexpr float ppm = physics_config::kPpm;
    constexpr float mpp = physics_config::kMpp;
    const BvhNode& node = nodes[node_idx];
    
    // Shrink by depth to avoid border overlap.
    const float padding = 1.5f * static_cast<float>(current_depth);
    const ImVec2 p_min = ImVec2(node.aabb.min_x * ppm + padding, window_height - node.aabb.min_y * ppm - padding);
    const ImVec2 p_max = ImVec2(node.aabb.max_x * ppm - padding, window_height - node.aabb.max_y * ppm + padding);
    
    // If the mouse is hovering over the current AABB.
    ImVec2 mouse_pos = ImGui::GetIO().MousePos;
    mouse_pos = {mouse_pos.x * mpp, (window_height - mouse_pos.y) * mpp};
    if (AabbContains(node.aabb, mouse_pos)) {
        hovered_node_idx = node_idx;
    }

    // Draw borders and fill with light colors
    const ImU32 stroke_color = GetDepthColor(current_depth, 0.8f);
    const ImU32 fill_color   = GetDepthColor(current_depth, 0.05f);
    
    draw_list->AddRectFilled(p_min, p_max, fill_color);
    draw_list->AddRect(p_min, p_max, stroke_color, 0.f, 0, 1.5f);

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

void Application::RenderBvhDebug(std::span<const BvhNode> nodes, const int root_idx, const int max_render_depth, const float window_height)
{
    ImDrawList* draw_list = ImGui::GetBackgroundDrawList();
    constexpr float ppm = physics_config::kPpm;
    
    int hovered_node_idx = -1;

    // Draw all AABB.
    DrawBvhNode(draw_list, nodes, root_idx, 0, max_render_depth, hovered_node_idx, window_height);

    // When the mouse hovers over an AABB, highlight and draw the children connection line.
    if (hovered_node_idx != -1) {
        const BvhNode& target = nodes[hovered_node_idx];
        ImVec2 target_center = GetAabbCenter(target.aabb);
        target_center = {target_center.x * ppm, window_height - target_center.y * ppm};

        // Highlight the AABB of the current hover (thick yellow border).
        draw_list->AddRect(
            ImVec2(target.aabb.min_x * ppm, window_height - target.aabb.min_y * ppm),
            ImVec2(target.aabb.max_x * ppm, window_height - target.aabb.max_y * ppm),
            IM_COL32(255, 255, 0, 255), 0.f, 0, 3.f
        );

        // Connect red line to the Left Child and mark it with a red dot.
        if (target.left_child != std::numeric_limits<uint32_t>::max()) {
            ImVec2 left_center = GetAabbCenter(nodes[target.left_child].aabb);
            left_center = {left_center.x * ppm, window_height - left_center.y * ppm};
            draw_list->AddLine(target_center, left_center, IM_COL32(255, 50, 50, 255), 2.f);
            draw_list->AddCircleFilled(left_center, 4.f, IM_COL32(255, 50, 50, 255));
        }

        // Right Child blue.
        if (target.right_child != std::numeric_limits<uint32_t>::max()) {
            ImVec2 right_center = GetAabbCenter(nodes[target.right_child].aabb);
            right_center = {right_center.x * ppm, window_height - right_center.y * ppm};
            draw_list->AddLine(target_center, right_center, IM_COL32(50, 150, 255, 255), 2.f);
            draw_list->AddCircleFilled(right_center, 4.f, IM_COL32(50, 150, 255, 255));
        }
    }
}
