#include <numbers>

#include "scene_builder.h"
#include "physics/physics_world.h"
#include "physics/physics_config.h"

namespace scenes
{
    SceneObjects SceneBuilder::Create(const PresetScene preset, PhysicsWorld& world, const ViewportSize viewport, const SceneSettings& settings)
    {
        switch (preset)
        {
        case PresetScene::kDefaultDemo:
            return CreateDefaultDemo(world, viewport, settings);

        case PresetScene::kBoxPyramid:
            return CreateBoxPyramid(world, viewport, settings);
            
        case PresetScene::kBoxStack:
            return CreateBoxStack(world, viewport, settings);
            
        case PresetScene::kSoftBodyDemo:
            return CreateSoftBodyDemo(world, viewport, settings);
            
        case PresetScene::kBoxStressTest:
            CreateBoxStressTest(world, viewport, settings);
            break;
            
        case PresetScene::kPressureCircleStressTest:
            CreatePressureCircleStressTest(world, viewport, settings);
            break;
            
        case PresetScene::kRopeStressTest:
            CreateRopeStressTest(world, viewport);
            break;
        }
        
        return {};
    }

    SceneObjects SceneBuilder::CreateDefaultDemo(PhysicsWorld& world, const ViewportSize viewport, const SceneSettings& settings)
    {
        SceneObjects objects{};

        const float w = static_cast<float>(viewport.width);
        const float h = static_cast<float>(viewport.height);
        constexpr float mpp = physics_config::kMpp;
        
        // Small body for debug (check contact info).
        // objects.debug_circle = world.CreateCircle(Vec2{0.f, 0.f}, 0.f, {0.f, 0.f}, 0.f, 0.f, 1.f, 0.2f, 1.f);
        // objects.debug_box = world.CreateBox(Vec2{0.f, 0.f} * physics_config::kMpp, 1.f, {0.f, 0.f}, 0.f, 0.f, 1.f, 1.f, 1.f);

        // Floating obstacle bodies.
        const Vec2 floating_circle_pos{(w - 700.f) * mpp, (h / 2.f) * mpp};

        objects.floating_circle = world.CreateCircle( floating_circle_pos, 0.f, {0.f, 0.f}, settings.floating_angular_velocity, 
            0.f, 1.75f, settings.floating_restitution, settings.floating_friction);

        objects.floating_circle2 = world.CreateCircle( {floating_circle_pos.x + 2.5f, floating_circle_pos.y}, 
            0.f, {0.f, 0.f}, settings.floating_angular_velocity, 
            0.f, 1.75f, settings.floating_restitution, settings.floating_friction);

        const Vec2 floating_wide_box_pos = Vec2{400.f, static_cast<float>(h) / 2.f } * physics_config::kMpp;
        objects.floating_wide_box1 = world.CreateBox(floating_wide_box_pos, 0.f,
            {0.f, 0.f}, settings.floating_angular_velocity,
            0.f, 3.2f, 0.3f, settings.floating_restitution, settings.floating_friction);
        
        objects.floating_wide_box2 = world.CreateBox(floating_wide_box_pos, std::numbers::pi / 2.f,
            {0.f, 0.f}, settings.floating_angular_velocity,
            0.f, 3.2f, 0.3f, settings.floating_restitution, settings.floating_friction);
        
        // Ceiling.
        const Vec2 ceiling_pos{ w / 2.f * mpp, (h + 15.f) * mpp };
        constexpr float ceiling_half_height = 1.f;
        const float ceiling_half_width = w / 2.f * mpp;
        objects.ceiling = world.CreateBox( ceiling_pos, 0.f, {0.f, 0.f}, 0.f, 
            0.f, ceiling_half_width, ceiling_half_height, settings.boundary_restitution, settings.boundary_friction);

        // Floor.
        const Vec2 floor_pos{ w / 2.f * mpp, 0.f };
        objects.floor = world.CreateBox( floor_pos, 0.f, {0.f, 0.f}, 0.f, 
            0.f, w / 2.f * mpp, 1.f, settings.boundary_restitution, settings.boundary_friction);

        // Left wall.
        const Vec2 left_wall_pos{ 10.f * mpp, h / 2.f * mpp };
        objects.left_wall = world.CreateBox( left_wall_pos, 0.f, {0.f, 0.f}, 0.f, 
            0.f, 1.f, h / 2.f * mpp, settings.boundary_restitution, settings.boundary_friction);

        // Right wall.
        const Vec2 right_wall_pos{ (w - 10.f) * mpp, h / 2.f * mpp };
        objects.right_wall = world.CreateBox( right_wall_pos, 0.f, {0.f, 0.f}, 0.f, 
            0.f, 1.f, h / 2.f * mpp, settings.boundary_restitution, settings.boundary_friction);
        
        // XPBD soft rope.
        Vec2 rope_start_pos = Vec2{static_cast<float>(w) / 2.f, static_cast<float>(h) / 2.f} * physics_config::kMpp;
        rope_start_pos.x -= 9.f;
        rope_start_pos.y += 5.f;
        const Vec2 rope_end_pos{rope_start_pos.x + 12.f, rope_start_pos.y};
        world.CreateRope(rope_start_pos, rope_end_pos, 64,  20, 0.5f, 0.00001f, true, true);
        
        // Hanging Cloth
        constexpr float cloth_width = 5.f;
        constexpr float cloth_height = 3.f;
        constexpr uint16_t res_x = 13;
        constexpr uint16_t res_y = 9;

        const Vec2 cloth_center{ ceiling_pos.x * 0.25f, ceiling_pos.y - ceiling_half_height - cloth_height * 0.5f };
        const SoftBodyHandle cloth = world.CreateSoftBox( cloth_center, cloth_width, cloth_height,
            res_x, res_y, 4.f, 0.5f, 0.001f, 0.01f);

        // Connect each particle on top of the cloth to the ceiling.
        for (uint16_t x = 0; x < res_x; ++x)
        {
            const uint32_t particle_index = cloth.start_particle_idx + x;
            const Vec2 particle_pos = world.GetParticlePosition(particle_index);
            world.AddRigidParticleJointConstraint( particle_index, objects.ceiling.id, {particle_pos.x - ceiling_pos.x, -ceiling_half_height});
        }
        
        // Hanging Box
        constexpr float hang_box_half_width = 0.8f;
        constexpr uint32_t hang_rope_segments = 5;
        const uint32_t hang_rope_start_idx = world.GetParticlePositionsX().size();
        const uint32_t hang_rope_end_idx = hang_rope_start_idx  + hang_rope_segments;
        const Vec2 hang_rope_start_pos{cloth_center.x + cloth_width * 0.5f + hang_box_half_width + 1.f, ceiling_pos.y - 1.f};
        const Vec2 hang_rope_end_pos{hang_rope_start_pos.x, hang_rope_start_pos.y - 3.f};
        const uint32_t hang_rope_handle_idx = world.GetSoftBodyHandles().size();
        world.CreateRope(hang_rope_start_pos, hang_rope_end_pos, hang_rope_segments, 2.f, 0.5f, 0.f, true, false);
        
        const uint32_t hang_box_idx = world.GetPositions().size();
        world.CreateBox(hang_rope_end_pos, 0.f, {8.f, 0.f}, 0.f, 3.f, hang_box_half_width, 0.8f, 0.5f, 0.5f);
        world.AddRigidParticleJointConstraint(hang_rope_end_idx, hang_box_idx, {0.f, 0.f});
        world.AddRigidSoftCollisionIgnore(hang_box_idx, hang_rope_handle_idx);

        return objects;
    }

    SceneObjects SceneBuilder::CreateBoxPyramid(PhysicsWorld& world, const ViewportSize viewport, const SceneSettings& settings)
    {
        SceneObjects objects{};

        const float w = static_cast<float>(viewport.width);
        constexpr float mpp = physics_config::kMpp;
        
        const float floor_half_width = w * mpp * 0.5f;
        constexpr float floor_half_height = 0.5f;
        constexpr float floor_y = 1.0f;

        const Vec2 floor_pos{ (static_cast<float>(w) * 0.5f) * mpp, floor_y };

        // Floor
        objects.floor = world.CreateBox(floor_pos, 0.0f, {0.0f, 0.0f}, 0.0f, 
            0.0f, floor_half_width, floor_half_height, 0.0f, 0.8f);

        // Pyramid parameters
        constexpr int box_count_per_base = 12;
        constexpr float box_half_width = 0.7f;
        constexpr float box_half_height = 0.7f;
        constexpr float box_mass = 2.0f;
        constexpr float box_restitution = 0.0f;
        constexpr float box_friction = 1.0f;
        
        // Initial spacing between 2 boxes
        constexpr float horizontal_spacing = 0.005f;
        constexpr float vertical_spacing = 0.00005f;

        constexpr float box_width = 2.0f * box_half_width;
        constexpr float box_height = 2.0f * box_half_height;

        // Floor top surface y pos
        constexpr float floor_top = floor_y + floor_half_height;
        
        // The center y pos of the first layer
        constexpr float first_row_y = floor_top + box_half_height + vertical_spacing;
        
        const float pyramid_center_x = static_cast<float>(w) * 0.5f * mpp;

        // Build from bottom to top, reducing one Box at each level
        for (int row = 0; row < box_count_per_base; ++row)
        {
            const int boxes_in_row = box_count_per_base - row;

            // Center each layer horizontally
            const float row_width = boxes_in_row * box_width + (boxes_in_row - 1) * horizontal_spacing;

            const float start_x = pyramid_center_x - row_width * 0.5f + box_half_width;

            const float y = first_row_y + row * (box_height + vertical_spacing);
            
            for (int col = 0; col < boxes_in_row; ++col)
            {
                const float x = start_x + col * (box_width + horizontal_spacing);

                world.CreateBox({x, y}, 0.0f, {0.0f, 0.0f},   0.0f, 
                    box_mass, box_half_width, box_half_height, box_restitution, box_friction );
            }
        }
        
        return objects;
    }

    SceneObjects SceneBuilder::CreateBoxStack(PhysicsWorld& world, const ViewportSize viewport, const SceneSettings& settings)
    {
        SceneObjects objects{};
        constexpr float mpp = physics_config::kMpp;

        const float screen_width = static_cast<float>(std::max(viewport.width, 1));
        const float world_width = screen_width * mpp;
        const float center_x = world_width * 0.5f;

        // Match the floor convention in CreateDefaultDemo().
        constexpr float floor_half_height = 1.f;
        constexpr float floor_top = floor_half_height;

        // Conservative values for a stacking stability test.
        constexpr float box_mass = 1.f;
        constexpr float box_restitution = 0.02f;
        constexpr float box_friction = 0.8f;
        
        // Floor
        objects.floor = world.CreateBox( {center_x, 0.f}, 0.f, {0.f, 0.f}, 0.f, 
            0.f, world_width * 0.5f, floor_half_height, settings.boundary_restitution, settings.boundary_friction);
        
        // Helper: create a column from stacked boxes
        // Each box rests directly on the box beneath it.
        // Returns the world-space Y coordinate of the column's top surface.
        const auto create_column = [&](float x, const uint32_t box_count, const float half_width, const float half_height) -> float
        {
            for (uint32_t i = 0; i < box_count; ++i)
            {
                const float y = floor_top + (2.f * static_cast<float>(i) + 1.f) * half_height;

                world.CreateBox({x, y}, 0.f, {0.f, 0.f}, 0.f, box_mass,
                    half_width, half_height, box_restitution, box_friction);
            }

            return floor_top + 2.f * static_cast<float>(box_count) * half_height;
        };
        
        // Outer left:
        // relatively narrow and short.
        const float outer_left_x = center_x - 4.1f;
        create_column( outer_left_x, 10, 0.30f, 0.35f);

        // Inner left:
        // 8 boxes, wider segments.
        // Total column height = 8 * (2 * 0.35) = 5.6.
        const float arch_left_x = center_x - 1.8f;
        const float arch_left_top = create_column( arch_left_x, 8, 0.42f, 0.35f);

        // Inner right:
        // 10 boxes, narrower segments vertically.
        // Total column height = 10 * (2 * 0.28) = 5.6.
        const float arch_right_x = center_x + 1.8f;
        const float arch_right_top = create_column( arch_right_x, 10, 0.48f, 0.28f);

        // Outer right:
        // taller than the outer-left column.
        const float outer_right_x = center_x + 4.1f;
        create_column( outer_right_x, 10, 0.27f, 0.30f);
        
        // Wide horizontal beam: the arch-like structure
        // Both support columns are designed to have the same top height. max() guards against tiny round-off differences in their calculated heights.
        const float support_top = std::max(arch_left_top, arch_right_top);

        constexpr float beam_half_height = 0.24f;
        constexpr float beam_half_width = 2.55f;

        const Vec2 beam_position{ center_x, support_top + beam_half_height };

        world.CreateBox( beam_position, 0.f, {0.f, 0.f}, 0.f,
            box_mass, beam_half_width, beam_half_height, box_restitution, box_friction);

        return objects;
    }

    SceneObjects SceneBuilder::CreateSoftBodyDemo(PhysicsWorld& world, const ViewportSize viewport, const SceneSettings& settings)
    {
        SceneObjects objects{};
        
        const float world_w = static_cast<float>(viewport.width) * physics_config::kMpp;
        const float world_h = static_cast<float>(viewport.height) * physics_config::kMpp;

        // Ceiling beam
        constexpr float beam_half_width = 30.2f;
        constexpr float beam_half_height = 0.2f;

        const Vec2 beam_center{ world_w * 0.32f, world_h * 0.88f };

        const uint32_t beam_index = world.GetPositions().size();

        world.CreateBox( beam_center, 0.f, {0.f, 0.f}, 0.f, 0.f,
            beam_half_width, beam_half_height, settings.boundary_restitution, settings.boundary_friction);

        // Hanging cloth
        constexpr float cloth_width = 5.f;
        constexpr float cloth_height = 3.f;
        constexpr uint16_t res_x = 13;
        constexpr uint16_t res_y = 9;

        const Vec2 cloth_center{ beam_center.x, beam_center.y - beam_half_height - cloth_height * 0.5f };

        const SoftBodyHandle cloth = world.CreateSoftBox( cloth_center, cloth_width, cloth_height,
            res_x, res_y, 4.f, 0.5f, 0.001f, 0.01f);
        
        // Connect each particle on top of the cloth to the ceiling.
        for (uint16_t x = 0; x < res_x; ++x)
        {
            const float t = static_cast<float>(x) / static_cast<float>(res_x - 1);
            const float local_x = -cloth_width * 0.5f + cloth_width * t;
            
            const uint32_t particle_index = cloth.start_particle_idx + x;
            world.AddRigidParticleJointConstraint( particle_index, beam_index, {local_x, -beam_half_height});
        }
        
        // Floor.
        const Vec2 floor_pos{ world_w / 2.f, 0.f };
        objects.floor = world.CreateBox( floor_pos, 0.f, {0.f, 0.f}, 0.f, 
            0.f, world_w / 2.f, 1.f, settings.boundary_restitution, settings.boundary_friction);

        // Left wall.
        const Vec2 left_wall_pos{ 0.f, world_h / 2.f };
        objects.left_wall = world.CreateBox( left_wall_pos, 0.f, {0.f, 0.f}, 0.f, 
            0.f, 1.f, world_h / 2.f, settings.boundary_restitution, settings.boundary_friction);

        // Right wall.
        const Vec2 right_wall_pos{ (world_w), world_h / 2.f };
        objects.right_wall = world.CreateBox( right_wall_pos, 0.f, {0.f, 0.f}, 0.f, 
           0.f, 1.f, world_h / 2.f, settings.boundary_restitution, settings.boundary_friction);
        
        const Vec2 rope_start_pos{ world_w * 0.1f, 7.f };
        const float rope_length = std::min(25.f, world_w * 0.8f);
        const Vec2 rope_end_pos{ rope_start_pos.x + rope_length, rope_start_pos.y };
        world.CreateRope(rope_start_pos, rope_end_pos, 101, 50.f, 0.5f, 0.f, true, true);
        
        
        const Vec2 pressure_circle_pos{ world_w * 0.2f, 9.f };
        world.CreatePressureSoftCircle( pressure_circle_pos, 2.f, 40, 10.f, 0.5f, 0.f);
        
        const Vec2 triangle_area_circle_pos{ world_w * 0.4f, 9.f };
        world.CreateSoftCircle( triangle_area_circle_pos, 2.f, 40, 10.f, 0.5f, 0.f);
        
        const Vec2 soft_box_pos{ world_w * 0.6f, 9.f };
        world.CreateSoftBox( soft_box_pos, 3.f, 3.f,
            8, 8, 4.f, 0.5f, 0.001f, 0.000005f);
        
        return objects;
    }

    namespace
    {
        constexpr float kWallHalfThickness = 1.0f;
        constexpr float kFloorHalfHeight = 1.0f;
        constexpr float kWallMarginPixels = 10.0f;

        struct StressTestBounds
        {
            float world_width;
            float world_height;

            float left_inner_x;
            float right_inner_x;

            float floor_top;
        };

        // Return the number of objects that match the available space.
        int FitCount(const float available, const float item_size, const float gap)
        {
            if (available < item_size || item_size <= 0.0f || gap < 0.0f) return 0;
            
            return static_cast<int>( std::floor( (available + gap) / (item_size + gap)));
        }

        StressTestBounds CreateStressTestBounds( PhysicsWorld& world, const ViewportSize viewport, const SceneSettings& settings)
        {
            constexpr float mpp = physics_config::kMpp;
            const float width = static_cast<float>(viewport.width) * mpp;
            const float height = static_cast<float>(viewport.height) * mpp;
            constexpr float left_wall_x = kWallMarginPixels * mpp;
            const float right_wall_x = (static_cast<float>(viewport.width) - kWallMarginPixels) * mpp;

            // Floor
            world.CreateBox( {width * 0.5f, 0.0f}, 0.0f, {0.0f, 0.0f}, 0.0f,
                0.0f, width * 0.5f + kWallHalfThickness, kFloorHalfHeight, settings.boundary_restitution, settings.boundary_friction);

            // Left Wall
            world.CreateBox( {left_wall_x, height * 0.5f}, 0.0f, {0.0f, 0.0f}, 0.0f,
                0.0f, kWallHalfThickness, height * 0.5f + kFloorHalfHeight, settings.boundary_restitution, settings.boundary_friction);

            // Right Wall
            world.CreateBox( {right_wall_x, height * 0.5f}, 0.0f, {0.0f, 0.0f}, 0.0f, 0.0f,
                kWallHalfThickness, height * 0.5f + kFloorHalfHeight, settings.boundary_restitution, settings.boundary_friction);

            return { width, height, left_wall_x + kWallHalfThickness, right_wall_x - kWallHalfThickness, kFloorHalfHeight };
        }
    }
    
    void SceneBuilder::CreateBoxStressTest(PhysicsWorld& world, const ViewportSize viewport, const SceneSettings& settings, const float box_half_extent, const float gap,
        const float restitution, const float friction)
    {
        if (box_half_extent <= 0.0f || gap < 0.0f) return;
        
        const StressTestBounds bounds = CreateStressTestBounds(world, viewport, settings);

        const float box_size = box_half_extent * 2.0f;
        const float spacing = box_size + gap;
        
        const float available_width = bounds.right_inner_x - bounds.left_inner_x - 2.0f * gap;
        const float available_height = bounds.world_height - bounds.floor_top - 2.0f * gap;

        const int columns = FitCount(available_width, box_size, gap);
        const int rows = FitCount(available_height, box_size, gap);

        if (columns <= 0 || rows <= 0) return;
        
        // Center the boxes between the left and right walls.
        const float total_width = columns * box_size + (columns - 1) * gap;
        const float first_x = (bounds.left_inner_x + bounds.right_inner_x - total_width) * 0.5f + box_half_extent;

        // The first row is located above the floor.
        const float first_y = bounds.floor_top + gap + box_half_extent;

        for (int row = 0; row < rows; ++row)
        {
            const float y = first_y + static_cast<float>(row) * spacing;

            for (int col = 0; col < columns; ++col)
            {
                const float x = first_x + static_cast<float>(col) * spacing;

                world.CreateBox( {x, y}, 0.0f, {0.0f, 0.0f},  0.0f,  1.0f,
                    box_half_extent, box_half_extent, restitution, friction);
            }
        }
    }

    void SceneBuilder::CreatePressureCircleStressTest(PhysicsWorld& world, const ViewportSize viewport, const SceneSettings& settings, const uint16_t segments,
        const float radius, const float mass, const float friction, const float compliance, const float gap)
    {
        if (segments < 3 || radius <= 0.0f || mass <= 0.0f || gap < 0.0f) return;
        
        const StressTestBounds bounds = CreateStressTestBounds(world, viewport, settings);

        const float circle_diameter = radius * 2.0f;
        const float spacing = circle_diameter + gap;
        
        const float available_width = bounds.right_inner_x - bounds.left_inner_x - 2.0f * gap;
        const float available_height = bounds.world_height - bounds.floor_top - 2.0f * gap;

        const int columns = FitCount(available_width, circle_diameter, gap);
        const int rows = FitCount(available_height, circle_diameter, gap);

        if (columns <= 0 || rows <= 0) return;
        
        // Center the soft bodies between the left and right walls.
        const float total_width = columns * circle_diameter + (columns - 1) * gap;
        const float first_x = (bounds.left_inner_x + bounds.right_inner_x - total_width) * 0.5f + radius;

        // The first row is located above the floor.
        const float first_y = bounds.floor_top + gap + radius;

        for (int row = 0; row < rows; ++row)
        {
            const float y = first_y + static_cast<float>(row) * spacing;

            for (int col = 0; col < columns; ++col)
            {
                const float x = first_x + static_cast<float>(col) * spacing;
                world.CreatePressureSoftCircle( {x, y}, radius, segments, mass, friction, compliance);
            }
        }
    }

    void SceneBuilder::CreateRopeStressTest(PhysicsWorld& world, const ViewportSize viewport, const uint16_t rope_count, const uint16_t segments)
    {
        const float world_w = static_cast<float>(viewport.width) * physics_config::kMpp;

        const float world_h = static_cast<float>(viewport.height) * physics_config::kMpp;

        constexpr float margin = 1.f;
        constexpr float rope_length = 15.f;

        const float usable_width = world_w - margin * 2.f;
        const float top_y = world_h * 0.82f;
        const float bottom_y = top_y - rope_length;

        for (uint16_t i = 0; i < rope_count; ++i)
        {
            const float t = (static_cast<float>(i) + 0.5f) / static_cast<float>(rope_count);

            const float x = margin + usable_width * t;

            const Vec2 start{x, top_y};
            const Vec2 end{x, bottom_y};

            world.CreateRope( start, end, segments, 1.f, 0.5f, 0.00001f, true, false);
        }
    }
}
