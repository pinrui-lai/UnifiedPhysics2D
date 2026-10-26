#pragma once
#include <random>

#include "physics/types.h"

class PhysicsWorld;

namespace scenes
{
    struct ViewportSize
    {
        int width;
        int height;
    };

    struct SceneSettings
    {
        float floating_angular_velocity = 0.f;
        float floating_restitution = 0.2f;
        float floating_friction = 1.f;

        float boundary_restitution = 0.f;
        float boundary_friction = 0.5f;
    };

    enum class PresetScene : uint8_t
    {
        kDefaultDemo,
        kBoxPyramid,
        kBoxStack,
        kSoftBodyDemo,
        kBoxStressTest,
        kBigPressureCircleStressTest,
        kSmallPressureCircleStressTest,
        kRopeStressTest,
        kEmpty,
    };

    struct SceneObjects
    {
        BodyHandle debug_circle{0xFFFFFFFF};
        BodyHandle debug_box{0xFFFFFFFF};
        
        BodyHandle floating_circle{0xFFFFFFFF};
        BodyHandle floating_circle2{0xFFFFFFFF};

        BodyHandle floating_wide_box1{0xFFFFFFFF};
        BodyHandle floating_wide_box2{0xFFFFFFFF};

        BodyHandle ceiling{0xFFFFFFFF};
        BodyHandle floor{0xFFFFFFFF};
        BodyHandle left_wall{0xFFFFFFFF};
        BodyHandle right_wall{0xFFFFFFFF};
    };

    class SceneBuilder
    {
    public:
        static SceneObjects Create(PresetScene preset, PhysicsWorld& world, ViewportSize viewport, const SceneSettings& settings);
        static float CreateColumn(PhysicsWorld& world, float floor_top, float x, uint32_t box_count, float half_width, float half_height, float mass, float restitution, float friction);

    private:
        static SceneObjects CreateDefaultDemo(PhysicsWorld& world, ViewportSize viewport, const SceneSettings& settings);
        static SceneObjects CreateEmpty(PhysicsWorld& world, ViewportSize viewport, const SceneSettings& settings);
        static SceneObjects CreateBoxPyramid(PhysicsWorld& world, ViewportSize viewport, const SceneSettings& settings);
        static SceneObjects CreateBoxStack(PhysicsWorld& world, ViewportSize viewport, const SceneSettings& settings);
        static SceneObjects CreateSoftBodyDemo(PhysicsWorld& world, ViewportSize viewport, const SceneSettings& settings);
        static void CreateBoxStressTest( PhysicsWorld& world, ViewportSize viewport, const SceneSettings& settings,
            float box_half_extent = 0.5f, float gap = 0.02f, float restitution = 0.05f, float friction = 0.6f);
        static void CreatePressureCircleStressTest( PhysicsWorld& world, ViewportSize viewport, const SceneSettings& settings, 
            uint16_t segments = 64, float radius = 64.f / 30.f, float mass = 10.0f, float friction = 0.5f, float compliance = 0.00000f, float gap = 0.04f);
        
        static void CreateBigPressureCircleStressTest( PhysicsWorld& world, ViewportSize viewport, const SceneSettings& settings, 
            uint16_t segments = 100, float radius = 100.f / 35.f, float mass = 25.0f, float friction = 0.5f, float compliance = 0.00000f, float gap = 0.04f)
        {
            CreatePressureCircleStressTest( world, viewport, settings, segments, radius, mass, friction, compliance, gap);
        }
        
        static void CreateSmallPressureCircleStressTest( PhysicsWorld& world, ViewportSize viewport, const SceneSettings& settings, 
            uint16_t segments = 32, float radius = 32.f / 25.f, float mass = 15.0f, float friction = 0.5f, float compliance = 0.00000f, float gap = 0.04f)
        {
            CreatePressureCircleStressTest( world, viewport, settings, segments, radius, mass, friction, compliance, gap);
        }
        
        static void CreateRopeStressTest( PhysicsWorld& world, ViewportSize viewport, uint16_t rope_count = 30, uint16_t segments = 101);
        
        template <typename CreateFn>
        static auto SpawnFalling( const Vec2& center, const size_t count, const float spread_x, const float height, CreateFn create_fn)
        {
            thread_local std::mt19937 rng{ std::random_device{}() };

            std::uniform_real_distribution<float> x_dist( -spread_x, spread_x );
            std::uniform_real_distribution<float> y_dist( 0.0f, height );

            for (size_t i = 0; i < count; ++i)
            {
                Vec2 pos{ center.x + x_dist(rng), center.y + y_dist(rng) };
                create_fn(pos);
            }
        }
    };
}
