#pragma once
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

    enum class PresetScene
    {
        kDefaultDemo,
        kBoxPyramid,
        kBoxStack,
        kSoftBodyDemo,
        kBoxStressTest,
        kPressureCircleStressTest,
        kRopeStressTest,
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

    private:
        static SceneObjects CreateDefaultDemo(PhysicsWorld& world, ViewportSize viewport, const SceneSettings& settings);
        static SceneObjects CreateBoxPyramid(PhysicsWorld& world, ViewportSize viewport, const SceneSettings& settings);
        static SceneObjects CreateBoxStack(PhysicsWorld& world, ViewportSize viewport, const SceneSettings& settings);
        static SceneObjects CreateSoftBodyDemo(PhysicsWorld& world, ViewportSize viewport, const SceneSettings& settings);
        static void CreateBoxStressTest( PhysicsWorld& world, ViewportSize viewport, const SceneSettings& settings,
            float box_half_extent = 0.5f, float gap = 0.02f, float restitution = 0.05f, float friction = 0.6f);
        static void CreatePressureCircleStressTest( PhysicsWorld& world, ViewportSize viewport, const SceneSettings& settings, 
            uint16_t segments = 25, float radius = 1.0f, float mass = 10.0f, float friction = 0.5f, float compliance = 0.00000f, float gap = 0.04f);
        static void CreateRopeStressTest( PhysicsWorld& world, ViewportSize viewport, uint16_t rope_count = 30, uint16_t segments = 101);
    };
}
