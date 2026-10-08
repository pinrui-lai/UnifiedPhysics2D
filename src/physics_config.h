#pragma once

namespace physics_config {
    inline constexpr float kPpm = 50.0f;              // Pixel per Meter
    inline constexpr float kMpp = 1.0f / kPpm;        // Meter per Pixel
    inline constexpr float kDefaultGravity = -9.8f;    // Default Gravity
    inline constexpr float kTwoParticlesThickness = 0.1f; // The thickness of the two particles
    inline constexpr uint32_t kSubBvhThreshold = 100;
}