#pragma once

namespace physics_config {
    inline constexpr float kPpm = 50.f; // Pixel per Meter
    inline constexpr float kMpp = 1.f / kPpm; // Meter per Pixel
    inline constexpr float kDefaultGravity = -9.8f; // Default Gravity
    inline constexpr float kTwoParticlesThickness = 0.1f; // The thickness of the two particles
    
    // Include collision pairs that are not currently in contact but
    // may come into contact as a result of solving other constraints in subsequent XPBD iterations.
    inline constexpr float kDetectionExpansion = 3.f; 
    
    inline constexpr float kParticleDensityRadius = 25.f;
    inline constexpr float kParticleDensityLenght = 5.f;
}