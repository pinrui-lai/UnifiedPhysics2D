#pragma once

#include <variant>

#include "types.h"

struct PositionComponent
{
    Vec2 val;
};

struct LinearVelocityComponent
{
    Vec2 val;
};

struct PseudoLinearVelocityComponent
{
    Vec2 val;
};

struct PseudoAngularVelocityComponent
{
    float val;
};

struct AngularVelocityComponent
{
    float val;
};

struct RotationComponent
{
    float val;
};

struct Rot {
    float c, s; // c = cos(angle), s = sin(angle)
    
    void Set(const float angle) {
        c = std::cos(angle);
        s = std::sin(angle);
    }

    // Rotation
    Vec2 Mul(const Vec2& v) const {
        return Vec2(c * v.x - s * v.y, s * v.x + c * v.y);
    }

    // Reverse Rotation
    Vec2 MulT(const Vec2& v) const {
        return Vec2(c * v.x + s * v.y, -s * v.x + c * v.y);
    }
};

struct RigidBodyComponent
{
    float inv_mass{1.f};
    float inv_inertia{1.f};
    float restitution{0.1f};
    float friction{0.1f};
};

struct CircleShape
{
    float radius;
};

struct BoxShape
{
    float half_width;
    float half_height;
};

struct PolygonShape
{
    uint16_t vertex_offset;
    uint32_t vertex_count;
};

using Shape = std::variant<PolygonShape, BoxShape, CircleShape>;