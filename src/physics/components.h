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