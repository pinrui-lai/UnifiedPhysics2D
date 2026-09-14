#include <SDL2/SDL2_gfxPrimitives.h>

#include "graphics.h"
#include "Physics/types.h"

void Graphics::DrawRotatedCircle(SDL_Renderer* renderer, Vec2 center, float radius, float radians, Uint8 r,
                                 Uint8 g, Uint8 b, Uint8 a, int lineThickness)
{
    // Circle outline.
    circleRGBA(renderer, static_cast<Sint16>(center.x), static_cast<Sint16>(center.y), static_cast<Sint16>(radius), r, g, b, a);

    // Points on the circumference.
    Vec2 edgePoint = {center.x + radius * std::cos(radians), center.y + radius * std::sin(radians)};

    // Line from center to circumference.
    thickLineRGBA(
        renderer, 
        static_cast<Sint16>(center.x), static_cast<Sint16>(center.y), 
        static_cast<Sint16>(edgePoint.x), static_cast<Sint16>(edgePoint.y), 
        lineThickness, 
        r, g, b, a
    );
}

void Graphics::DrawPolygon(SDL_Renderer* renderer, const std::vector<Vec2>& vertices, Uint8 r, Uint8 g, Uint8 b,
    Uint8 a)
{
    if (vertices.size() < 3) return; // Need at least 3 vertices. 

    // Get Sint16 X and Y vectors for SDL2_gfx. 
    std::vector<Sint16> vx;
    std::vector<Sint16> vy;
    vx.reserve(vertices.size());
    vy.reserve(vertices.size());

    for (const auto& v : vertices) {
        vx.push_back(static_cast<Sint16>(v.x));
        vy.push_back(static_cast<Sint16>(v.y));
    }

    polygonRGBA(
        renderer, 
        vx.data(), 
        vy.data(), 
        static_cast<int>(vertices.size()), 
        r, g, b, a
    );
}

void Graphics::DrawFilledPolygon(SDL_Renderer* renderer, const std::vector<Vec2>& vertices, Uint8 r, Uint8 g, Uint8 b,
    Uint8 a)
{
    if (vertices.size() < 3) return;

    std::vector<Sint16> vx;
    std::vector<Sint16> vy;
    vx.reserve(vertices.size());
    vy.reserve(vertices.size());

    for (const auto& v : vertices) {
        vx.push_back(static_cast<Sint16>(v.x));
        vy.push_back(static_cast<Sint16>(v.y));
    }

    filledPolygonRGBA(
        renderer, 
        vx.data(), 
        vy.data(), 
        static_cast<int>(vertices.size()), 
        r, g, b, a
    );
}
