#pragma once

#include <vector>
#include <SDL_render.h>

struct Vec2;

class Graphics
{
public:
    static void DrawRotatedCircle(SDL_Renderer* renderer, Vec2 center, float radius, float radians, Uint8 r, Uint8 g , Uint8 b, Uint8 a, int lineThickness = 2);
    static void DrawPolygon(SDL_Renderer* renderer, const std::vector<Vec2>& vertices, Uint8 r, Uint8 g , Uint8 b, Uint8 a);
    static void DrawFilledPolygon(SDL_Renderer* renderer, const std::vector<Vec2>& vertices, Uint8 r, Uint8 g , Uint8 b, Uint8 a);
    
};
