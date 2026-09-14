#pragma once
#include <SDL_events.h>
#include <SDL_render.h>

class PhysicsWorld;

class Application
{
public:
    static void Input(SDL_Event &event, SDL_Window* window, bool &is_running, PhysicsWorld& world);
    static void Update(PhysicsWorld& world, const float dt);
    static void Render(SDL_Renderer* renderer, SDL_Window* window, PhysicsWorld& world);
};
