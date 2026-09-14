#include <iostream>
#include <SDL.h>

#include "application.h"
#include "physics_config.h"
#include "physics/physics_world.h"
#include "Physics/types.h"

int main(int argc, char* argv[]) {
    if (SDL_Init(SDL_INIT_VIDEO) < 0) {
        std::cerr << "SDL Initialization Failed: " << SDL_GetError() << '\n';
        return 1;
    }

    SDL_Window* window = SDL_CreateWindow(
        "Rider SDL2 Window",
        SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
        1920, 1080,
        SDL_WINDOW_SHOWN
    );

    if (!window) {
        std::cerr << "Window creation failed: " << SDL_GetError() << '\n';
        SDL_Quit();
        return 1;
    }

    SDL_Renderer* renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
    
    // Game Setup.
    bool is_running = true;
    SDL_Event event;
    PhysicsWorld world;
    
    int w, h;
    SDL_GetWindowSize(window, &w, &h);
    
    // Small object for debug (check contact info).
    // world.CreateCircle(Vec2{0.f, 0.f}, 0.f, {0.f, 0.f}, 0.0f, 0.f, 1.f, 0.2f, 1.f); // Small ball.
    // world.CreateBox(Vec2{0, 0 } * physics_config::kMpp, 1.f, {0.f, 0.f}, 0.0f, 0.f, 1.f, 1.f, 1.f); // Small box.

    // Floating objects.
    world.CreateCircle(Vec2{static_cast<float>(w) / 2.f + 200, static_cast<float>(h) / 2.f } * physics_config::kMpp, 0.f, {0.f, 0.f}, 0.0f, 0.f, 2.f, 0.2f, 1.f); // Big ball.
    world.CreateBox(Vec2{static_cast<float>(w) / 2.f - 500.f, static_cast<float>(h) / 2.f } * physics_config::kMpp, 0.f, {0.f, 0.f}, 0.0f, 0.f, 3.f, 3.f, 0.5f, 0.5f); // Big box.
    
    // Floor, Left wall, Right wall
    world.CreateBox(Vec2{static_cast<float>(w) / 2.f, 50.f } * physics_config::kMpp, 0.f, {0.f, 0.f}, 0.0f, 0.f, static_cast<float>(w) / 2 * physics_config::kMpp, 1.f, 0.5f, 1.f);
    world.CreateBox(Vec2{50, static_cast<float>(h) / 2.f } * physics_config::kMpp, 0.f, {0.f, 0.f}, 0.0f, 0.f, 1.f, static_cast<float>(h) / 2 * physics_config::kMpp, 0.5f, 1.f);
    world.CreateBox(Vec2{static_cast<float>(w) - 50, static_cast<float>(h) / 2.f } * physics_config::kMpp, 0.f, {0.f, 0.f}, 0.0f, 0.f, 1.f, static_cast<float>(h) / 2 * physics_config::kMpp, 0.5f, 1.f);
    
    Uint64 last_time = SDL_GetPerformanceCounter();
    // Game Loop.
    while (is_running) {
        const Uint64 current_time = SDL_GetPerformanceCounter();
        const float delta_time = static_cast<float>(current_time - last_time) / static_cast<float>(SDL_GetPerformanceFrequency());
        last_time = current_time;
        
        Application::Input(event, window, is_running, world);
        Application::Update(world, delta_time);
        Application::Render(renderer, window, world);
    }
    
    // Quit.
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();

    return 0;
}