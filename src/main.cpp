#include <iostream>
#include <SDL3/SDL.h>
#include <imgui_impl_sdl3.h>

#include "imgui.h"
#include "imgui_impl_sdlrenderer3.h"
#include "application.h"
#include "physics_config.h"
#include "physics/physics_world.h"
#include "Physics/types.h"

int main(int argc, char* argv[]) {
    if (!SDL_Init(SDL_INIT_VIDEO | SDL_INIT_GAMEPAD)) {
        SDL_Log("SDL_Init Error: %s", SDL_GetError());
        return -1;
    }

    SDL_Window* window = SDL_CreateWindow("SDL3 + ImGui Drawing Demo", 1920, 1080, SDL_WINDOW_RESIZABLE);
    SDL_Renderer* renderer = SDL_CreateRenderer(window, NULL);
    if (!window || !renderer) {
        SDL_Log("Window/Renderer Creation Error: %s", SDL_GetError());
        return -1;
    }
    SDL_SetRenderVSync(renderer, 1);
    
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;
    ImGui::StyleColorsDark();
    
    ImGui_ImplSDL3_InitForSDLRenderer(window, renderer);
    ImGui_ImplSDLRenderer3_Init(renderer);
    
    // Game Setup.
    bool is_running = true;
  
    PhysicsWorld world;
    
    int w, h;
    SDL_GetWindowSize(window, &w, &h);
    
    // Small object for debug (check contact info).
    // world.CreateCircle(Vec2{0.f, 0.f}, 0.f, {0.f, 0.f}, 0.0f, 0.f, 1.f, 0.2f, 1.f); // Small ball.
    // world.CreateBox(Vec2{0, 0 } * physics_config::kMpp, 1.f, {0.f, 0.f}, 0.0f, 0.f, 1.f, 1.f, 1.f); // Small box.

    // Floating obstacle objects.
    world.CreateCircle(Vec2{static_cast<float>(w) / 2.f + 200, static_cast<float>(h) / 2.f } * physics_config::kMpp, 0.0f, {0.f, 0.f}, 0.5f, 0.f, 2.f, 0.2f, 1.f); // Big ball.
    world.CreateBox(Vec2{static_cast<float>(w) / 2.f - 500.f, static_cast<float>(h) / 2.f } * physics_config::kMpp, 0.0f, {0.f, 0.f}, 0.5f, 0.f, 3.f, 3.f, 0.5f, 0.5f); // Big box.
    
    // Floor, Left wall, Right wall.
    world.CreateBox(Vec2{static_cast<float>(w) / 2.f, 50.f } * physics_config::kMpp, 0.f, {0.f, 0.f}, 0.0f, 0.f, static_cast<float>(w) / 2 * physics_config::kMpp, 1.f, 0.5f, 1.f);
    world.CreateBox(Vec2{50, static_cast<float>(h) / 2.f } * physics_config::kMpp, 0.f, {0.f, 0.f}, 0.0f, 0.f, 1.f, static_cast<float>(h) / 2 * physics_config::kMpp, 0.5f, 1.f);
    world.CreateBox(Vec2{static_cast<float>(w) - 50, static_cast<float>(h) / 2.f } * physics_config::kMpp, 0.f, {0.f, 0.f}, 0.0f, 0.f, 1.f, static_cast<float>(h) / 2 * physics_config::kMpp, 0.5f, 1.f);
    
    Uint64 last_time = SDL_GetPerformanceCounter();
    // Game Loop.
    while (is_running) {
        const Uint64 current_time = SDL_GetPerformanceCounter();
        const float delta_time = static_cast<float>(current_time - last_time) / static_cast<float>(SDL_GetPerformanceFrequency());
        last_time = current_time;
        
        Application::Input(window, is_running, world);
        Application::Update(world, delta_time);
        Application::Render(renderer, window, world);
    }
    
    // Quit.
    ImGui_ImplSDLRenderer3_Shutdown();
    ImGui_ImplSDL3_Shutdown();
    ImGui::DestroyContext();

    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();

    return 0;
}