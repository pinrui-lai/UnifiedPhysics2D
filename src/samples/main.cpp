#include <iostream>
#include <SDL3/SDL.h>
#include <imgui_impl_sdl3.h>

#include "imgui.h"
#include "imgui_impl_sdlrenderer3.h"
#include "application.h"
#include "physics/physics_config.h"
#include "physics/physics_world.h"

int main(int argc, char* argv[]) {
    if (!SDL_Init(SDL_INIT_VIDEO | SDL_INIT_GAMEPAD)) {
        SDL_Log("SDL_Init Error: %s", SDL_GetError());
        return -1;
    }

    SDL_Window* window = SDL_CreateWindow("SDL3 + ImGui Drawing Demo", 1920, 1080, SDL_WINDOW_RESIZABLE);
    SDL_Renderer* renderer = SDL_CreateRenderer(window, nullptr);
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
  
    PhysicsWorld world(physics_config::kDefaultGravity, 4 * 1024 * 1024);
    Application application;
    
    // Create floor, walls and floating obstacles.
    application.CreateInitialBodies(window, world);
    
    const Uint64 target_frame_time = SDL_GetPerformanceFrequency() / 60;
    Uint64 last_time = SDL_GetPerformanceCounter();
    constexpr float max_delta_time = 1.f / 60.f;
    // Game Loop.
    while (is_running) {
        const Uint64 current_time = SDL_GetPerformanceCounter();
        float delta_time = static_cast<float>(current_time - last_time) / static_cast<float>(SDL_GetPerformanceFrequency());
        
        if (delta_time < 1e-6f) continue;
        
        delta_time = std::min(max_delta_time, delta_time);
        last_time = current_time;
        
        application.Input(window, is_running, world);
        Application::Update(world, delta_time);
        application.Render(renderer, window, world);
        
        // Fix frame rate.
        const Uint64 frame_end = SDL_GetPerformanceCounter();
        const Uint64 elapsed = frame_end - current_time;
        
        if (elapsed < target_frame_time)
        {
            // Convert to ms.
            const Uint32 sleep_ms = (target_frame_time - elapsed) * 1000 / SDL_GetPerformanceFrequency();
            if (sleep_ms > 1)
            {
                SDL_Delay(sleep_ms - 1); // Leave some time for busy-waiting to make precise adjustments.
            }
            // remaining time busy-wait to ensure a stable frame rate.
            while ((SDL_GetPerformanceCounter() - current_time) < target_frame_time) 
            {
                // spin lock
            }
        }
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