cmake_minimum_required(VERSION 3.20)
project(PhysicsEngine2D CXX)

set(CMAKE_CXX_STANDARD 20)

find_package(SDL3 CONFIG REQUIRED)
find_package(imgui CONFIG REQUIRED)

file(GLOB_RECURSE ALL_SOURCES
        "src/*.cpp"
        "src/*.h"
)

add_executable(PhysicsEngine2D ${ALL_SOURCES})

target_include_directories(PhysicsEngine2D PRIVATE
        ${CMAKE_SOURCE_DIR}/src/physics
        ${CMAKE_SOURCE_DIR}/src/physics/systems
)

target_link_libraries(PhysicsEngine2D PRIVATE
        SDL3::SDL3
        imgui::imgui
)