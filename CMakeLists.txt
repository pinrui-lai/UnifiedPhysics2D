cmake_minimum_required(VERSION 3.20)
project(PhysicsEngine2D CXX)

set(CMAKE_CXX_STANDARD 20)

find_package(SDL2 REQUIRED)

find_path(SDL2_GFX_INCLUDE_DIR SDL2_gfxPrimitives.h
        PATH_SUFFIXES include/SDL2 include
) 

find_library(SDL2_GFX_LIBRARY NAMES SDL2_gfx
        PATH_SUFFIXES lib
)

file(GLOB_RECURSE ALL_SOURCES
        "src/*.cpp"
        "src/*.h"
)

add_executable(PhysicsEngine2D ${ALL_SOURCES})

target_include_directories(PhysicsEngine2D PRIVATE
        ${CMAKE_SOURCE_DIR}/src/physics
        ${CMAKE_SOURCE_DIR}/src/physics/systems
        ${SDL2_INCLUDE_DIRS}
        ${SDL2_GFX_INCLUDE_DIR}
)

target_link_libraries(PhysicsEngine2D PRIVATE
        SDL2::SDL2main
        SDL2::SDL2
        ${SDL2_GFX_LIBRARY}
)