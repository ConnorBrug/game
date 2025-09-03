cmake_minimum_required(VERSION 3.20)
project(game CXX)

# If you use vcpkg, make sure you configure CMake with:
# -DCMAKE_TOOLCHAIN_FILE=C:/vcpkg/scripts/buildsystems/vcpkg.cmake

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find SDL2 and SDL2_image from vcpkg
find_package(SDL2 CONFIG REQUIRED)
find_package(SDL2_image CONFIG REQUIRED)

add_executable(game
        src/main.cpp
        src/world.cpp
        src/sprites.cpp
        src/renderer.cpp
        src/font.cpp
        src/ui.cpp
        src/hittest.cpp
        src/pose.cpp
        src/input.cpp
        src/camera.cpp
        src/wallrun.cpp
        src/combat.cpp
)

# Include your headers
target_include_directories(game PRIVATE ${CMAKE_SOURCE_DIR}/include)

# Link imported targets. This automatically picks debug/release libs.
target_link_libraries(game PRIVATE
        SDL2::SDL2 SDL2::SDL2main
        SDL2_image::SDL2_image
)

# (Optional) On Windows, ensure console subsystem
if (MSVC)
    target_link_options(game PRIVATE /SUBSYSTEM:CONSOLE)
endif()
