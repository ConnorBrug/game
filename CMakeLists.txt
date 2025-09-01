cmake_minimum_required(VERSION 3.16)
project(microdoom LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# vcpkg provides an SDL2 CMake config; with your CMake options:
#   -DCMAKE_TOOLCHAIN_FILE=C:/vcpkg/scripts/buildsystems/vcpkg.cmake -DVCPKG_TARGET_TRIPLET=x64-windows
find_package(SDL2 CONFIG REQUIRED)

add_executable(microdoom src/main.cpp
        include/config.h
        include/types.h
        include/framebuffer.h
        include/util.h
        include/world.h
        src/world.cpp
        include/font.h
        include/ui.h
        include/renderer.h
        src/sprites.cpp
        include/sprites.h
        src/renderer.cpp
        src/font.cpp
        src/ui.cpp
        include/hittest.h
        src/hittest.cpp
        include/pose.h
        src/pose.cpp
)

# Link SDL2 (and SDL2main if it exists on this platform/toolchain)
if (TARGET SDL2::SDL2main)
    target_link_libraries(microdoom PRIVATE SDL2::SDL2main SDL2::SDL2)
else()
    target_link_libraries(microdoom PRIVATE SDL2::SDL2)
endif()
