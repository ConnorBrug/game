#pragma once
#include <cmath>

// === Big world, simple layout ===
constexpr int BASE_W = 24;
constexpr int BASE_H = 24;
#ifndef MAP_TILES
#define MAP_TILES 6  // 6 -> 144x144 (bump to 8 for 192x192)
#endif

constexpr int MAP_W = BASE_W * MAP_TILES;
constexpr int MAP_H = BASE_H * MAP_TILES;

// Toggle lanes on/off (1 = on, 0 = off)
#ifndef ENABLE_WALL_RUN_LANES
#define ENABLE_WALL_RUN_LANES 1
#endif

// Tile IDs
enum : int {
    TILE_EMPTY      = 0,
    TILE_WALL       = 1,
    TILE_PLATFORM   = 2,
    TILE_ALT_WALL   = 3,
    TILE_GOAL       = 4,
    TILE_TALL_WALL  = 5
};

// Global maps
extern int   worldMap[MAP_H][MAP_W];
extern float wallHeightMap[MAP_H][MAP_W];

// Queries
bool  isWallCell(int mx, int my);
bool  isWallAt(double x, double y);
float wallHeightAt(int mx, int my);
float wallHeightAtWorld(double x, double y);

// Build map
void initWorld();

// ---- inline helpers ----
inline bool isWallCell(int mx,int my){
    if(mx<0||my<0||mx>=MAP_W||my>=MAP_H) return true;
    return worldMap[my][mx]!=TILE_EMPTY;
}
inline bool isWallAt(double x,double y){
    int mx = (int)std::floor(x);
    int my = (int)std::floor(y);
    return isWallCell(mx,my);
}
inline float wallHeightAt(int mx,int my){
    if(mx<0||my<0||mx>=MAP_W||my>=MAP_H) return 1.0f;
    return wallHeightMap[my][mx];
}
inline float wallHeightAtWorld(double x,double y){
    int mx = (int)std::floor(x);
    int my = (int)std::floor(y);
    return wallHeightAt(mx,my);
}
