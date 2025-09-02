// world.cpp
#include "../include/world.h"
#include <algorithm>
#include <cmath>

int   worldMap[MAP_H][MAP_W];
float wallHeightMap[MAP_H][MAP_W];

// ----------------------- tuning -----------------------
static constexpr float H_WALL        = 2.0f; // outer border
static constexpr float H_TALL        = 2.6f; // run-able interior walls

static constexpr int   SAFE_MARGIN   = 2;    // keep features off the very edge
static constexpr int   END_GAP       = 4;    // how far each long wall stops before the border

// orientation toggle: true = horizontal quarter walls, false = vertical quarter walls
static constexpr bool  HORIZONTAL_WALLS = true;

// --------------------- helpers ------------------------
static inline bool inBounds(int x, int y){
    return (x>=0 && x<MAP_W && y>=0 && y<MAP_H);
}

static inline float defaultHeightFor(int v){
    switch(v){
        case TILE_WALL:      return H_WALL;
        case TILE_TALL_WALL: return H_TALL;
        default:             return 0.0f;
    }
}

static inline void setTile(int x, int y, int t, float h=-1.0f){
    if(!inBounds(x,y)) return;
    worldMap[y][x]      = t;
    wallHeightMap[y][x] = (h>=0.0f ? h : defaultHeightFor(t));
}

static void clearAll(){
    for(int y=0;y<MAP_H;++y)
        for(int x=0;x<MAP_W;++x){
            worldMap[y][x]      = TILE_EMPTY;
            wallHeightMap[y][x] = 0.0f;
        }
}

static void addBorder(){
    for(int x=0;x<MAP_W;++x){ setTile(x,0,TILE_WALL,H_WALL); setTile(x,MAP_H-1,TILE_WALL,H_WALL); }
    for(int y=0;y<MAP_H;++y){ setTile(0,y,TILE_WALL,H_WALL); setTile(MAP_W-1,y,TILE_WALL,H_WALL); }
}

// ------------------- layout builder -------------------
static void buildTwoQuarterWalls(){
    if (HORIZONTAL_WALLS){
        // y at quarters
        const int y1 = MAP_H / 4;
        const int y2 = (3 * MAP_H) / 4;

        // run stops short of borders
        const int x0 = SAFE_MARGIN + END_GAP;
        const int x1 = (MAP_W - 1) - SAFE_MARGIN - END_GAP;

        for (int x = x0; x <= x1; ++x){
            setTile(x, y1, TILE_TALL_WALL, H_TALL);
            setTile(x, y2, TILE_TALL_WALL, H_TALL);
        }
    } else {
        // vertical walls at quarter columns
        const int x1 = MAP_W / 4;
        const int x2 = (3 * MAP_W) / 4;

        // run stops short of borders
        const int y0 = SAFE_MARGIN + END_GAP;
        const int y1 = (MAP_H - 1) - SAFE_MARGIN - END_GAP;

        for (int y = y0; y <= y1; ++y){
            setTile(x1, y, TILE_TALL_WALL, H_TALL);
            setTile(x2, y, TILE_TALL_WALL, H_TALL);
        }
    }
}

// ---------------------- entrypoint ---------------------
void initWorld(){
    clearAll();
    addBorder();
    buildTwoQuarterWalls();

    // No platforms, ramps, diagonals, or pillars — clean arena with just two long walls.
}
