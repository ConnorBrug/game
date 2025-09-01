#include "../include/world.h"
#include <algorithm>

int   worldMap[MAP_H][MAP_W];
float wallHeightMap[MAP_H][MAP_W];

// Original 24x24 (unchanged)
static const int baseMap[BASE_H][BASE_W] = {
    {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
    {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
    {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
    {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
    {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
    {1,0,3,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
    {1,0,0,0,0,0,0,0,0,2,2,2,2,0,0,0,0,0,0,0,0,0,0,1},
    {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
    {1,0,3,0,0,0,0,0,0,2,0,0,2,0,0,0,0,0,0,0,0,0,0,1},
    {1,0,3,0,0,0,0,0,0,2,2,2,2,0,0,0,0,0,0,0,0,0,0,1},
    {1,0,3,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
    {1,0,3,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
    {1,0,3,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
    {1,0,3,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
    {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
    {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
    {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
    {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
    {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
    {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
    {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,4,0,0,1},
    {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,4,0,0,1},
    {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,4,0,0,1},
    {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1}
};

// --- simple lane params (sparser than before) ---
static constexpr int   LANE_SPACING = 12;   // was 8 → fewer lanes
static constexpr int   SEGMENT_LEN  = 8;    // longer rhythm, fewer toggles
static constexpr float TALL_FACTOR  = 2.5f; // how tall the tall walls are

static inline float defaultHeightFor(int v){
    switch(v){
        case TILE_WALL:      return 1.4f;
        case TILE_ALT_WALL:  return 1.6f;
        case TILE_PLATFORM:  return 0.6f;
        case TILE_GOAL:      return 1.2f;
        case TILE_TALL_WALL: return TALL_FACTOR;
        default:             return 0.0f;
    }
}

// Vertical wall-run lanes only (no horizontal lanes = ~half the clutter)
static void addWallRunLanesLite(){
    const int margin = 2;
    int startX = margin + 4;

    for(int x = startX, i = 0; x < MAP_W - margin; x += LANE_SPACING, ++i){
        for(int y = 1; y < MAP_H - 1; ++y){
            // alternating solid/gap segments along Y
            bool place = (((y / SEGMENT_LEN) + i) % 2 == 0);
            if(place){
                worldMap[y][x]      = TILE_TALL_WALL;
                wallHeightMap[y][x] = TALL_FACTOR;
            }
        }
    }
}

void initWorld(){
    // 1) Tile base map; remove interior seams
    for(int ty=0; ty<MAP_TILES; ++ty){
        for(int tx=0; tx<MAP_TILES; ++tx){
            for(int y=0; y<BASE_H; ++y){
                for(int x=0; x<BASE_W; ++x){
                    int v = baseMap[y][x];

                    bool edgeX = (x==0 || x==BASE_W-1);
                    bool edgeY = (y==0 || y==BASE_H-1);

                    bool isOuterLeft   = (tx==0           && x==0);
                    bool isOuterRight  = (tx==MAP_TILES-1 && x==BASE_W-1);
                    bool isOuterTop    = (ty==0           && y==0);
                    bool isOuterBottom = (ty==MAP_TILES-1 && y==BASE_H-1);

                    if((edgeX || edgeY) && !(isOuterLeft || isOuterRight || isOuterTop || isOuterBottom)){
                        v = TILE_EMPTY; // clear interior borders
                    }

                    int gx = tx*BASE_W + x;
                    int gy = ty*BASE_H + y;
                    worldMap[gy][gx]      = v;
                    wallHeightMap[gy][gx] = defaultHeightFor(v);
                }
            }
        }
    }

    // 2) Outer border, slightly taller
    for(int y=0; y<MAP_H; ++y){
        worldMap[y][0]         = TILE_WALL;
        worldMap[y][MAP_W-1]   = TILE_WALL;
        wallHeightMap[y][0]         = 2.0f;
        wallHeightMap[y][MAP_W-1]   = 2.0f;
    }
    for(int x=0; x<MAP_W; ++x){
        worldMap[0][x]         = TILE_WALL;
        worldMap[MAP_H-1][x]   = TILE_WALL;
        wallHeightMap[0][x]         = 2.0f;
        wallHeightMap[MAP_H-1][x]   = 2.0f;
    }

    // 3) Sparse, vertical-only lanes
#if ENABLE_WALL_RUN_LANES
    addWallRunLanesLite();
#endif
}
