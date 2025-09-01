#pragma once
#include <cstdint>
#include <vector>

// Stickman / bullet texture sizes
extern const int SM_W;
extern const int SM_H;
extern const int BL_W;
extern const int BL_H;

// Textures
extern std::vector<uint32_t> g_stickman;  // RGBA sprite
extern std::vector<uint32_t> g_bullet;    // RGBA bullet

// Per-texel mask used for hit classification
// Labels must stay stable across TU's.
enum : uint8_t {
 SM_None = 0,
 SM_Head = 1,
 SM_Body = 2,
 SM_ArmL = 3,
 SM_ArmR = 4,
 SM_LegL = 5,
 SM_LegR = 6
};
extern std::vector<uint8_t> g_stickman_mask;

// Build all sprite assets & masks
void buildSprites();
