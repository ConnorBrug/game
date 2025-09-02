#pragma once
#include <vector>
#include "types.h"
#include "hittest.h"

namespace Combat {
    inline constexpr double FIRE_DELAY   = 0.12;  // matches your main
    inline constexpr double MUZZLE_DIST  = 0.35;

    // Returns true if fired; updates fireCooldown
    bool tryFire(std::vector<Bullet>& bullets, const Player& player,
                 bool triggerDown, double& fireCooldown,
                 int inH, double ppuZ, double pitchRad, double viewZNow);

    // Returns number of hits this frame; updates enemies/bullets; consumes bullets on hit
    int updateBullets(std::vector<Bullet>& bullets, std::vector<Enemy>& enemies,
                      const Player& player, double dt,
                      int inW, int inH, double fov, double ppuZ, int horizonBase);
}
