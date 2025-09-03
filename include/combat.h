#pragma once
#include <vector>

struct Player;
struct Enemy;
struct Bullet;

namespace Combat {

    // Fire a bullet if allowed.
    // Pass `useFov` (the live render FOV) and `rollRad` (effective camera roll in radians).
    bool tryFire(std::vector<Bullet>& bullets, const Player& player,
                 bool triggerDown, double& fireCooldown,
                 int inH, double ppuZ, double pitchRad, double viewZNow,
                 double adsT, double useFov, double rollRad);

    // Step bullets, do collisions, return number of kills this frame.
    int updateBullets(std::vector<Bullet>& bullets, std::vector<Enemy>& enemies,
                      const Player& player, double dt,
                      int inW, int inH, double fov, double ppuZ, int horizonBase,
                      double pitchRad, double wallScaleUsed);

} // namespace Combat
