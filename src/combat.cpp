#include "../include/combat.h"
#include "../include/config.h"
#include "../include/world.h"
#include <cmath>

bool Combat::tryFire(std::vector<Bullet>& bullets, const Player& player,
                     bool triggerDown, double& fireCooldown,
                     int inH, double ppuZ, double pitchRad, double viewZNow)
{
    if (!triggerDown || fireCooldown > 0.0) return false;

    const double fxc = std::cos(player.dir);
    const double fyc = std::sin(player.dir);

    const double bx = player.x + fxc * MUZZLE_DIST;
    const double by = player.y + fyc * MUZZLE_DIST;

    const double zAtCrosshairWorld = viewZNow + ((double)inH * 0.5 / ppuZ) * std::tan(pitchRad);

    bullets.push_back({
        bx, by,
        std::cos(player.dir) * BULLET_SPEED,
        std::sin(player.dir) * BULLET_SPEED,
        BULLET_LIFE_SEC,
        zAtCrosshairWorld,
        0.0
    });

    fireCooldown = FIRE_DELAY;
    return true;
}

int Combat::updateBullets(std::vector<Bullet>& bullets, std::vector<Enemy>& enemies,
                          const Player& player, double dt,
                          int inW, int inH, double fov, double ppuZ, int horizonBase)
{
    using HitZone = HitTest::Zone;
    int hits = 0;

    HitTest::Context hrc{ inW, inH, fov, ppuZ, horizonBase, 0.0, &player }; // pitch is only used in horizon/persp (already baked)

    for (size_t i = 0; i < bullets.size();) {
        Bullet& b = bullets[i];

        const double sx = b.x,  sy = b.y;
        const double nx = b.x + b.vx * dt;
        const double ny = b.y + b.vy * dt;

        bool remove = false;

        const double dx = nx - sx, dy = ny - sy;
        const double dist = std::hypot(dx, dy);
        const double maxStep = std::max(1e-6, (BULLET_RADIUS + BULLET_HIT_PAD) * 0.6);
        const int steps = std::max(1, (int)std::ceil(dist / maxStep));

        const double zFixed = b.z;

        for (int s = 1; s <= steps && !remove; ++s) {
            const double t  = double(s) / steps;
            const double px = sx + dx * t;
            const double py = sy + dy * t;

            if (isWallAt(px, py)) { remove = true; break; }

            for (auto& en : enemies) {
                if (!en.alive) continue;

                double rr = std::max(en.radius * LIMB_PRUNE_MUL, LIMB_PRUNE_MIN) + BULLET_RADIUS + BULLET_HIT_PAD;
                double ex = en.x - px, ey = en.y - py;
                if (ex * ex + ey * ey > rr * rr) continue;

                HitZone hz = HitTest::classify(hrc, en, zFixed, px, py);
                if (hz == HitZone::None) continue;

                switch (hz) {
                    case HitZone::Head:
                        en.alive = false; en.hitFlash = ENEMY_HIT_FLASH_SEC; ++hits; break;
                    case HitZone::Body:
                        en.bodyHits++; en.hitFlash = ENEMY_HIT_FLASH_SEC;
                        if (en.bodyHits >= BODY_HITS_TO_KILL) { en.alive = false; ++hits; }
                        break;
                    case HitZone::ArmL: if (en.armL) { en.armL = false; en.hitFlash = ENEMY_HIT_FLASH_SEC; } break;
                    case HitZone::ArmR: if (en.armR) { en.armR = false; en.hitFlash = ENEMY_HIT_FLASH_SEC; } break;
                    case HitZone::LegL: if (en.legL) { en.legL = false; en.hitFlash = ENEMY_HIT_FLASH_SEC; } break;
                    case HitZone::LegR: if (en.legR) { en.legR = false; en.hitFlash = ENEMY_HIT_FLASH_SEC; } break;
                    default: break;
                }
                remove = true;
                break;
            }
        }

        if (!remove) {
            b.x = nx; b.y = ny;
            b.life -= dt;
            if (b.life <= 0.0) remove = true;
        }

        if (remove) bullets.erase(bullets.begin() + i);
        else        ++i;
    }
    return hits;
}
