#include "../include/combat.h"

#include <algorithm>
#include <cmath>
#include <corecrt_math_defines.h>

#include "../include/config.h"
#include "../include/world.h"
#include "../include/hittest.h"

// ==== helpers ====
static inline double clamp01(double t) { return (t < 0.0 ? 0.0 : (t > 1.0 ? 1.0 : t)); }
static inline double lerp(double a, double b, double t) { return a + (b - a) * clamp01(t); }
static inline double wrapTwoPi(double a) { return std::remainder(a, 2.0 * M_PI); }

// Sweep camera->muzzle to avoid spawning bullets inside walls (common in wall-run).
static inline void sweepToFreePoint(double ox, double oy, double tx, double ty,
                                    double& outX, double& outY)
{
    const double dx = tx - ox, dy = ty - oy;
    const double dist = std::hypot(dx, dy);
    if (dist <= 1e-6) { outX = ox; outY = oy; return; }

    const double step = 0.02; // ≈2cm
    const int steps = std::max(1, (int)std::ceil(dist / step));

    for (int i = 1; i <= steps; ++i) {
        const double t = double(i) / steps;
        const double x = ox + dx * t;
        const double y = oy + dy * t;
        if (isWallAt(x, y)) {
            const double tBack = double(i - 1) / steps;
            outX = ox + dx * tBack;
            outY = oy + dy * tBack;
            // Tiny retreat toward camera to avoid immediate re-hit
            const double bx = outX - dx * (1.5 / std::max(1.0, (double) steps));
            const double by = outY - dy * (1.5 / std::max(1.0, (double) steps));
            if (!isWallAt(bx, by)) { outX = bx; outY = by; }
            return;
        }
    }
    outX = tx; outY = ty;
}

// -----------------------------------------------------------------------------
// Firing model (final):
//
// - Offsets authored in camera space (Right/Down).
// - **Roll-only vertical tweak**: keep base side offset unchanged; only rotate the
//   vertical (“below crosshair”) vector by roll, so "1 mm below" becomes a
//   45° diagonal at 45° roll without shrinking the side offset.
// - zCross math matches renderer (live FOV + proj-pitch tanSafe).
// - Sweep camera→muzzle to ensure muzzle never starts inside a wall.
// -----------------------------------------------------------------------------

bool Combat::tryFire(std::vector<Bullet>& bullets, const Player& player,
                     bool triggerDown, double& fireCooldown,
                     int inH, double ppuZ, double pitchRad, double viewZNow,
                     double adsT, double useFov, double rollRad)
{
    if (!triggerDown || fireCooldown > 0.0) return false;

    // View basis (yaw-only) for world XY
    const double fxc = std::cos(player.dir);
    const double fyc = std::sin(player.dir);
    const double rxc = std::cos(player.dir + 1.5707963267948966);
    const double ryc = std::sin(player.dir + 1.5707963267948966);

    // ADS-aware forward push
    const double muzzleFwd = MUZZLE_DIST * lerp(1.0, MUZZLE_DIST_ADS_MUL, adsT);

    // Camera-space offsets
    const double ads = clamp01(adsT);
    const double side_s = MUZZLE_SIDE * (1.0 - ads); // base side offset collapses in ADS
    const double zOff_s = MUZZLE_Z_OFF_HIP + (MUZZLE_Z_OFF_ADS - MUZZLE_Z_OFF_HIP) * ads; // screen-down

    // Roll-only vertical tweak:
    // Keep base side_s as-is; rotate only the vertical vector by roll, and add its side component.
    const double cR = std::cos(rollRad), sR = std::sin(rollRad);
    const double side_w =  side_s  - zOff_s * sR; // base side + rotated vertical’s side component
    const double zOff_w  =             zOff_s * cR; // vertical projected to world “down”

    // Candidate muzzle XY (from camera), then sweep to stay out of walls
    const double camX = player.x, camY = player.y;
    const double candX = camX + fxc * muzzleFwd + rxc * side_w;
    const double candY = camY + fyc * muzzleFwd + ryc * side_w;

    double bx = candX, by = candY;
    sweepToFreePoint(camX, camY, candX, candY, bx, by);

    // ---- Crosshair plane world-Z (match renderer) ----
    const double pitchWrapped = wrapTwoPi(pitchRad);
    const bool   upsideDown   = std::cos(pitchWrapped) < 0.0;
    const double projPitch    = upsideDown
        ? (pitchWrapped > 0.0 ? pitchWrapped - M_PI : pitchWrapped + M_PI)
        : pitchWrapped;

    const double sP = std::sin(projPitch);
    const double cP = std::cos(projPitch);
    const double eps = 1e-4;
    const double tanSafe = sP / std::max(std::abs(cP), eps);

    const double fy = (double(inH) * 0.5) / std::tan(useFov * 0.5); // same as renderer
    const double pitchPx = fy * tanSafe;
    const double dZ = pitchPx / ppuZ;

    const double zCross  = viewZNow + dZ;
    const double zMuzzle = zCross + zOff_w; // visually "down from crosshair", roll-aware

    // ---- Convergence along look direction (yaw) ----
    const double convHip = std::max(0.25, MUZZLE_CONVERGE_M);
    const double convAds = std::max(convHip, MUZZLE_CONVERGE_M * CONVERGE_ADS_MUL);
    const double conv    = lerp(convHip, convAds, ads);

    const double tx = player.x + fxc * conv;
    const double ty = player.y + fyc * conv;

    // Direction from muzzle to converge point in XY
    double dirX = tx - bx, dirY = ty - by;
    double len  = std::hypot(dirX, dirY);
    if (len < 1e-6) { dirX = fxc; dirY = fyc; len = 1.0; }
    dirX /= len; dirY /= len;

    // XY velocity + optional player velocity inheritance
    const double vx = dirX * BULLET_SPEED + player.vx * BULLET_INHERIT_VEL;
    const double vy = dirY * BULLET_SPEED + player.vy * BULLET_INHERIT_VEL;

    // Vertical slope so after traveling 'len' we hit the crosshair plane
    const double zSlope = (zCross - zMuzzle) / len; // equals (-zOff_w)/len → no pitch creep

    // Spawn bullet
    bullets.push_back({
        bx, by,                      // x, y
        vx, vy,                      // vx, vy
        BULLET_LIFE_SEC,             // life
        zMuzzle,                     // z
        zSlope,                      // zSlope
        len,                         // zLockLeft
        fxc, fyc,                    // aimDirX / aimDirY
        false                        // snapped
    });

    fireCooldown = FIRE_DELAY;
    return true;
}

int Combat::updateBullets(std::vector<Bullet>& bullets, std::vector<Enemy>& enemies,
                          const Player& player, double dt,
                          int inW, int inH, double fov, double ppuZ, int horizonBase,
                          double pitchRad, double wallScaleUsed)
{
    using HitZone = HitTest::Zone;
    int hits = 0;

    HitTest::Context hrc{ inW, inH, fov, ppuZ, horizonBase, pitchRad, wallScaleUsed, &player };

    // Precompute frame-projection basis (same as HitTest::screenFor)
    const double dirX   = std::cos(player.dir), dirY = std::sin(player.dir);
    const double planeX = std::cos(player.dir + 1.5707963267948966) * std::tan(fov * 0.5);
    const double planeY = std::sin(player.dir + 1.5707963267948966) * std::tan(fov * 0.5);
    const double invDet = 1.0 / (planeX * dirY - dirX * planeY);

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
        const double stepLen = (steps > 0) ? (dist / steps) : 0.0;

        double zCur = b.z;

        for (int s = 1; s <= steps && !remove; ++s) {
            const double t  = double(s) / steps;
            const double px = sx + dx * t;
            const double py = sy + dy * t;

            // Advance Z only up to the remaining lock distance
            if (b.zLockLeft > 0.0) {
                const double adv = std::min(stepLen, b.zLockLeft);
                zCur        += b.zSlope * adv;
                b.zLockLeft -= adv;

                // Reached crosshair plane → lock Z and snap XY direction
                if (b.zLockLeft <= 0.0) {
                    b.zSlope = 0.0;
                    if (!b.snapped) {
                        const double sp = std::max(1e-6, std::hypot(b.vx, b.vy));
                        b.vx = b.aimDirX * sp;
                        b.vy = b.aimDirY * sp;
                        b.snapped = true;
                    }
                }
            }

            const double zNow = zCur;
            const int bulletY = horizonBase - (int)std::lround(zNow * ppuZ);

            // walls
            if (isWallAt(px, py)) { remove = true; break; }

            // FLOOR CULL at this step's depth (allow a tiny near-ground pass)
            const double rx = px - player.x, ry = py - player.y;
            const double depth = invDet * (-planeY * rx + planeX * ry);
            if (depth > 0.0) {
                const int colH = (int)std::abs((inH * wallScaleUsed * std::cos(pitchRad))
                                               / std::max(0.2, depth));
                const int groundY = horizonBase + colH / 2;
                if (bulletY >= groundY + 1 && zNow <= BULLET_GROUND_EPS) { remove = true; break; }
            }

            // enemies (use zNow)
            for (auto& en : enemies) {
                if (!en.alive) continue;

                double rr = std::max(en.radius * LIMB_PRUNE_MUL, LIMB_PRUNE_MIN)
                            + BULLET_RADIUS + BULLET_HIT_PAD;
                double ex = en.x - px, ey = en.y - py;
                if (ex * ex + ey * ey > rr * rr) continue;

                HitZone hz = HitTest::classify(hrc, en, zNow, px, py);
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
            b.z = zCur;

            b.life -= dt;
            if (b.life <= 0.0) remove = true;
        }

        if (remove) bullets.erase(bullets.begin() + i);
        else        ++i;
    }

    return hits;
}
