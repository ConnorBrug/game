// wallrun.cpp
#include "../include/wallrun.h"
#include <algorithm>
#include <cmath>

// --- local helpers (collision sampling used for conservative movement) ---
static inline bool blockedAt(double x, double y, double rad) {
    if (isWallAt(x, y)) return true;
    return isWallAt(x + rad, y) || isWallAt(x - rad, y) || isWallAt(x, y + rad) || isWallAt(x, y - rad);
}
static inline void tryMove(Player& pl, double& px, double& py, double rad) {
    if (!blockedAt(px, pl.y, rad)) pl.x = px; else { px = pl.x; pl.vx = 0.0; }
    if (!blockedAt(pl.x, py, rad)) pl.y = py; else { py = pl.y; pl.vy = 0.0; }
}
static inline void popOffWall(Player& pl, double nx, double ny, double extra = 0.02) {
    // tiny positional nudge off the surface so we don't immediately re-collide
    const double pop = (WallRun::R_NORMAL - WallRun::R_RUN) + extra;
    pl.x += nx * pop; pl.y += ny * pop;
}

// ---------------- soft auto-detach tuning ----------------
// Outward push is expressed as a fraction of JUMP_PUSH (smaller than a wall-jump)
static constexpr double DETACH_PUSH_SCALE_HITMISS = 0.55; // wall ended / miss ahead
static constexpr double DETACH_PUSH_SCALE_IDLE    = 0.40; // idle/slow auto-detach

// How much of along-wall velocity we carry into free space
static constexpr double DETACH_TANGENT_CARRY      = 0.85;

// Small upward pops (well below a true jump)
// Tune +/- 1.0 to taste; keep << your main jump velocity
static constexpr double DETACH_UP_VEL_HITMISS     = 5.5;
static constexpr double DETACH_UP_VEL_IDLE        = 3.5;

// Camera pitch damping on auto-detach — slightly softer than a hard detach
static constexpr double DETACH_PITCH_DAMP_SCALE   = 0.65;

// (Optional) wall-jump vertical speed (kept moderate; your main jump lives in main.cpp)
static constexpr double WALL_JUMP_VEL             = 8.6;

// Soft auto-detach: carry tangent motion, add small outward push, tiny vertical pop
static inline void softDetach(
    WallRunState& s, Player& player, double& vZ,
    double nx, double ny, double tanX, double tanY, double alongVel,
    double pushScale, double carryMul, double upVel,
    double& outDetachPitchDamp)
{
    // step a hair off the wall to avoid immediate re-contact
    popOffWall(player, nx, ny, 0.03);

    // carry tangent velocity so motion feels continuous
    double vx = tanX * alongVel * carryMul;
    double vy = tanY * alongVel * carryMul;

    // gentle outward impulse (much smaller than a wall-jump)
    vx += nx * (WallRun::JUMP_PUSH * pushScale);
    vy += ny * (WallRun::JUMP_PUSH * pushScale);

    // apply soft cap consistent with jump detaches
    double sp = std::hypot(vx, vy);
    if (sp > WallRun::DETACH_SPEED_CAP) {
        const double sc = WallRun::DETACH_SPEED_CAP / sp;
        vx *= sc; vy *= sc;
    }
    player.vx = vx; player.vy = vy;

    // tiny upward pop (never exceeds a real jump)
    vZ = std::max(vZ, upVel);

    // slightly lighter camera damping than the hard detach
    outDetachPitchDamp = WallRun::POST_DETACH_PITCH_DAMP_SEC * DETACH_PITCH_DAMP_SCALE;

    // clear wall-run motion & timers; brief relatch lock
    s.alongVel = 0.0; s.idleTimer = 0.0;
    s.relatchLock = 0.12;
}

// ---------------- API impl ----------------

Hit WallRun::probe(double x, double y) {
    Hit h;
    bool wL = isWallAt(x - DETECT_DIST, y),
         wR = isWallAt(x + DETECT_DIST, y),
         wU = isWallAt(x, y - DETECT_DIST),
         wD = isWallAt(x, y + DETECT_DIST);
    if (wL || wR || wU || wD) {
        h.hit = true;
        h.nx = (wL ? 1.0 : 0.0) + (wR ? -1.0 : 0.0);
        h.ny = (wU ? 1.0 : 0.0) + (wD ? -1.0 : 0.0);
        double len = std::hypot(h.nx, h.ny);
        if (len > 0.0) { h.nx /= len; h.ny /= len; }
    }
    return h;
}

bool WallRun::canStart(const Hit& h, double ix, double iy, const WallRunState& s) {
    if (!h.hit) return false;
    if (s.relatchLock > 0.0) return false;
    const double tx = -h.ny, ty = h.nx;
    const double along  = ix * tx + iy * ty;
    const double toward = ix * h.nx + iy * h.ny;
    return (std::fabs(along) > 0.35) && (toward < 0.45);
}

void WallRun::start(WallRunState& s, const Hit& h, Player& player, float& yawVel, float& pitchVel) {
    s.normalX = h.nx; s.normalY = h.ny;
    s.tanX = -h.ny;   s.tanY   = h.nx;
    s.sticky = STICKY_TIME;

    const double proj = player.vx * s.tanX + player.vy * s.tanY;
    s.alongVel = std::clamp(proj, -LATCH_MAXINIT, LATCH_MAXINIT);

    yawVel = 0.0f; pitchVel = 0.0f;
}

ParkourState WallRun::update(
    WallRunState& s, Player& player, double dt,
    double pZ, double& vZ, bool jumpPressed, double ix, double iy,
    int& jumpsLeft, double& momentum, double& outDetachPitchDamp)
{
    jumpsLeft = 2;

    // Near ground → drop to Normal (use a very soft outward carry; no vertical pop)
    if (pZ <= 0.08 && vZ <= 0.0) {
        softDetach(s, player, vZ,
                   s.normalX, s.normalY, s.tanX, s.tanY, s.alongVel,
                   /*pushScale*/ 0.30, /*carryMul*/ 0.80, /*upVel*/ 0.0,
                   outDetachPitchDamp);
        return ParkourState::Normal;
    }

    // Re-probe wall; allow slight ahead probe so we don't drop on tiny gaps
    Hit h = probe(player.x - s.normalX * 0.10, player.y - s.normalY * 0.10);
    if (!h.hit) {
        const double aheadX = player.x + s.tanX * 0.30;
        const double aheadY = player.y + s.tanY * 0.30;
        Hit h2 = probe(aheadX, aheadY);
        if (h2.hit) { h = h2; s.sticky = STICKY_TIME; }
    } else {
        s.sticky = STICKY_TIME;
    }

    // If wall is gone → smooth auto-detach with slight pop & carry
    if (!h.hit) {
        softDetach(s, player, vZ,
                   s.normalX, s.normalY, s.tanX, s.tanY, s.alongVel,
                   DETACH_PUSH_SCALE_HITMISS, DETACH_TANGENT_CARRY, DETACH_UP_VEL_HITMISS,
                   outDetachPitchDamp);
        return ParkourState::Normal;
    }

    // Update contact frame
    s.normalX = h.nx; s.normalY = h.ny;
    s.tanX = -h.ny;   s.tanY   = h.nx;

    // Input along tangent
    const double inAlong = ix * s.tanX + iy * s.tanY;

    // alongVel update (accelerate/brake/friction)
    if (inAlong * s.alongVel < -0.01) {
        double sp = std::fabs(s.alongVel);
        double ns = std::max(0.0, sp - BRAKE * dt);
        s.alongVel = (s.alongVel > 0 ? 1 : -1) * ns;
    } else {
        if (std::fabs(inAlong) > 0.05) s.alongVel += inAlong * ACCEL * dt;
        else {
            double sp = std::fabs(s.alongVel);
            double ns = std::max(0.0, sp - FRICTION * dt);
            s.alongVel = (s.alongVel > 0 ? 1 : -1) * ns;
        }
    }
    s.alongVel = std::clamp(s.alongVel, -MAX_SPEED, MAX_SPEED);

    // Move along wall; slight pull into it
    double px = player.x + s.tanX * s.alongVel * dt;
    double py = player.y + s.tanY * s.alongVel * dt;
    px -= h.nx * STICK_PULL;
    py -= h.ny * STICK_PULL;

    // Kill into-wall normal velocity; align to tangent
    const double vNorm = player.vx * h.nx + player.vy * h.ny;
    player.vx -= vNorm * h.nx;
    player.vy -= vNorm * h.ny;
    player.vx  = s.tanX * s.alongVel;
    player.vy  = s.tanY * s.alongVel;

    tryMove(player, px, py, R_RUN);

    // Slide down along the wall
    vZ -= GRAVITY * SLIDE_GRAVITY * dt;
    if (vZ < SLIDE_MAX_FALL) vZ = SLIDE_MAX_FALL;

    // Auto detach if idle / too slow & falling (smooth version)
    const double alongInput = std::fabs(inAlong);
    const double alongSpeed = std::fabs(s.alongVel);
    if (alongInput < 0.03 && alongSpeed < 0.15) s.idleTimer += dt; else s.idleTimer = 0.0;

    if (s.idleTimer > IDLE_DETACH_SEC ||
        (std::fabs(inAlong) < 0.02 && std::fabs(s.alongVel) < 0.20 && vZ < -0.5))
    {
        softDetach(s, player, vZ,
                   s.normalX, s.normalY, s.tanX, s.tanY, s.alongVel,
                   DETACH_PUSH_SCALE_IDLE, DETACH_TANGENT_CARRY, DETACH_UP_VEL_IDLE,
                   outDetachPitchDamp);
        return ParkourState::Normal;
    }

    // --- Wall jump (preserve tangent momentum; outward push; small pop off) ---
    if (jumpPressed) {
        vZ = WALL_JUMP_VEL; // keep this below your main jump if you want contrast

        // Preserve current tangent speed; add outward push
        player.vx += h.nx * JUMP_PUSH;
        player.vy += h.ny * JUMP_PUSH;

        // Small positional pop so we don't re-collide with the wall
        player.x += h.nx * 0.08;
        player.y += h.ny * 0.08;

        // Soft cap that still allows fast exits
        double sp = std::hypot(player.vx, player.vy);
        if (sp > DETACH_SPEED_CAP) {
            const double sc = DETACH_SPEED_CAP / sp;
            player.vx *= sc; player.vy *= sc;
        }

        s.alongVel = 0.0; s.idleTimer = 0.0; s.relatchLock = 0.28;
        jumpsLeft = 1;

        // Give a small momentum bump; clamp to your max
        momentum = std::min(1.8, momentum + 0.20);

        outDetachPitchDamp = POST_DETACH_PITCH_DAMP_SEC;
        return ParkourState::Normal;
    }

    return ParkourState::WallRun;
}
