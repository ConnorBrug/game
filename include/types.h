#pragma once
#include <cstdint>

// ---------------- Player ----------------
struct Player {
    double x = 12.0, y = 12.0, dir = 0.0;
    double vx = 0.0, vy = 0.0;
};

// ---------------- Enemy with limb state & size ----------------
struct Enemy {
    double x = 0.0, y = 0.0;
    bool   alive = true;
    double phase = 0.0;

    // Physical size (used everywhere)
    double radius = 0.28;

    // Limb / damage state
    int  bodyHits = 0;         // needs 2 body hits to die
    bool armL = true, armR = true;
    bool legL = true, legR = true;

    double hitFlash = 0.0;     // seconds left of red flash

    // Helpers
    bool bothLegsGone() const { return !legL && !legR; }
    bool oneLegGone()  const { return legL ^ legR; }
    bool bothArmsGone() const { return !armL && !armR; }
    double poseCrawlT  = 0.0;  // 0..1 (both legs gone -> 1)
    double poseTiltDeg = 0.0;  // current visual tilt (deg)
    int    poseTiltSign = +1;  // stable side for tilt
    bool   poseInit     = false;

};

// ---------------- Bullet (constant Z plane) ----------------
struct Bullet {
    double x, y;
    double vx, vy;
    double life;
    double z;
    double zSlope;
    double zLockLeft;   // XY meters left before we “hit” the crosshair plane
    double aimDirX;     // aim ray (player look) at fire time
    double aimDirY;
    bool   snapped;     // whether we already snapped direction to the aim ray
};
