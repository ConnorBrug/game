#pragma once
#include <algorithm>
#include <cmath>
#include "types.h"
#include "world.h"
#include "parkour_state.h"

// Simple hit result for wall probes
struct Hit { bool hit=false; double nx=0, ny=0; };

// Runtime wall-run state
struct WallRunState {
    double normalX=0, normalY=0;
    double tanX=0,    tanY=0;
    double alongVel=0;
    double sticky=0;
    double relatchLock=0;
    double idleTimer=0;
};

namespace WallRun {
    // Tunables
    inline constexpr double DETECT_DIST       = 0.33;
    inline constexpr double ACCEL             = 26.0;
    inline constexpr double BRAKE             = 32.0;
    inline constexpr double FRICTION          = 5.0;
    inline constexpr double MAX_SPEED         = 8.0;
    inline constexpr double STICK_PULL        = 0.022;
    inline constexpr double SLIDE_GRAVITY     = 0.34;
    inline constexpr double SLIDE_MAX_FALL    = -3.6;
    inline constexpr double JUMP_PUSH         = 1.45;
    inline constexpr double TAN_CARRY         = 1.6;   // (kept for other uses, no longer used to crush speed)
    inline constexpr double LATCH_MAXINIT     = 3.0;
    inline constexpr double STICKY_TIME       = 0.12;
    inline constexpr double IDLE_DETACH_SEC   = 0.10;
    inline constexpr double POST_DETACH_PITCH_DAMP_SEC = 0.22;

    inline constexpr double R_NORMAL = 0.18;
    inline constexpr double R_RUN    = 0.14;
    inline constexpr double GRAVITY  = 16.0;

    // Allow higher speed right after a wall jump so we don't kill flow
    inline constexpr double DETACH_SPEED_CAP  = 14.0;  // was clamped to 9 before
    inline constexpr double AIR_SPEED_CAP     = 9.0;   // general air cap used elsewhere

    // Probing/entry
    Hit  probe(double x, double y);
    bool canStart(const Hit& h, double ix, double iy, const WallRunState& s);
    void start(WallRunState& s, const Hit& h, Player& player, float& yawVel, float& pitchVel);

    // State update (includes pZ to check for near-ground)
    ParkourState update(
        WallRunState& s, Player& player, double dt,
        double pZ, double& vZ, bool jumpPressed, double ix, double iy,
        int& jumpsLeft, double& momentum, double& outDetachPitchDamp);

    inline void tickTimers(WallRunState& s, double dt) {
        s.sticky      = std::max(0.0, s.sticky - dt);
        s.relatchLock = std::max(0.0, s.relatchLock - dt);
    }
} // namespace WallRun
