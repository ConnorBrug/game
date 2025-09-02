#pragma once
#include <algorithm>
#include "types.h"
#include "parkour_state.h"

struct CameraState {
    // Look
    double pitchRad = 0.0;
    float  yawVel = 0.f, pitchVel = 0.f;
    float  smoothFactor = 0.20f;
    bool   invertY = false;

    // Roll / pre-roll
    double rollDeg = 0.0, rollTarget = 0.0;
    double preRollTimer = 0.0, preRollLean = 0.0, preRollSign = 0.0;

    // Pitch damp after detach
    double pitchDampenTimer = 0.0;
};

namespace Camera {
    // Tunables matching your current behavior
    inline constexpr double PITCH_RATE_CAP = 0.040;
    inline constexpr double WALL_PITCH_DAMP = 0.35;

    inline constexpr double ROLL_MAX_DEG = 13.0;
    inline constexpr double ROLL_RESP    = 12.0;
    inline constexpr double PRE_ROLL_LEAD_SEC = 0.033;

    inline constexpr double VIEW_Z_MUL_AIR_GROUND = 0.35;
    inline constexpr double VIEW_Z_MUL_WALLRUN    = 0.12;
    inline constexpr int    VIEW_Z_SIGN           = +1;

    inline void beginFrame(CameraState& c, double dt) {
        c.pitchDampenTimer = std::max(0.0, c.pitchDampenTimer - dt);
        c.preRollTimer     = std::max(0.0, c.preRollTimer - dt);
    }

    void updateLook(CameraState& c, bool mouseCaptured, bool gameOver,
                    float rawDX, float rawDY, Player& player,
                    ParkourState pstate, double dt);

    inline double viewZ(double pZ, ParkourState ps) {
        const double mul = (ps == ParkourState::WallRun) ? VIEW_Z_MUL_WALLRUN : VIEW_Z_MUL_AIR_GROUND;
        return VIEW_Z_SIGN * mul * pZ;
    }

    int computeHorizon(int inH, double ppuZ, double viewZNow, double pitchRad);

    void updateRoll(CameraState& c, double dt, ParkourState pstate,
                    const Player& player, double wallNormalX, double wallNormalY,
                    double alongVel, double wallrunMaxSpeed);

    inline void dampPitchAfterDetach(CameraState& c, double seconds) { c.pitchDampenTimer = seconds; }

    inline void cuePreRoll(CameraState& c, double sign, double lean) {
        c.preRollSign = sign; c.preRollLean = lean; c.preRollTimer = PRE_ROLL_LEAD_SEC;
    }
}
