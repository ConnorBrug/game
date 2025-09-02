#include "../include/camera.h"
#include <cmath>

void Camera::updateLook(CameraState& c, bool mouseCaptured, bool gameOver,
                        float rawDX, float rawDY, Player& player,
                        ParkourState pstate, double dt)
{
    if (!mouseCaptured || gameOver) return;

    const float inv = std::clamp(1.0f - c.smoothFactor, 0.0f, 1.0f);
    c.yawVel   = c.yawVel   * c.smoothFactor + rawDX * inv;
    c.pitchVel = c.pitchVel * c.smoothFactor + rawDY * inv;

    player.dir += c.yawVel * 0.0025f;

    double dyAngle = (c.invertY ? 1.0 : -1.0) * (double)c.pitchVel * 0.0018;
    const double pitchDamp = (pstate == ParkourState::WallRun || c.pitchDampenTimer > 0.0) ? WALL_PITCH_DAMP : 1.0;
    dyAngle *= pitchDamp;

    if (dyAngle >  PITCH_RATE_CAP) dyAngle =  PITCH_RATE_CAP;
    if (dyAngle < -PITCH_RATE_CAP) dyAngle = -PITCH_RATE_CAP;

    c.pitchRad += dyAngle;
    if (c.pitchRad >  1.35) c.pitchRad =  1.35;
    if (c.pitchRad < -1.35) c.pitchRad = -1.35;
}

int Camera::computeHorizon(int inH, double ppuZ, double viewZNow, double pitchRad) {
    // Same formula you used
    const double term = viewZNow * ppuZ + (inH / 2.0) * std::tan(pitchRad);
    return inH / 2 + (int)std::lround(term);
}

void Camera::updateRoll(CameraState& c, double dt, ParkourState pstate,
                        const Player& player, double wallNormalX, double wallNormalY,
                        double alongVel, double wallrunMaxSpeed)
{
    double target = 0.0;

    if (pstate == ParkourState::WallRun) {
        const double sx = -std::sin(player.dir), sy = std::cos(player.dir);
        const double side = sx * wallNormalX + sy * wallNormalY;
        const double speedFactor = std::clamp(std::fabs(alongVel) / wallrunMaxSpeed, 0.0, 1.0);
        const double lean = ROLL_MAX_DEG + 8.0 * speedFactor;
        target = (side > 0.0 ? -lean : +lean);
    } else if (c.preRollTimer > 0.0) {
        target = c.preRollSign * c.preRollLean;
    } else {
        target = 0.0;
    }

    c.rollTarget = target;
    const double alpha = std::clamp(dt * ROLL_RESP, 0.0, 1.0);
    c.rollDeg += (c.rollTarget - c.rollDeg) * alpha;
}
