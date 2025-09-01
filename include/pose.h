#pragma once
#include "types.h"

namespace Pose {

    enum class State { Normal, Crawl, Immobile };

    State  classify(const Enemy& en);

    float  visibleHeightScale(const Enemy& en);

    // Base drop so what’s left actually touches floor.
    int    groundDropSrcPx(const Enemy& en);

    // Extra dip on each “pull” beat (whole body).
    int    crawlPullExtraDropSrcPx(const Enemy& en);

    // NEW: how much to lean the upper body toward the camera (source px).
    int    forwardLeanSrcPx(const Enemy& en);

    // Crawl tilt wobble (sideways micro-wobble; forward lean is separate).
    double crawlTiltRadians(const Enemy& en);

    // Arm reach per side (source px). Positive = toward camera.
    int    armReachSrcPx(const Enemy& en, bool leftArm);

    double moveSpeedMul(const Enemy& en);
}
