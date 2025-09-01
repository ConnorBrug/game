#pragma once
#include "types.h"
#include "framebuffer.h"

namespace HitTest {

    enum class Zone { None, Head, Body, ArmL, ArmR, LegL, LegR };

    struct Context {
        int     inW = 0, inH = 0;
        double  fov = 0.0;
        double  ppuZ = 0.0;
        int     horizonBase = 0;
        double  pitchRad = 0.0;
        const Player* player = nullptr;
    };

    // Classify which limb/body a bullet sample hits (analytic, screen-space)
    Zone classify(const Context& rc, const Enemy& en,
                  double bulletWorldZ, double bx, double by);

} // namespace HitTest
