#pragma once
#include "types.h"

struct Player;
struct Enemy;

namespace HitTest {

    enum class Zone { None, Head, Body, ArmL, ArmR, LegL, LegR };

    struct Context {
        int    inW;
        int    inH;
        double fov;          // live FOV used for rendering this frame
        double ppuZ;         // pixels-per-world-Z (same as renderer)
        int    horizonBase;  // same horizon row used by renderer
        double pitchRad;     // actual camera pitch used this frame
        double wallScale;    // same wallScale you passed to renderer (includes ADS zoomMul)
        const Player* player;
    };

    Zone classify(const Context& rc, const Enemy& en,
                  double bulletWorldZ, double bx, double by);

} // namespace HitTest
