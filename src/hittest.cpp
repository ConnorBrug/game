#define _USE_MATH_DEFINES
#include <cmath>
#include <algorithm>
#include "../include/hittest.h"
#include "../include/pose.h"
#include "../include/config.h"
#include "../include/sprites.h"
#include "../include/world.h"

namespace {
constexpr double kHalfPi = 1.5707963267948966;

struct StickGeom {
    int cx, cy, rHead;
    int bodyTopY, bodyBotY;
    int armLx0, armLy0, armLx1, armLy1;
    int armRx0, armRy0, armRx1, armRy1;
    int legLx0, legLy0, legLx1, legLy1;
    int legRx0, legRy0, legRx1, legRy1;
};
inline StickGeom makeStickGeom() {
    StickGeom g{};
    g.cx = SM_W / 2; g.cy = 14; g.rHead = 8;
    g.bodyTopY = g.cy + g.rHead;
    g.bodyBotY = g.cy + g.rHead + 22;
    int armY0 = g.cy + g.rHead + 8;
    g.armLx0 = g.cx - 1; g.armLy0 = armY0;     g.armLx1 = g.cx - 1 - 15; g.armLy1 = armY0 + 7;
    g.armRx0 = g.cx + 1; g.armRy0 = armY0;     g.armRx1 = g.cx + 1 + 15; g.armRy1 = armY0 + 7;
    int legY0 = g.cy + g.rHead + 22;
    g.legLx0 = g.cx - 1; g.legLy0 = legY0;     g.legLx1 = g.cx - 1 - 9;  g.legLy1 = legY0 + 18;
    g.legRx0 = g.cx + 1; g.legRy0 = legY0;     g.legRx1 = g.cx + 1 + 9;  g.legRy1 = legY0 + 18;
    return g;
}

inline double dist2ToSeg(double px, double py,
                         double ax, double ay, double bx, double by) {
    double vx = bx - ax, vy = by - ay;
    double wx = px - ax, wy = py - ay;
    double vv = vx * vx + vy * vy;
    double t  = (vv > 0.0) ? std::clamp((vx * wx + vy * wy) / vv, 0.0, 1.0) : 0.0;
    double dx = (ax + vx * t) - px;
    double dy = (ay + vy * t) - py;
    return dx * dx + dy * dy;
}

inline bool screenFor(const HitTest::Context& rc, double wx, double wy,
                      int& outX, double& outDepth) {
    const double dirX = std::cos(rc.player->dir), dirY = std::sin(rc.player->dir);
    const double planeX = std::cos(rc.player->dir + kHalfPi) * std::tan(rc.fov / 2.0);
    const double planeY = std::sin(rc.player->dir + kHalfPi) * std::tan(rc.fov / 2.0);

    const double invDet = 1.0 / (planeX * dirY - dirX * planeY);
    const double dx = wx - rc.player->x, dy = wy - rc.player->y;
    const double transformX = invDet * (dirY * dx - dirX * dy);
    const double transformY = invDet * (-planeY * dx + planeX * dy);
    if (transformY <= 0.0 || transformY > SPRITE_FAR_CLIP) return false;

    outX     = int((rc.inW / 2.0) * (1.0 + transformX / transformY));
    outDepth = transformY;
    return true;
}

struct Billboard {
    int leftX = 0, topY = 0, w = 0, h = 0, groundY = 0, spriteBot = 0, enX = 0;
    double enDepth = 0.0;
    double tiltRad = 0.0;
};

inline bool buildBillboard(const HitTest::Context& rc, const Enemy& en, Billboard& bb) {
    if (!screenFor(rc, en.x, en.y, bb.enX, bb.enDepth)) return false;

    const double sizeDepth = std::max(0.25, bb.enDepth);

    // IMPORTANT: use the SAME wallScale and pitch that the renderer used
    const int sHeight = int(std::abs((rc.inH * rc.wallScale * std::cos(rc.pitchRad)) / sizeDepth) * SPRITE_SCALE);
    if (sHeight <= 0) return false;
    const int sWidth = int(sHeight * SM_W / double(SM_H));

    const int colH = int(std::abs((rc.inH * rc.wallScale * std::cos(rc.pitchRad)) / std::max(0.2, bb.enDepth)));
    const int groundY = rc.horizonBase + colH / 2;
    bb.groundY = groundY;

    // Stickman baseline detection (same as renderer)
    static int STICKMAN_BASELINE = -1;
    if (STICKMAN_BASELINE < 0) {
        STICKMAN_BASELINE = 0;
        for (int y = SM_H - 1; y >= 0; --y) {
            for (int x = 0; x < SM_W; ++x) {
                if ((g_stickman[y * SM_W + x] >> 24) != 0) { STICKMAN_BASELINE = y; goto have_base; }
            }
        }
        have_base: ;
    }

    const double texFootToScreen = ((STICKMAN_BASELINE + 0.5) * sHeight) / double(SM_H);
    const int spriteTop  = int(std::floor(groundY - texFootToScreen));
    const int spriteBot  = spriteTop + sHeight;
    bb.spriteBot = spriteBot;

    // Apply pose scale & sink so hit-test matches what we draw
    const float scale = Pose::visibleHeightScale(en);
    bb.h = std::max(1, int(std::round(sHeight * scale)));
    bb.w = sWidth;
    int top = spriteBot - bb.h;

    const int sinkSrc = Pose::groundDropSrcPx(en)
                      + Pose::crawlPullExtraDropSrcPx(en)
                      + Pose::forwardLeanSrcPx(en);
    const int sinkPx  = int(std::round(double(sinkSrc) * (double(bb.h) / double(SM_H))));
    top += sinkPx;

    bb.topY  = top;
    bb.leftX = bb.enX - bb.w / 2;
    bb.tiltRad = 0.0; // (no shear here; renderer also draws enemies upright)
    return true;
}

inline void srcToScreen(const Billboard& bb, double sx, double sy,
                        double& outX, double& outY) {
    const double sxScale = double(bb.w) / double(SM_W);
    const double syScale = double(bb.h) / double(SM_H);
    outX = bb.leftX + sx * sxScale;
    outY = bb.topY  + sy * syScale;

    if (std::fabs(bb.tiltRad) > 1e-6) {
        const double centerX = (SM_W * 0.5);
        const double yShiftSrc = (sx - centerX) * std::tan(bb.tiltRad) * (double(bb.h) / double(SM_W));
        outY += yShiftSrc;
    }
}

} // anonymous

namespace HitTest {

Zone classify(const Context& rc, const Enemy& en,
              double bulletWorldZ, double bx, double by) {
    Billboard bb{}; if (!buildBillboard(rc, en, bb)) return Zone::None;

    int bpx = 0; double bDepth = 0.0;
    if (!screenFor(rc, bx, by, bpx, bDepth)) return Zone::None;

    // Convert bullet world-Z to screen Y with the SAME horizon/ppuZ used by renderer
    const int bulletY = rc.horizonBase - int(std::lround(bulletWorldZ * rc.ppuZ));

    if (bpx < bb.leftX || bpx >= bb.leftX + bb.w) return Zone::None;
    if (bulletY < bb.topY || bulletY >= bb.topY + bb.h) return Zone::None;

    // Thickness in screen px
    const double sX = double(bb.w) / double(SM_W);
    const double sY = double(bb.h) / double(SM_H);
    const double s  = 0.5 * (sX + sY);
    const double HEAD_THICK = 0.8 * s;
    const double ARM_THICK  = 2.0 * s;
    const double LEG_THICK  = 2.0 * s;
    const double BODY_THICK = 1.8 * s;

    const StickGeom G = makeStickGeom();

    // HEAD
    double hcX, hcY; srcToScreen(bb, G.cx, G.cy, hcX, hcY);
    const double headR = G.rHead * s;
    const double dhx = double(bpx) - hcX, dhy = double(bulletY) - hcY;
    if (dhx * dhx + dhy * dhy <= (headR + HEAD_THICK) * (headR + HEAD_THICK)) return Zone::Head;

    auto segHit = [&](int sx0, int sy0, int sx1, int sy1, double thick) -> bool {
        double ax, ay, bx2, by2; srcToScreen(bb, sx0, sy0, ax, ay); srcToScreen(bb, sx1, sy1, bx2, by2);
        return dist2ToSeg(bpx, bulletY, ax, ay, bx2, by2) <= thick * thick;
    };

    if (en.armL && segHit(G.armLx0, G.armLy0, G.armLx1, G.armLy1, ARM_THICK)) return Zone::ArmL;
    if (en.armR && segHit(G.armRx0, G.armRy0, G.armRx1, G.armRy1, ARM_THICK)) return Zone::ArmR;
    if (en.legL && segHit(G.legLx0, G.legLy0, G.legLx1, G.legLy1, LEG_THICK)) return Zone::LegL;
    if (en.legR && segHit(G.legRx0, G.legRy0, G.legRx1, G.legRy1, LEG_THICK)) return Zone::LegR;

    // spine/body capsule
    double b0x, b0y, b1x, b1y; srcToScreen(bb, G.cx, G.bodyTopY, b0x, b0y); srcToScreen(bb, G.cx, G.bodyBotY, b1x, b1y);
    if (dist2ToSeg(bpx, bulletY, b0x, b0y, b1x, b1y) <= BODY_THICK * BODY_THICK) return Zone::Body;

    return Zone::None;
}

} // namespace HitTest
