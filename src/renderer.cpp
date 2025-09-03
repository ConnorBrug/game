// renderer.cpp — 360° pitch (wrap), no zoom-warp; sprites grounded.
// Floor grid: exact ray–plane projection in WORLD space (no treadmill, no yaw-locked sprite).
// pZ only shifts the final image (horizon/walls/sprites). Grid sampling ignores pZ
// so jumping does not change the pattern. Grid is anchored to world XY.

#include <algorithm>
#include <cmath>
#include <vector>

#include "../include/renderer.h"
#include "../include/framebuffer.h"

#ifdef _MSC_VER
  #include <corecrt_math_defines.h>
#endif

#include "../include/config.h"
#include "../include/util.h"
#include "../include/world.h"
#include "../include/sprites.h"
#include "../include/font.h"
#include "../include/ui.h"
#include "../include/types.h"
#include "../include/pose.h"

static constexpr double kHalfPi = 1.5707963267948966;
static constexpr double kTiny   = 1e-6;

// ==== grid tuning ====
static constexpr double GRID_PERIOD     = 0.5;   // world units between grid lines
static constexpr double GRID_HALF_THICK = 0.012; // world half thickness
// =====================

static constexpr double NEAR_CLAMP  = 0.035;
static constexpr double kEyeHeightWorld = 0.5; // camera height in WORLD units; pZ is visual shift only

Renderer::Renderer(int w, int h) { resize(w, h); }

void Renderer::resize(int w, int h) {
    W = std::max(1, w);
    H = std::max(1, h);
    zbuffer.assign(W, 0.0);

    int diag = int(std::ceil(std::sqrt(double(W) * W + double(H) * H)));
    int padding = 8;
    int overscanW = diag + 2 * padding;
    int overscanH = diag + 2 * padding;
    overscanBuffer.resize(size_t(overscanW) * size_t(overscanH));
}

void Renderer::putPixel(Framebuffer& fb, int x, int y, uint32_t color) const {
    if (x < 0 || y < 0) return;
    if (x >= fb.pitchPixels) return;
    if (activeFbHeight <= 0) return;
    if (y >= activeFbHeight) return;
    fb.pixels[size_t(y) * size_t(fb.pitchPixels) + size_t(x)] = color;
}

void Renderer::render(Framebuffer& fb,
                      const Player& player,
                      const std::vector<Enemy>& enemies,
                      const std::vector<Bullet>& bullets,
                      int score, int health, int maxHealth,
                      bool showMinimap,
                      double pZ, double pitchRad,
                      double /*fov*/, double useFov,
                      double wallScale, double pixelsPerUnitZ,
                      double hitFlash,
                      double rollDeg) {
    std::fill_n(fb.pixels, size_t(W) * size_t(H), 0xFF000000);

    // ---- 360° pitch: wrap pitch, project with safe pitch, add 180° roll when upside down
    auto wrapTwoPi = [](double a){ return std::remainder(a, 2.0 * M_PI); }; // (-pi, pi]
    const double pitchWrapped = wrapTwoPi(pitchRad);
    const bool   upsideDown   = std::cos(pitchWrapped) < 0.0;

    // Keep projection pitch within (-pi/2, pi/2] to avoid tan() singularity
    const double projPitch = upsideDown
        ? (pitchWrapped > 0.0 ? pitchWrapped - M_PI : pitchWrapped + M_PI)
        : pitchWrapped;

    const double effectiveRollDeg = rollDeg + (upsideDown ? 180.0 : 0.0);
    // ---- end wrap

    bool doRoll = std::fabs(effectiveRollDeg) > 0.75;
    if (doRoll) {
        int diag = int(std::ceil(std::sqrt(double(W) * W + double(H) * H)));
        int padding = 8;
        int overscanW = diag + 2 * padding;
        int overscanH = diag + 2 * padding;

        if (overscanBuffer.size() != size_t(overscanW) * size_t(overscanH)) {
            overscanBuffer.resize(size_t(overscanW) * size_t(overscanH));
        }

        Framebuffer ofs{ overscanBuffer.data(), overscanW };
        std::fill(overscanBuffer.begin(), overscanBuffer.end(), 0xFF000000);

        activeFbHeight = overscanH;
        renderWorldToBuffer(ofs, overscanH, W, H,
                            player, enemies, bullets,
                            pZ, projPitch, useFov, wallScale, pixelsPerUnitZ);

        const double a  = -effectiveRollDeg * M_PI / 180.0;
        const double ca = std::cos(a), sa = std::sin(a);
        const double sx = overscanW / 2.0, sy = overscanH / 2.0;
        const double dx = W / 2.0,       dy = H / 2.0;

        for (int y = 0; y < H; ++y) {
            double yr = double(y) - dy;
            double xr0 = -dx;
            double xs = xr0 * ca - yr * sa + sx;
            double ys = xr0 * sa + yr * ca + sy;

            for (int x = 0; x < W; ++x) {
                int xi = (int)xs;
                int yi = (int)ys;
                if ((unsigned)xi < (unsigned)ofs.pitchPixels &&
                    (unsigned)yi < (unsigned)overscanH) {
                    fb.pixels[size_t(y) * size_t(W) + size_t(x)] =
                        ofs.pixels[size_t(yi) * size_t(ofs.pitchPixels) + size_t(xi)];
                }
                xs += ca;
                ys += sa;
            }
        }
        activeFbHeight = H;
    } else {
        activeFbHeight = H;
        renderWorldToBuffer(fb, H, W, H,
                            player, enemies, bullets,
                            pZ, projPitch, useFov, wallScale, pixelsPerUnitZ);
    }

    drawUI(fb, player, enemies, score, health, maxHealth, !showMinimap);
    drawCrosshair(fb);

    if (hitFlash > 0.0) {
        float alpha = float(hitFlash / 0.18f);
        uint8_t add = (uint8_t)(alpha * 100);
        for (int y = 0; y < H; ++y) {
            for (int x = 0; x < W; ++x) {
                if (((x ^ y) & 3) == 0) {
                    uint32_t c = fb.pixels[size_t(y) * size_t(W) + size_t(x)];
                    uint8_t A = (c >> 24) & 255, R = (c >> 16) & 255, G = (c >> 8) & 255, B = c & 255;
                    R = (uint8_t)std::min(255, int(R) + int(add));
                    G = (uint8_t)std::min(255, int(G) + int(add) / 2);
                    fb.pixels[size_t(y) * size_t(W) + size_t(x)] = ARGB(A, R, G, B);
                }
            }
        }
    }
}

void Renderer::renderWorldToBuffer(Framebuffer& fb, int bufferHeight,
                                   int baseWidth, int baseHeight,
                                   const Player& player,
                                   const std::vector<Enemy>& enemies,
                                   const std::vector<Bullet>& bullets,
                                   double pZ, double pitchRad,
                                   double useFov, double wallScale,
                                   double pixelsPerUnitZ) {
    const int fbWidth = fb.pitchPixels;

    const int offsetX = (fbWidth      - baseWidth ) / 2;
    const int offsetY = (bufferHeight - baseHeight) / 2;

    // -------- Camera focal length from FOV (square pixels) --------
    const double fx = (double(baseWidth) * 0.5) / std::tan(useFov * 0.5);
    const double fy = fx;

    const double sP = std::sin(pitchRad);
    const double cP = std::cos(pitchRad);
    const double eps = 1e-4;
    const double tanSafe = sP / std::max(std::abs(cP), eps); // avoids tan blowups

    // -------- Sky / Horizon (for walls + clipping) --------
    const int pitchPx = int(std::lround(fy * tanSafe));
    const int horizonBase = baseHeight/2 +
                            int(std::lround(pZ * pixelsPerUnitZ)) + // pZ shifts the whole image
                            pitchPx;
    const int horizon = offsetY + horizonBase;

    const uint32_t SKY_TOP   = ARGB(255, 178, 196, 214);
    const uint32_t SKY_HORIZ = ARGB(255, 204, 210, 198);

    const int skyEnd = std::min(std::max(horizon, 0), bufferHeight);
    for (int y = 0; y < skyEnd; ++y) {
        float t = (skyEnd>0) ? float(y) / float(std::max(1, skyEnd)) : 0.f;
        uint32_t c = lerpColor(SKY_TOP, SKY_HORIZ, t);
        uint32_t* row = fb.pixels + size_t(y) * size_t(fbWidth);
        std::fill(row, row + fbWidth, c);
    }

    // -------- Common ray basis for walls --------
    const double dirX = std::cos(player.dir);
    const double dirY = std::sin(player.dir);
    const double planeX = std::cos(player.dir + kHalfPi) * std::tan(useFov / 2.0);
    const double planeY = std::sin(player.dir + kHalfPi) * std::tan(useFov / 2.0);

    std::vector<int> floorClip(fbWidth, horizon - 1);

    // -------- Walls --------
    const double sunX = std::cos(0.8), sunY = std::sin(0.8);
    std::fill(zbuffer.begin(), zbuffer.end(), 1e30);

    for (int x = 0; x < fbWidth; ++x) {
        const double u = (double(x) - (offsetX + baseWidth * 0.5)) / (double(baseWidth) * 0.5);
        const double cameraX = u;

        double rayDirX = dirX + planeX * cameraX;
        double rayDirY = dirY + planeY * cameraX;

        int mapX = int(player.x), mapY = int(player.y);
        double deltaX = (std::abs(rayDirX) < kTiny) ? 1e30 : std::abs(1.0 / rayDirX);
        double deltaY = (std::abs(rayDirY) < kTiny) ? 1e30 : std::abs(1.0 / rayDirY);
        double sideX, sideY;
        int stepX, stepY, side = 0;

        if (rayDirX < 0) { stepX = -1; sideX = (player.x - mapX) * deltaX; }
        else             { stepX =  1; sideX = (mapX + 1.0 - player.x) * deltaX; }
        if (rayDirY < 0) { stepY = -1; sideY = (player.y - mapY) * deltaY; }
        else             { stepY =  1; sideY = (mapY + 1.0 - player.y) * deltaY; }

        int tile = 0; bool hit = false;
        while (!hit) {
            if (sideX < sideY) { sideX += deltaX; mapX += stepX; side = 0; }
            else               { sideY += deltaY; mapY += stepY; side = 1; }
            if (mapX < 0 || mapY < 0 || mapX >= MAP_W || mapY >= MAP_H) { hit = true; tile = 1; break; }
            if (worldMap[mapY][mapX] > 0) { hit = true; tile = worldMap[mapY][mapX]; }
        }

        double perpDist = (side == 0 ? sideX - deltaX : sideY - deltaY);
        if (perpDist < NEAR_CLAMP) perpDist = NEAR_CLAMP;

        double wallX = 0.0;
        if (side == 0) wallX = player.y + perpDist * rayDirY;
        else           wallX = player.x + perpDist * rayDirX;
        wallX -= std::floor(wallX);

        int lineH = int((baseHeight * wallScale) / perpDist);
        int drawTop = offsetY + (horizonBase - lineH/2);
        int drawBot = offsetY + (horizonBase + lineH/2);
        drawTop = std::max(0, drawTop);
        drawBot = std::min(bufferHeight - 1, drawBot);

        uint32_t base;
        switch (tile) {
            case 1: base = ARGB(255, 200, 108, 108); break;
            case 2: base = ARGB(255, 100, 170, 120); break;
            case 3: base = ARGB(255, 100, 135, 200); break;
            case 4: base = ARGB(255, 210, 190, 100); break;
            default: base = ARGB(255, 160, 160, 160); break;
        }

        double nX = (side==0) ? -double(stepX) : 0.0;
        double nY = (side==1) ? -double(stepY) : 0.0;
        double lambert = std::max(0.0, nX*sunX + nY*sunY);
        float faceBoost = float(0.08 + 0.16 * lambert);
        float distBoost = float(0.08 / (1.0 + 0.25 * perpDist));
        float intensity = float(0.90 + faceBoost + distBoost);
        if (intensity > 1.20f) intensity = 1.20f;

        const double rim = 0.08;
        float rimBoost = 0.0f;
        if (wallX < rim || wallX > (1.0 - rim)) {
            bool external = false;
            if (side == 0) {
                if (wallX < rim)   external = (mapY-1 < 0) || (worldMap[mapY-1][mapX] == 0);
                else               external = (mapY+1 >= MAP_H) || (worldMap[mapY+1][mapX] == 0);
            } else {
                if (wallX < rim)   external = (mapX-1 < 0) || (worldMap[mapY][mapX-1] == 0);
                else               external = (mapX+1 >= MAP_W) || (worldMap[mapY][mapX+1] == 0);
            }
            if (external) {
                double d = (wallX < rim) ? (rim - wallX) : (wallX - (1.0 - rim));
                float t = float(std::max(0.0, std::min(1.0, d / rim)));
                rimBoost = 0.12f * (1.0f - t);
            }
        }

        uint32_t shade = Modulate(base, intensity + rimBoost);
        uint32_t topHL = Modulate(shade, 1.06f);
        uint32_t botHL = Modulate(shade, 1.04f);

        for (int y = drawTop; y <= drawBot; ++y) {
            if (y == drawTop)      { putPixel(fb, x, y, topHL); }
            else if (y == drawBot) { putPixel(fb, x, y, botHL); }
            else                   { putPixel(fb, x, y, shade); }
        }

        if (drawBot > floorClip[x]) floorClip[x] = drawBot;

        if (x >= offsetX && x < offsetX + baseWidth) {
            int lx = x - offsetX;
            if (lx >= 0 && lx < W) zbuffer[lx] = perpDist;
        }
    }

    // -------- Floor (after walls) --------
    // Exact ray→ground-plane (Z=0) per-row mapping in WORLD space.
    // IMPORTANT:
    //   • v DOES NOT include pZ → jumping doesn't change sampled world points.
    //   • The horizontal step across the row must use the camera RIGHT vector,
    //     not the FORWARD vector (this fixes the rotation/treadmill artifacts).
    const uint32_t FLOOR_BASE = ARGB(255, 176, 176, 176);
    const uint32_t LINE_FAR   = ARGB(255, 132, 132, 132);
    const uint32_t LINE_NEAR  = ARGB(255, 108, 108, 108);
    const uint32_t DOT_NEAR   = ARGB(255,  96,  96,  96);

    const double cy = offsetY + baseHeight * 0.5;
    const double cx = offsetX + baseWidth  * 0.5;

    const double yaw   = player.dir;
    const double cYaw  = std::cos(yaw);
    const double sYaw  = std::sin(yaw);
    // World camera basis:
    const double rightX = -sYaw, rightY =  cYaw;
    const double fwdX   =  cYaw, fwdY   =  sYaw;

    const double invFx = 1.0 / fx;

    const int floorStart = std::max(0, std::min(horizon, bufferHeight));
    for (int y = floorStart; y < bufferHeight; ++y) {
        // Pixel center — do NOT include pZ in v
        const double v = ((double(y) + 0.5) - cy) / fy;

        // Pitch rotate direction (row-constant terms)
        const double yPrime =  cP + v * sP;   // forward-like component
        const double zPrime =  sP - v * cP;

        if (zPrime >= -1e-8) continue; // above horizon → no intersection

        // Intersection scale parameter; constant world eye height
        const double t = kEyeHeightWorld / (-zPrime);

        // Camera u at left edge & step per pixel
        const double du  = invFx;
        const double u0  = ((0.5 - cx) * invFx);

        // WORLD position at x=0 using correct basis: world = player + t*(u*RIGHT + y'*FWD)
        double wx = player.x + t * ( u0 * rightX + yPrime * fwdX );
        double wy = player.y + t * ( u0 * rightY + yPrime * fwdY );

        // Step when moving one pixel to the right on screen → along RIGHT vector
        const double stepWx = t * du * rightX;
        const double stepWy = t * du * rightY;

        // Row shading
        float contrast = std::min(1.f, float(y - horizon) / 64.f);
        const uint32_t LINE_COL = lerpColor(LINE_FAR, LINE_NEAR, contrast);
        const uint32_t DOT_COL  = lerpColor(LINE_FAR, DOT_NEAR, contrast);

        for (int x = 0; x < fbWidth; ++x, wx += stepWx, wy += stepWy) {
            if (y <= floorClip[x]) continue;

            // WORLD-LOCKED modulo positions
            double fxg = std::fmod(wx, GRID_PERIOD); if (fxg < 0) fxg += GRID_PERIOD;
            double fyg = std::fmod(wy, GRID_PERIOD); if (fyg < 0) fyg += GRID_PERIOD;

            // distance to nearest grid line in X/Y (WORLD units)
            double dxg = std::min(fxg, GRID_PERIOD - fxg);
            double dyg = std::min(fyg, GRID_PERIOD - fyg);

            const bool onX = dxg < GRID_HALF_THICK;
            const bool onY = dyg < GRID_HALF_THICK;
            const uint32_t c = (onX && onY) ? DOT_COL : (onX || onY) ? LINE_COL : FLOOR_BASE;

            putPixel(fb, x, y, c);
        }
    }

    // -------- Sprites --------
    struct SpriteProj {
        int    kind;     // 0 = enemy, 1 = bullet
        int    idx;
        double dist;
        int    screenX;
        int    width, height;
        double depth;
        double zWorld;
    };

    std::vector<SpriteProj> sprs; sprs.reserve(enemies.size() + bullets.size());

    auto project = [&](double ex, double ey, double zWorld, int wBase, int hBase, int kind, int idx, float scale) {
        double dx = ex - player.x, dy = ey - player.y;

        const double planeXb = std::cos(player.dir + kHalfPi) * std::tan(useFov / 2.0);
        const double planeYb = std::sin(player.dir + kHalfPi) * std::tan(useFov / 2.0);

        const double invDet     = 1.0 / (planeXb * std::sin(player.dir) - std::cos(player.dir) * planeYb);
        const double transformX = invDet * (std::sin(player.dir) * dx - std::cos(player.dir) * dy);
        const double transformY = invDet * (-planeYb * dx + planeXb * dy);
        if (transformY <= 0.0) return;
        if (transformY > SPRITE_FAR_CLIP) return;

        const int screenX = int((baseWidth / 2.0) * (1.0 + transformX / transformY)) + offsetX;

        double sizeDepth = std::max(0.25, transformY);
        const int h = int(std::abs((baseHeight * wallScale) / sizeDepth) * scale * SPRITE_SCALE);
        const int w = int(h * wBase / double(hBase));
        if (w <= 0 || h <= 0) return;

        sprs.push_back({kind, idx, dx*dx + dy*dy, screenX, w, h, transformY, zWorld});
    };

    for (int i = 0; i < (int)enemies.size(); ++i)
        if (enemies[i].alive)
            project(enemies[i].x, enemies[i].y, 0.0, SM_W, SM_H, 0, i, 1.0f);

    for (int i = 0; i < (int)bullets.size(); ++i)
        project(bullets[i].x, bullets[i].y, bullets[i].z, BL_W, BL_H, 1, i, float(2.0 * BULLET_RADIUS));

    std::sort(sprs.begin(), sprs.end(),
              [](const SpriteProj& a, const SpriteProj& b){ return a.dist > b.dist; });

    static int STICKMAN_BASELINE = -1;
    if (STICKMAN_BASELINE < 0) {
        STICKMAN_BASELINE = 0;
        for (int y = SM_H - 1; y >= 0; --y) {
            for (int x = 0; x < SM_W; ++x) {
                if ((g_stickman[y * SM_W + x] >> 24) != 0) { STICKMAN_BASELINE = y; goto have_baseline; }
            }
        }
        have_baseline: ;
    }

    for (auto& s : sprs) {
        const int zPixels = int(std::lround(s.zWorld * pixelsPerUnitZ));

        int topY, bottomY;
        if (s.kind == 0) {
            // Grounded enemy: feet land on ground line derived from same horizon
            const double depth = std::max(0.2, s.depth);
            const int    colH  = int(std::abs((baseHeight * wallScale) / depth));
            const int    groundY = offsetY + horizonBase + colH / 2;

            const double texFootToScreen = ((STICKMAN_BASELINE + 0.5) * s.height) / double(SM_H);
            topY    = int(std::floor(groundY - texFootToScreen));
            bottomY = topY + s.height;
        } else {
            // Bullets: centered around pitched horizon, with world Z offset
            const int centerY = offsetY + horizonBase - zPixels;
            topY    = centerY - s.height / 2;
            bottomY = topY + s.height;
        }

        int dsX = std::max(0, -s.width / 2 + s.screenX);
        int deX = std::min(fb.pitchPixels - 1, s.width / 2 + s.screenX);
        if (dsX > deX) continue;

        if (s.kind == 0) {
            const Enemy& en = enemies[s.idx];

            const float  scaleVis = Pose::visibleHeightScale(en);
            const int    hVis     = std::max(1, int(std::round(s.height * scaleVis)));
            const int    wVis     = s.width;
            int          topYVis  = bottomY - hVis;

            const int sinkSrc = Pose::groundDropSrcPx(en) + Pose::crawlPullExtraDropSrcPx(en);
            const int sinkPx  = int(std::round(double(sinkSrc) * (double(hVis) / double(SM_H))));
            topYVis += sinkPx; // ensure floor contact

            const int bottomYVis = topYVis + hVis;
            const int deY        = std::min(bufferHeight - 1, bottomYVis);

            const int leanSrc   = Pose::forwardLeanSrcPx(en);
            const int leanPxMax = int(std::round(double(leanSrc) * (double(hVis) / double(SM_H))));

            const float flashT = (float)std::min(1.0, en.hitFlash / ENEMY_HIT_FLASH_SEC);

            for (int stripe = dsX; stripe <= deX; ++stripe) {
                int lx = stripe - offsetX;
                if (lx < 0 || lx >= W || !(s.depth < zbuffer[lx])) continue;

                int texX = int((stripe - (-wVis / 2 + s.screenX)) * SM_W / double(wVis));
                if ((unsigned)texX >= (unsigned)SM_W) continue;

                const int topYStripe = topYVis;

                const int startY = std::max(0, topYStripe);
                for (int y = startY; y <= deY; ++y) {
                    int d    = y - topYStripe;
                    int texY = int((int64_t)d * SM_H / hVis);
                    if ((unsigned)texY >= (unsigned)SM_H) continue;

                    uint32_t px = g_stickman[texY * SM_W + texX];
                    if ((px >> 24) == 0) continue;

                    // limb masking
                    uint8_t m = g_stickman_mask[texY * SM_W + texX];
                    bool masked =
                        (!en.armL && m == SM_ArmL) ||
                        (!en.armR && m == SM_ArmR) ||
                        (!en.legL && m == SM_LegL) ||
                        (!en.legR && m == SM_LegR);
                    if (masked) continue;

                    // forward lean toward camera (more at head)
                    int yLean = y + (leanPxMax * (SM_H - 1 - texY)) / (SM_H - 1);

                    // arm reach shift (crawl)
                    int yDst = yLean;
                    if ((m == SM_ArmL || m == SM_ArmR) && Pose::classify(en) == Pose::State::Crawl) {
                        const bool isLeft  = (m == SM_ArmL);
                        const int  reachSrc = Pose::armReachSrcPx(en, isLeft);
                        const int  reachPx  = int(std::round(double(reachSrc) * (double(hVis) / double(SM_H))));
                        yDst += reachPx;
                    }

                    if (flashT > 0.f) {
                        uint8_t A = (px >> 24) & 255, R = (px >> 16) & 255, G = (px >> 8) & 255, B = px & 255;
                        int r = int(R + (255 - R) * flashT);
                        int g = int(G + ( 50 - G) * flashT);
                        int b = int(B + ( 50 - B) * flashT);
                        px = (uint32_t(A) << 24) | (uint32_t(r) << 16) | (uint32_t(g) << 8) | uint32_t(b);
                    }

                    if ((unsigned)yDst < (unsigned)bufferHeight) {
                        putPixel(fb, stripe, yDst, px);
                    }
                }
            }
        } else {
            // bullets
            int dsY   = std::max(0, topY);
            int deY   = std::min(bufferHeight - 1, bottomY);
            if (dsY <= deY) {
                for (int stripe = dsX; stripe <= deX; ++stripe) {
                    int lx = stripe - offsetX;
                    if (lx >= 0 && lx < W && s.depth < zbuffer[lx]) {
                        int texX = int((stripe - (-s.width / 2 + s.screenX)) * BL_W / double(s.width));
                        for (int y = dsY; y <= deY; ++y) {
                            int d = y - topY;
                            int texY = int((int64_t)d * BL_H / s.height); // correct axis
                            if ((unsigned)texY >= (unsigned)BL_H || (unsigned)texX >= (unsigned)BL_W) continue;
                            uint32_t px = g_bullet[texY * BL_W + texX];
                            if ((px >> 24) == 0) continue;
                            putPixel(fb, stripe, y, Modulate(px, 1.04f));
                        }
                    }
                }
            }
        }
    }
}

void Renderer::drawCrosshair(Framebuffer& fb) const {
    const int cx = W / 2;
    const int cy = H / 2;

    const double sx = double(W) / double(SCREEN_W);
    const double sy = double(H) / double(SCREEN_H);
    const double s  = std::min(sx, sy);
    const int    len   = std::max(6, int(8 * s));
    const int    thick = (s >= 1.8 ? 2 : 1);

    for (int i = -len; i <= len; ++i) {
        for (int t = 0; t < thick; ++t) {
            putPixel(fb, cx + i, cy + t, ARGB(255, 235, 235, 235));
            putPixel(fb, cx + t, cy + i, ARGB(255, 235, 235, 235));
        }
    }
}

void Renderer::drawUI(Framebuffer& fb,
                      const Player& player,
                      const std::vector<Enemy>& enemies,
                      int score, int health, int maxHealth,
                      bool /*hideMinimap*/) const {
    const double sx = double(W) / double(SCREEN_W);
    const double sy = double(H) / double(SCREEN_H);
    const double s  = std::min(sx, sy);
    const int    pad = std::max(8, int(12 * s));
    const uint32_t white = ARGB(255, 230, 230, 230);

    int target = int(std::round(std::min(W, H) * 0.26));
    int maxAllowed = std::max(32, std::min(W, H) - 2 * pad);
    target = std::min(target, maxAllowed);

    const int tile = std::max(2, target / std::max(MAP_W, MAP_H));
    const int mmW  = MAP_W * tile;
    const int mmH  = MAP_H * tile;

    const int mmX = std::max(pad, W - pad - mmW);
    const int mmY = pad;

    for (int y = 0; y < mmH; ++y)
        for (int x = 0; x < mmW; ++x)
            putPixel(fb, mmX + x, mmY + y, ARGB(200, 8, 8, 8));

    for (int my = 0; my < MAP_H; ++my)
        for (int mx = 0; mx < MAP_W; ++mx) {
            const bool wall = worldMap[my][mx] != 0;
            const uint32_t c = wall ? ARGB(255, 190, 190, 190) : ARGB(255, 32, 32, 32);
            const int px = mmX + mx * tile, py = mmY + my * tile;
            for (int yy = 0; yy < tile; ++yy)
                for (int xx = 0; xx < tile; ++xx)
                    putPixel(fb, px + xx, py + yy, c);
        }

    for (const auto& en : enemies) if (en.alive) {
        const int ex = mmX + int(en.x * tile);
        const int ey = mmY + int(en.y * tile);
        for (int dy = -1; dy <= 1; ++dy)
            for (int dx = -1; dx <= 1; ++dx)
                putPixel(fb, ex + dx, ey + dy, ARGB(255, 60, 210, 110));
    }

    {
        const int px = mmX + int(player.x * tile);
        const int py = mmY + int(player.y * tile);
        for (int dy = -1; dy <= 1; ++dy)
            for (int dx = -1; dx <= 1; ++dx)
                putPixel(fb, px + dx, py + dy, ARGB(255, 230, 70, 70));
    }

    const int sTxt = std::max(2, int(3 * s));
    const int cw   = 3 * sTxt + sTxt;
    const int labelWidth = 6 * cw;

    drawText  (fb, pad, pad, "SCORE:", sTxt, white);
    drawNumber(fb, pad + labelWidth + int(8 * s), pad, score, sTxt, white);

    // Health bar
    {
        const double s   = std::min(double(W)/double(SCREEN_W), double(H)/double(SCREEN_H));
        const int    pad = std::max(8, int(12 * s));
        const uint32_t white = ARGB(255,230,230,230);

        const int hbW  = std::max(120, int(180 * s));
        const int hbH  = std::max(12,  int(std::round(16 * s)));
        const int hbX  = pad;
        int       hbY  = H - pad - hbH;
        if (hbY < pad) hbY = pad;

        auto clampf = [](float v){ return v < 0.0f ? 0.0f : (v > 1.0f ? 1.0f : v); };
        auto lerp   = [](int a,int b,float t){ return a + int(std::round((b-a)*t)); };

        float ratio = (maxHealth>0)? clampf(float(health)/float(maxHealth)) : 0.f;

        uint32_t fg;
        if (ratio >= 0.5f) {
            float t = (ratio - 0.5f) / 0.5f;
            fg = ARGB(255, lerp(220,  70, t), lerp(170, 220, t), 70);
        } else {
            float t = ratio / 0.5f;
            fg = ARGB(255, 220, lerp(70, 170, t), 70);
        }
        uint32_t bg     = ARGB(255, 22, 22, 22);
        uint32_t border = white;

        for (int y = hbY; y < hbY + hbH; ++y)
            for (int x = hbX; x < hbX + hbW; ++x)
                putPixel(fb, x, y, bg);

        int clampedH = std::max(0, std::min(health, maxHealth));
        int fillW = (maxHealth > 0) ? int((int64_t)clampedH * hbW / maxHealth) : 0;
        for (int y = hbY+1; y < hbY + hbH - 1; ++y)
            for (int x = hbX+1; x < hbX + 1 + fillW; ++x)
                putPixel(fb, x, y, fg);

        for (int x = hbX; x < hbX + hbW; ++x) {
            putPixel(fb, x, hbY, border);
            putPixel(fb, x, hbY + hbH - 1, border);
        }
        for (int y = hbY; y < hbY + hbH; ++y) {
            putPixel(fb, hbX, y, border);
            putPixel(fb, hbX + hbW - 1, y, border);
        }
    }
}
