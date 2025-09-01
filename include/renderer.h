#pragma once
#include <vector>
#include <cstdint>
#include "types.h"
#include "framebuffer.h"

class Renderer {
public:
    Renderer(int w, int h);
    void resize(int w, int h);

    // Supports overscan + CPU rotation for camera roll
    void render(Framebuffer& fb,
                const Player& player,
                const std::vector<Enemy>& enemies,
                const std::vector<Bullet>& bullets,
                int score, int health, int maxHealth,
                bool showMinimap,
                double pZ, double pitchRad,
                double fov, double useFov,
                double wallScale, double pixelsPerUnitZ,
                double hitFlash,
                double rollDeg);

    int  W=1, H=1;

private:
    void putPixel(Framebuffer& fb, int x, int y, uint32_t color) const;

    void renderWorldToBuffer(Framebuffer& fb, int bufferHeight,
                             int baseWidth, int baseHeight,
                             const Player& player,
                             const std::vector<Enemy>& enemies,
                             const std::vector<Bullet>& bullets,
                             double pZ, double pitchRad,
                             double useFov, double wallScale,
                             double pixelsPerUnitZ);

    void drawCrosshair(Framebuffer& fb) const;
    void drawUI(Framebuffer& fb,
                const Player& player,
                const std::vector<Enemy>& enemies,
                int score, int health, int maxHealth,
                bool hideMinimap) const;

    mutable int activeFbHeight = 1;
    std::vector<double>   zbuffer;

    // Overscan buffer used for CPU rotation
    std::vector<uint32_t> overscanBuffer;
};
