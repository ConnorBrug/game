#pragma once
#include <cstdint>
#include <vector>

struct Framebuffer;
struct Player;
struct Enemy;
struct Bullet;

class Renderer {
public:
    Renderer(int w, int h);

    void resize(int w, int h);

    void render(Framebuffer& fb,
                const Player& player,
                const std::vector<Enemy>& enemies,
                const std::vector<Bullet>& bullets,
                int score, int health, int maxHealth,
                bool showMinimap,
                double pZ, double pitchRad,
                double fovIgnored, double useFov,
                double wallScale, double pixelsPerUnitZ,
                double hitFlash,
                double rollDeg);

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

private:
    int W = 1, H = 1;
    int activeFbHeight = 1;
    std::vector<double>   zbuffer;
    std::vector<uint32_t> overscanBuffer;
};
