#include "../include/ui.h"
#include "../include/config.h"   // for SCREEN_W/SCREEN_H
#include "../include/util.h"
#include "../include/world.h"

void drawMinimap(Framebuffer& fb,
                 const Player& p,
                 const std::vector<Enemy>& enemies,
                 bool minimized)
{
    if (minimized) return;

    int scale = 4;

    // top-right corner
    int offX = SCREEN_W - MAP_W * scale - 10;
    int offY = 10;

    // backdrop
    for (int y = 0; y < MAP_H * scale; ++y)
        for (int x = 0; x < MAP_W * scale; ++x)
            putPixel(fb, offX + x, offY + y, ARGB(150, 10, 10, 10));

    // tiles
    for (int y = 0; y < MAP_H; ++y) for (int x = 0; x < MAP_W; ++x) {
        uint32_t c = worldMap[y][x] ? ARGB(255, 200, 200, 200) : ARGB(255, 30, 30, 30);
        for (int yy = 0; yy < scale; ++yy) for (int xx = 0; xx < scale; ++xx)
            putPixel(fb, offX + x * scale + xx, offY + y * scale + yy, c);
    }

    // enemies
    for (const auto& en : enemies) {
        if (!en.alive) continue;
        int ex = offX + int(en.x * scale), ey = offY + int(en.y * scale);
        for (int dy = -1; dy <= 1; ++dy) for (int dx = -1; dx <= 1; ++dx)
            putPixel(fb, ex + dx, ey + dy, ARGB(255, 50, 255, 120));
    }

    // player
    int px = offX + int(p.x * scale), py = offY + int(p.y * scale);
    for (int dy = -2; dy <= 2; ++dy) for (int dx = -2; dx <= 2; ++dx)
        putPixel(fb, px + dx, py + dy, ARGB(255, 255, 60, 60));
}

void drawBar(Framebuffer& fb, int x, int y, int w, int h,
             int value, int maxValue, uint32_t bg, uint32_t border)
{
    drawRect(fb, x, y, w, h, bg);
    float t = (maxValue > 0) ? std::clamp(value / float(maxValue), 0.f, 1.f) : 0.f;

    uint32_t green  = ARGB(255, 60, 220, 60);
    uint32_t yellow = ARGB(255, 230, 200, 60);
    uint32_t red    = ARGB(255, 220, 60, 60);

    auto lerp = [&](uint32_t a, uint32_t b, float u){
        auto ch=[&](uint32_t c,int sh){ return (c>>sh)&0xFF; };
        uint8_t ar=ch(a,16),ag=ch(a,8),ab=ch(a,0), br=ch(b,16),bg=ch(b,8),bb=ch(b,0);
        uint8_t r=uint8_t(ar+(br-ar)*u), g=uint8_t(ag+(bg-ag)*u), bl=uint8_t(ab+(bb-ab)*u);
        return ARGB(255, r, g, bl);
    };

    uint32_t fg = (t > 0.5f) ? lerp(yellow, green, (t - 0.5f)/0.5f)
                             : lerp(red,    yellow, t/0.5f);

    drawRect(fb, x+1, y+1, int((w-2)*t), h-2, fg);

    for (int i=0;i<w;++i){ putPixel(fb,x+i,y,border); putPixel(fb,x+i,y+h-1,border); }
    for (int i=0;i<h;++i){ putPixel(fb,x,y+i,border); putPixel(fb,x+w-1,y+i,border); }
}
