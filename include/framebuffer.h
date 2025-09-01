#ifndef FRAMEBUFFER_H
#define FRAMEBUFFER_H

// Pull in screen sizes right here so any TU that includes framebuffer.h
// automatically gets SCREEN_W / SCREEN_H.
#include "config.h"
#include <cstdint>
#include <algorithm>

// If something goes weird with include paths, this will fail loudly at compile time.
static_assert(SCREEN_W > 0 && SCREEN_H > 0, "SCREEN_W/SCREEN_H not defined. Did config.h get included?");

struct Framebuffer {
    uint32_t* pixels = nullptr;
    int pitchPixels  = 0;
};

inline void putPixel(Framebuffer& fb, int x, int y, uint32_t c){
    if((unsigned)x < (unsigned)SCREEN_W && (unsigned)y < (unsigned)SCREEN_H)
        fb.pixels[y*fb.pitchPixels + x] = c;
}
inline void drawRect(Framebuffer& fb, int x,int y,int w,int h, uint32_t c){
    int x0 = x < 0 ? 0 : x;
    int y0 = y < 0 ? 0 : y;
    int x1 = (x+w > SCREEN_W) ? SCREEN_W : (x+w);
    int y1 = (y+h > SCREEN_H)? SCREEN_H : (y+h);
    for(int yy=y0; yy<y1; ++yy)
        for(int xx=x0; xx<x1; ++xx)
            fb.pixels[yy*fb.pitchPixels + xx] = c;
}

#endif // FRAMEBUFFER_H
