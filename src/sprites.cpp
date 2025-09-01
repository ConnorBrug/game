#include "../include/sprites.h"
#include "../include/util.h"
#include <algorithm>
#include <cmath>
#include <vector>

const int SM_W = 32, SM_H = 64;
// Upgrade bullet texture from 1x1 to 8x8 for nicer upscaling
const int BL_W = 8,  BL_H = 8;

std::vector<uint32_t> g_stickman;
std::vector<uint32_t> g_bullet;
std::vector<uint8_t>  g_stickman_mask;

static inline void P(std::vector<uint32_t>& buf,int W,int H,int x,int y,uint32_t c){
    if((unsigned)x<(unsigned)W && (unsigned)y<(unsigned)H) buf[y*W+x]=c;
}
static inline void PM(std::vector<uint8_t>& buf,int W,int H,int x,int y,uint8_t m, uint8_t priority){
    if((unsigned)x>=(unsigned)W || (unsigned)y>=(unsigned)H) return;
    // Prefer non-body over body, head over everything, then arms/legs
    static const uint8_t prio[] = {0, 4, 1, 3, 3, 2, 2}; // None,H,B,AL,AR,LL,LR
    uint8_t& dst = buf[y*W+x];
    if (prio[m] >= prio[dst]) dst = m;
}

static void buildStickman(){
    g_stickman.assign(SM_W*SM_H, ARGB(0,0,0,0));
    g_stickman_mask.assign(SM_W*SM_H, SM_None);

    uint32_t col=ARGB(255,240,240,240);
    int cx=SM_W/2, cy=14, r=8;

    // Head (filled circle)
    for(int y=cy-r;y<=cy+r;++y)
        for(int x=cx-r;x<=cx+r;++x){
            int dx=x-cx,dy=y-cy;
            if(dx*dx+dy*dy<=r*r){ P(g_stickman,SM_W,SM_H,x,y,col); PM(g_stickman_mask,SM_W,SM_H,x,y,SM_Head,0); }
        }

    // Torso (center 3px)
    for(int y=cy+r; y<cy+r+22; ++y){
        P(g_stickman,SM_W,SM_H,cx,y,col);     PM(g_stickman_mask,SM_W,SM_H,cx,  y,SM_Body,0);
        P(g_stickman,SM_W,SM_H,cx-1,y,col);   PM(g_stickman_mask,SM_W,SM_H,cx-1,y,SM_Body,0);
        P(g_stickman,SM_W,SM_H,cx+1,y,col);   PM(g_stickman_mask,SM_W,SM_H,cx+1,y,SM_Body,0);
    }

    // Arms (diagonals)
    for(int i=0;i<16;++i){
        int lx = cx-1-i, ly = cy+r+8+i/2;
        int rx = cx+1+i, ry = cy+r+8+i/2;
        P(g_stickman,SM_W,SM_H,lx,ly,col);    PM(g_stickman_mask,SM_W,SM_H,lx,ly,SM_ArmL,0);
        P(g_stickman,SM_W,SM_H,rx,ry,col);    PM(g_stickman_mask,SM_W,SM_H,rx,ry,SM_ArmR,0);
    }

    // Legs (diagonals)
    for(int i=0;i<18;++i){
        int lx = cx-1-i/2, ly = cy+r+22+i;
        int rx = cx+1+i/2, ry = cy+r+22+i;
        P(g_stickman,SM_W,SM_H,lx,ly,col);    PM(g_stickman_mask,SM_W,SM_H,lx,ly,SM_LegL,0);
        P(g_stickman,SM_W,SM_H,rx,ry,col);    PM(g_stickman_mask,SM_W,SM_H,rx,ry,SM_LegR,0);
    }
}

static void buildBullet(){
    g_bullet.assign(BL_W*BL_H, ARGB(0,0,0,0));
    const float cx=(BL_W-1)*0.5f, cy=(BL_H-1)*0.5f;
    const float r = (float)std::min(BL_W,BL_H)*0.5f - 0.5f; // soft circle
    for(int y=0;y<BL_H;++y) for(int x=0;x<BL_W;++x){
        float dx=x-cx, dy=y-cy;
        float d = std::sqrt(dx*dx+dy*dy);
        if(d<=r){
            float t = std::max(0.f, std::min(1.f, d / r));
            int R = (int)std::round(255 * (1.0f - 0.10f * t));
            int G = (int)std::round(240 * (1.0f - 0.25f * t));
            int B = (int)std::round( 80 * (1.0f - 0.40f * t));
            int A = (int)std::round(255 * (1.0f - 0.0f  * t));
            g_bullet[y*BL_W+x] = ARGB(A,R,G,B);
        }
    }
}

void buildSprites(){
    buildStickman();
    buildBullet();
}
