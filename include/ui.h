//
// Created by gunne on 8/29/2025.
//

#ifndef UI_H
#define UI_H

#include <vector>
#include <cstdint>
#include "framebuffer.h"
#include "types.h"

void drawMinimap(Framebuffer& fb,const Player& p,const std::vector<Enemy>& enemies,bool minimized);
void drawBar(Framebuffer& fb,int x,int y,int w,int h,int value,int maxValue,uint32_t bg,uint32_t border);

#endif // UI_H
