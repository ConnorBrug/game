//
// Created by gunne on 8/29/2025.
//

#ifndef FONT_H
#define FONT_H

#include <string>
#include <cstdint>
#include "framebuffer.h"

void drawText(Framebuffer& fb,int x,int y,const std::string& t,int s,uint32_t col);
void drawNumber(Framebuffer& fb,int x,int y,int v,int s,uint32_t col);

#endif // FONT_H
