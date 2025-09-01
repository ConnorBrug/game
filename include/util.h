//
// Created by gunne on 8/29/2025.
//

#ifndef UTIL_H
#define UTIL_H

#include <cstdint>
#include <cmath>

inline uint32_t ARGB(uint8_t a,uint8_t r,uint8_t g,uint8_t b){
    return (uint32_t(a)<<24)|(uint32_t(r)<<16)|(uint32_t(g)<<8)|uint32_t(b);
}
inline uint32_t Modulate(uint32_t c,float f){
    uint8_t a=(c>>24)&255,r=(c>>16)&255,g=(c>>8)&255,b=c&255;
    auto clamp=[&](float v){ return (uint8_t)std::fmin(255.0f,v); };
    return ARGB(a, clamp(r*f), clamp(g*f), clamp(b*f));
}
inline uint32_t lerpColor(uint32_t a,uint32_t b,float t){
    auto ch=[&](uint32_t c,int sh){ return (c>>sh)&0xFF; };
    uint8_t ar=ch(a,16),ag=ch(a,8),ab=ch(a,0), br=ch(b,16),bg=ch(b,8),bb=ch(b,0);
    uint8_t r=uint8_t(ar+(br-ar)*t), g=uint8_t(ag+(bg-ag)*t), bl=uint8_t(ab+(bb-ab)*t);
    return ARGB(255,r,g,bl);
}
#endif //UTIL_H
