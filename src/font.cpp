#include <cctype>
#include <string>

#include "../include/font.h"
#include "../include/util.h"

struct Glyph{ uint8_t rows[5]; };

static const Glyph DIGITS[10] = {
  {{0b111,0b101,0b101,0b101,0b111}}, {{0b010,0b110,0b010,0b010,0b111}},
  {{0b111,0b001,0b111,0b100,0b111}}, {{0b111,0b001,0b111,0b001,0b111}},
  {{0b101,0b101,0b111,0b001,0b001}}, {{0b111,0b100,0b111,0b001,0b111}},
  {{0b111,0b100,0b111,0b101,0b111}}, {{0b111,0b001,0b001,0b010,0b010}},
  {{0b111,0b101,0b111,0b101,0b111}}, {{0b111,0b101,0b111,0b001,0b111}}
};

static bool getLetterGlyph(char ch, Glyph& g){
    switch((char)std::toupper((unsigned char)ch)){
        case 'A': g={{0b010,0b101,0b111,0b101,0b101}}; return true;
        case 'C': g={{0b111,0b100,0b100,0b100,0b111}}; return true;
        case 'E': g={{0b111,0b100,0b111,0b100,0b111}}; return true;
        case 'G': g={{0b111,0b100,0b101,0b101,0b111}}; return true;
        case 'M': g={{0b101,0b111,0b111,0b101,0b101}}; return true;
        case 'O': g={{0b111,0b101,0b101,0b101,0b111}}; return true;
        case 'P': g={{0b110,0b101,0b110,0b100,0b100}}; return true;
        case 'R': g={{0b110,0b101,0b110,0b101,0b101}}; return true;
        case 'S': g={{0b111,0b100,0b111,0b001,0b111}}; return true;
        case 'V': g={{0b101,0b101,0b101,0b101,0b010}}; return true;
        default: return false;
    }
}

static void drawGlyph(Framebuffer& fb,int x,int y,const Glyph& g,int s,uint32_t col){
    for(int r=0;r<5;++r) for(int c=0;c<3;++c)
        if(g.rows[r]&(1<<(2-c))) for(int yy=0; yy<s; ++yy) for(int xx=0; xx<s; ++xx)
            putPixel(fb,x+c*s+xx,y+r*s+yy,col);
}

static void drawChar(Framebuffer& fb,int x,int y,char ch,int s,uint32_t col){
    if(ch>='0'&&ch<='9'){ drawGlyph(fb,x,y,DIGITS[ch-'0'],s,col); return; }
    if(ch==':'){ Glyph g={{0b000,0b010,0b000,0b010,0b000}}; drawGlyph(fb,x,y,g,s,col); return; }
    if(ch==' ') return;
    Glyph g; if(getLetterGlyph(ch,g)) drawGlyph(fb,x,y,g,s,col);
}

void drawText(Framebuffer& fb,int x,int y,const std::string& t,int s,uint32_t col){
    int cx=x; for(char ch: t){ drawChar(fb,cx,y,(char)std::toupper((unsigned char)ch),s,col); cx += (3*s + s); }
}

void drawNumber(Framebuffer& fb,int x,int y,int v,int s,uint32_t col){
    if(v==0){ drawGlyph(fb,x,y,DIGITS[0],s,col); return; }
    std::string srt=std::to_string(v); int cx=x;
    for(char ch: srt){ drawGlyph(fb,cx,y,DIGITS[ch-'0'],s,col); cx+=(3*s+s); }
}
