#include "../include/input.h"
#include <algorithm>

static inline float clamp01(float v) { return std::max(-1.f, std::min(1.f, v)); }

void Input::handleEvent(const SDL_Event& e, InputState& s, bool mouseCaptured, bool gameOver) {
    switch (e.type) {
        case SDL_QUIT: s.quitRequested = true; break;

        case SDL_KEYDOWN: {
            const SDL_Keycode k = e.key.keysym.sym;
            if (k == SDLK_ESCAPE) {
                if (mouseCaptured) s.releaseCapture = true; else s.quitRequested = true;
            }
            if (!gameOver) {
                if (k == SDLK_m)  s.toggleMinimap = true;
                if (k == SDLK_F6) s.toggleInvertY = true;
                if (k == SDLK_c)  s.resetPitch    = true;
            }
        } break;

        case SDL_MOUSEMOTION:
            if (!gameOver && mouseCaptured) {
                s.mouseDX += (float)e.motion.xrel;
                s.mouseDY += (float)e.motion.yrel;
            }
        break;

        case SDL_MOUSEBUTTONDOWN:
            if (!gameOver && e.button.button == SDL_BUTTON_LEFT) {
                if (!mouseCaptured) s.requestCapture = true;
                s.mouseLeftDown = true;
            }
        if (!gameOver && e.button.button == SDL_BUTTON_RIGHT) {
            if (!mouseCaptured) s.requestCapture = true;
            s.mouseRightDown = true;
        }
        break;

        case SDL_MOUSEBUTTONUP:
            if (e.button.button == SDL_BUTTON_LEFT)  s.mouseLeftDown  = false;
        if (e.button.button == SDL_BUTTON_RIGHT) s.mouseRightDown = false;
        break;

        default: break;
    }
}

void Input::gatherContinuous(InputState& s) {
    const Uint8* ks = SDL_GetKeyboardState(nullptr);
    float forward = 0.f, strafe = 0.f;
    if (ks[SDL_SCANCODE_W]) forward += 1.f;
    if (ks[SDL_SCANCODE_S]) forward -= 1.f;
    if (ks[SDL_SCANCODE_D]) strafe  += 1.f;
    if (ks[SDL_SCANCODE_A]) strafe  -= 1.f;

    s.axisForward = clamp01(forward);
    s.axisStrafe  = clamp01(strafe);
    s.shiftHeld   = ks[SDL_SCANCODE_LSHIFT] != 0;
    s.spaceHeld   = ks[SDL_SCANCODE_SPACE]  != 0;
}
