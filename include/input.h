#pragma once
#include <SDL.h>

struct InputState {
    // Per-frame accumulators
    float mouseDX = 0, mouseDY = 0;
    bool  mouseLeftDown  = false;
    bool  mouseRightDown = false;

    // Continuous axes/buttons
    float axisForward = 0.f; // W/S → +1/-1 (later: merge with controller)
    float axisStrafe  = 0.f; // D/A → +1/-1
    bool  shiftHeld   = false;
    bool  spaceHeld   = false;

    // One-frame toggles
    bool toggleMinimap = false;
    bool toggleInvertY = false;
    bool resetPitch    = false;

    // App / capture intents
    bool requestCapture = false;
    bool releaseCapture = false;
    bool quitRequested  = false;
};

namespace Input {
    inline void init() {}

    inline void beginFrame(InputState& s) {
        s.mouseDX = s.mouseDY = 0.f;
        s.toggleMinimap = s.toggleInvertY = s.resetPitch = false;
        s.requestCapture = s.releaseCapture = false;
        // held states persist
    }

    void handleEvent(const SDL_Event& e, InputState& s, bool mouseCaptured, bool gameOver);
    void gatherContinuous(InputState& s);
}
