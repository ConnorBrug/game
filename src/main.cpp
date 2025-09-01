// main.cpp
//==============================================================
// microGame Parkour — Uses external HitTest + Pose helpers
//==============================================================

#define _USE_MATH_DEFINES
#include <SDL.h>
#include <cmath>
#include <cstdint>
#include <vector>
#include <iostream>
#include <algorithm>
#include <random>

#include "../include/config.h"
#include "../include/types.h"
#include "../include/framebuffer.h"
#include "../include/renderer.h"
#include "../include/world.h"
#include "../include/sprites.h"
#include "../include/hittest.h"   // NEW: analytic screen-space limb classifier
#include "../include/pose.h"      // NEW: pose/movement helpers

//==============================================================
// Globals & Tunables
//==============================================================

// Needed by screen-space projection helpers
static constexpr double kHalfPi = 1.5707963267948966; // π/2

// Bullets
static const double BULLET_LIFE = BULLET_LIFE_SEC;

//==============================================================
// Parkour / Movement (unchanged behavior, with Pose speed mul)
//==============================================================

enum class ParkourState { Normal, WallRun };

static const double WALL_DETECTION_DIST = 0.33;
struct Hit { bool hit = false; double nx = 0, ny = 0; };

static Hit probeWall(double x, double y) {
    Hit h;
    bool wL = isWallAt(x - WALL_DETECTION_DIST, y),
         wR = isWallAt(x + WALL_DETECTION_DIST, y),
         wU = isWallAt(x, y - WALL_DETECTION_DIST),
         wD = isWallAt(x, y + WALL_DETECTION_DIST);
    if (wL || wR || wU || wD) {
        h.hit = true;
        h.nx = (wL ? 1.0 : 0.0) + (wR ? -1.0 : 0.0);
        h.ny = (wU ? 1.0 : 0.0) + (wD ? -1.0 : 0.0);
        double len = std::hypot(h.nx, h.ny); if (len > 0) { h.nx /= len; h.ny /= len; }
    }
    return h;
}

// ----------- Movement / parkour tuning -----------
static double wallNormalX = 0.0, wallNormalY = 0.0;
static double wallTanX = 0.0, wallTanY = 0.0;
static double alongVel = 0.0;

static const double WALLRUN_ACCEL = 26.0;
static const double WALLRUN_BRAKE = 32.0;
static const double WALLRUN_FRICTION = 5.0;
static const double WALLRUN_MAX_SPEED = 8.0;
static const double WALL_STICK_PULL = 0.022;
static const double WALL_SLIDE_GRAVITY = 0.34;
static const double WALL_SLIDE_MAX_FALL = -3.6;

static const double WALL_JUMP_PUSH = 1.45;
static const double WALL_TAN_CARRY = 1.6;
static const double AIR_SPEED_CAP = 9.0;

static const double WALL_LATCH_MAXINIT = 3.0;
static const double WALLRUN_STICKY_TIME = 0.12;

static double wallrunSticky = 0.0;
static double wallRelatchLock = 0.0;

// Vertical motion
static double pZ = 0.0;
static double vZ = 0.0;
// crisp stopping when no input on ground
static const double GROUND_STOP_BRAKE = 60.0;
static const double STOP_EPS = 0.08;

// Player radius (for collisions)
static const double R_NORMAL = 0.18;
static const double R_RUN = 0.14;

// Jump & gravity
static const double GRAVITY = 16.0;
static const double JUMP_VEL = 8.6;
static const int MAX_JUMPS = 2;

// Momentum (bunny/flow)
static const double GROUND_FRICTION = 8.0;
static const double AIR_FRICTION = 0.0;
static const double AIR_ACCEL = 24.0;
static const double COYOTE_TIME = 0.10;
static double coyote = 0.0;

static double momentum = 1.0;
static const double MOMENTUM_MAX = 1.8;
static const double MOMENTUM_ADD_GROUND = 1.4;  // /s at full stick
static const double MOMENTUM_ADD_WALL = 2.6;    // /s while wallrunning
static const double MOMENTUM_DECAY_IDLE = 1.0;  // /s when not moving
static const double MOMENTUM_DECAY_AIR = 0.35;  // /s in air

// camera roll (during wall-run)
static const double ROLL_MAX_DEG = 13.0;
static const double ROLL_RESP = 12.0;
static double rollDeg = 0.0;
static double rollTarget = 0.0;

static const double PRE_ROLL_LEAD_SEC = 0.033;
static double preRollTimer = 0.0;
static double preRollSign = 0.0;
static double preRollLean = 0.0;

// ---- Pitch dampening (NO auto-tilt) ----
static const double WALL_PITCH_DAMP = 0.35;
static const double POST_DETACH_PITCH_DAMP_SEC = 0.22;
static double pitchDampenTimer = 0.0;

//==============================================================
// Utility helpers
//==============================================================

static inline bool blockedAt(double x, double y, double rad) {
    if (isWallAt(x, y)) return true;
    return isWallAt(x + rad, y) || isWallAt(x - rad, y) || isWallAt(x, y + rad) || isWallAt(x, y - rad);
}
static inline void tryMove(Player& pl, double& px, double& py, double rad) {
    if (!blockedAt(px, pl.y, rad)) pl.x = px; else { px = pl.x; pl.vx = 0.0; }
    if (!blockedAt(pl.x, py, rad)) pl.y = py; else { py = pl.y; pl.vy = 0.0; }
}
static inline std::pair<double, double> intendedMove(const Uint8* ks, const Player& p) {
    double fx = std::cos(p.dir),  fy = std::sin(p.dir);
    double sx = -std::sin(p.dir), sy = std::cos(p.dir);
    double mx = 0, my = 0;
    if (ks[SDL_SCANCODE_W]) { mx += fx; my += fy; }
    if (ks[SDL_SCANCODE_S]) { mx -= fx; my -= fy; }
    if (ks[SDL_SCANCODE_D]) { mx += sx; my += sy; }
    if (ks[SDL_SCANCODE_A]) { mx -= sx; my -= sy; }
    double len = std::hypot(mx, my); if (len > 0) { mx /= len; my /= len; }
    return { mx, my };
}
static inline bool canStartWallRun(const Hit& h, const Uint8* ks, const Player& p) {
    if (!h.hit) return false;
    if (wallRelatchLock > 0.0) return false;
    auto [ix, iy] = intendedMove(ks, p);
    double tx = -h.ny, ty = h.nx;
    double along = ix * tx + iy * ty;
    double toward = ix * h.nx + iy * h.ny;
    return (std::fabs(along) > 0.35) && (toward < 0.45);
}
static inline void popOffWall(Player& pl, double nx, double ny, double extra = 0.02) {
    const double pop = (R_NORMAL - R_RUN) + extra;
    pl.x += nx * pop; pl.y += ny * pop;
}

// Dynamic internal resolution chooser
static void pickInternalSize(int outW, int outH, int& inW, int& inH) {
    const double MAX_PIXELS = 7.5e5; // ~1220x615
    double target = double(outW) * double(outH);
    double scale = (target > MAX_PIXELS) ? std::sqrt(MAX_PIXELS / target) : 1.0;
    inW = std::max(360, int(std::round(outW * scale)));
    inH = std::max(200, int(std::round(outH * scale)));
    if (inW & 1) ++inW;
    if (inH & 1) ++inH;
}

//==============================================================
// Main
//==============================================================
int main(int, char**) {
    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER | SDL_INIT_EVENTS) != 0) {
        std::cerr << "SDL_Init failed: " << SDL_GetError() << "\n"; return 1;
    }

    SDL_Window* window = SDL_CreateWindow(
        "microDoom Parkour",
        SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
        SCREEN_W, SCREEN_H,
        SDL_WINDOW_SHOWN | SDL_WINDOW_ALLOW_HIGHDPI | SDL_WINDOW_RESIZABLE);
    if (!window) { std::cerr << "CreateWindow failed: " << SDL_GetError() << "\n"; return 1; }

    SDL_SetHint(SDL_HINT_MOUSE_RELATIVE_MODE_WARP, "1");

    SDL_Renderer* rendererSDL = SDL_CreateRenderer(
        window, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
    if (!rendererSDL) { std::cerr << "CreateRenderer failed: " << SDL_GetError() << "\n"; return 1; }

    SDL_SetHint(SDL_HINT_RENDER_SCALE_QUALITY, "1");

    int outW = 0, outH = 0;
    SDL_GetRendererOutputSize(rendererSDL, &outW, &outH);

    int inW = 0, inH = 0;
    pickInternalSize(outW, outH, inW, inH);

    SDL_Texture* frame = nullptr;
    auto recreateBackbuffer = [&](int iw, int ih) {
        if (frame) { SDL_DestroyTexture(frame); frame = nullptr; }
        frame = SDL_CreateTexture(rendererSDL, SDL_PIXELFORMAT_ARGB8888,
                                  SDL_TEXTUREACCESS_STREAMING, iw, ih);
        if (!frame) { std::cerr << "CreateTexture failed: " << SDL_GetError() << "\n"; std::exit(1); }
    };
    recreateBackbuffer(inW, inH);

    buildSprites();
    initWorld();

    Renderer renderer(inW, inH);

    auto onResize = [&]() {
        int w = 0, h = 0;
        SDL_GetRendererOutputSize(rendererSDL, &w, &h);
        if (w <= 0 || h <= 0) return;
        int newInW, newInH;
        pickInternalSize(w, h, newInW, newInH);
        if (newInW != inW || newInH != inH) {
            inW = newInW; inH = newInH;
            recreateBackbuffer(inW, inH);
            renderer.resize(inW, inH);
        }
        outW = w; outH = h;
    };

    Player player;
    std::vector<Enemy> enemies;
    std::vector<Bullet> bullets;

    bool showMinimap = true; int score = 0;

    // mouse look
    double pitchRad = 0.0;
    bool mouseCaptured = true;
    bool invertY = false;
    float smoothFactor = 0.20f;
    float yawVel = 0.0f, pitchVel = 0.0f;

    const double FOV_FIXED = FOV_BASE;
    double fov = FOV_FIXED;

    // health/firing
    int health = 100, maxHealth = 100;
    double damageCooldown = 0.0, hitFlash = 0.0;
    double fireCooldown = 0.0;
    const double FIRE_DELAY = 0.12;
    bool mouseLeft = false;

    // jump
    int jumpsLeft = MAX_JUMPS;

    ParkourState pstate = ParkourState::Normal;

    auto resetRound = [&](std::vector<Enemy>& enemiesRef) {
        enemiesRef.clear();
        std::mt19937 rng((unsigned)SDL_GetTicks());
        std::uniform_int_distribution<int> rx(1, MAP_W - 2), ry(1, MAP_H - 2);
        auto spotFree = [&](int x, int y) { return worldMap[y][x] == 0; };
        const int ENEMY_COUNT = 16;
        for (int i = 0; i < ENEMY_COUNT; ++i) {
            int x, y; int tries = 0;
            do { x = rx(rng); y = ry(rng); tries++; }
            while ((!spotFree(x, y) || std::hypot(x - MAP_W * 0.5, y - MAP_H * 0.5) < 8.0) && tries < 400);
            Enemy e;
            e.x = x + 0.5; e.y = y + 0.5; e.alive = true; e.phase = double(rng() % 1000) / 1000.0;
            e.radius = ENEMY_RADIUS;
            e.bodyHits = 0; e.armL = e.armR = e.legL = e.legR = true; e.hitFlash = 0.0;
            enemiesRef.push_back(e);
        }
        player.x = MAP_W * 0.5; player.y = MAP_H * 0.5; player.dir = -M_PI / 2.0;
        player.vx = player.vy = 0.0;

        wallrunSticky = 0.0; wallRelatchLock = 0.0;
        wallNormalX = wallNormalY = 0.0; wallTanX = wallTanY = 0.0; alongVel = 0.0;

        bullets.clear(); fireCooldown = 0.0; score = 0;
        pZ = 0.0; vZ = 0.0;
        health = maxHealth; damageCooldown = 0.0; hitFlash = 0.0;
        rollDeg = 0.0;
        jumpsLeft = MAX_JUMPS; coyote = 0.0;
        momentum = 1.0;
    };
    resetRound(enemies);

    auto setMouseCaptured = [&](bool cap) {
        mouseCaptured = cap;
        SDL_ShowCursor(cap ? SDL_DISABLE : SDL_ENABLE);
        SDL_SetRelativeMouseMode(cap ? SDL_TRUE : SDL_FALSE);
        SDL_SetWindowGrab(window, cap ? SDL_TRUE : SDL_FALSE);
        SDL_CaptureMouse(cap ? SDL_TRUE : SDL_FALSE);
    };
    setMouseCaptured(true);

    uint64_t last = SDL_GetPerformanceCounter();
    double freq = (double)SDL_GetPerformanceFrequency();
    bool running = true, gameOver = false;
    bool prevSpace = false;

    const double PITCH_RATE_CAP = 0.040;

    while (running) {
        onResize();

        uint64_t now = SDL_GetPerformanceCounter();
        double dt = double(now - last) / freq; last = now;
        if (dt > 0.05) dt = 0.05;

        fireCooldown = std::max(0.0, fireCooldown - dt);
        wallRelatchLock = std::max(0.0, wallRelatchLock - dt);
        pitchDampenTimer = std::max(0.0, pitchDampenTimer - dt);
        preRollTimer = std::max(0.0, preRollTimer - dt);

        // ---------- events ----------
        float rawDX = 0.0f, rawDY = 0.0f; SDL_Event e;
        while (SDL_PollEvent(&e)) {
            if (e.type == SDL_QUIT) running = false;

            if (e.type == SDL_WINDOWEVENT &&
               (e.window.event == SDL_WINDOWEVENT_SIZE_CHANGED || e.window.event == SDL_WINDOWEVENT_RESIZED)) {
                onResize();
            }

            if (e.type == SDL_KEYDOWN) {
                if (e.key.keysym.sym == SDLK_ESCAPE) { if (mouseCaptured) setMouseCaptured(false); else running = false; }
                if (e.key.keysym.sym == SDLK_m) showMinimap = !showMinimap;
                if (gameOver && e.key.keysym.sym == SDLK_r) { gameOver = false; resetRound(enemies); }
                if (e.key.keysym.sym == SDLK_F6) invertY = !invertY;
                if (e.key.keysym.sym == SDLK_c)  pitchRad = 0.0;
            }

            if (!gameOver) {
                if (e.type == SDL_MOUSEMOTION && mouseCaptured) { rawDX += (float)e.motion.xrel; rawDY += (float)e.motion.yrel; }
                if (e.type == SDL_MOUSEBUTTONDOWN) {
                    if (e.button.button == SDL_BUTTON_LEFT) { if (!mouseCaptured) setMouseCaptured(true); mouseLeft = true; }
                }
                if (e.type == SDL_MOUSEBUTTONUP) {
                    if (e.button.button == SDL_BUTTON_LEFT)  mouseLeft = false;
                }
            }
        }

        // ---------- look ----------
        if (mouseCaptured && !gameOver) {
            float inv = std::clamp(1.0f - smoothFactor, 0.0f, 1.0f);
            yawVel   = yawVel   * smoothFactor + rawDX * inv;
            pitchVel = pitchVel * smoothFactor + rawDY * inv;

            player.dir += yawVel * 0.0025f;

            double dyAngle = (invertY ? 1.0 : -1.0) * (double)pitchVel * 0.0018;

            double pitchDamp = (pstate == ParkourState::WallRun || pitchDampenTimer > 0.0) ? WALL_PITCH_DAMP : 1.0;
            dyAngle *= pitchDamp;

            if (dyAngle >  PITCH_RATE_CAP) dyAngle =  PITCH_RATE_CAP;
            if (dyAngle < -PITCH_RATE_CAP) dyAngle = -PITCH_RATE_CAP;

            pitchRad += dyAngle;
            if (pitchRad >  1.35) pitchRad =  1.35;
            if (pitchRad < -1.35) pitchRad = -1.35;
        }

        const Uint8* ks = SDL_GetKeyboardState(nullptr);
        bool spaceHeld = ks[SDL_SCANCODE_SPACE] != 0;
        bool spacePressed = (spaceHeld && !prevSpace);

        // ---------- vertical physics ----------
        bool grounded = (pZ <= 0.0 && vZ <= 0.0);
        if (grounded) {
            pZ = 0.0; vZ = 0.0;
            if (pstate == ParkourState::WallRun) { pstate = ParkourState::Normal; alongVel = 0.0; }
            jumpsLeft = MAX_JUMPS;
            coyote = COYOTE_TIME;
        } else {
            coyote = std::max(0.0, coyote - dt);
        }

        // Jump / Double jump with coyote
        if (!gameOver && spacePressed) {
            if (grounded || coyote > 0.0) {
                vZ = JUMP_VEL; pZ += 1e-6;
                jumpsLeft = MAX_JUMPS - 1; coyote = 0.0;
            } else if (jumpsLeft > 0 && pstate != ParkourState::WallRun) {
                vZ = JUMP_VEL; pZ += 1e-6;
                jumpsLeft--;
            }
        }

        // ---------- momentum update ----------
        {
            auto [ix, iy] = intendedMove(ks, player);
            double inputMag = std::min(1.0, std::hypot(ix, iy));
            if (pstate == ParkourState::WallRun) {
                momentum += MOMENTUM_ADD_WALL * dt;
            } else if (grounded) {
                momentum += MOMENTUM_ADD_GROUND * inputMag * dt;
                if (inputMag < 0.2) momentum -= MOMENTUM_DECAY_IDLE * dt;
            } else {
                momentum -= MOMENTUM_DECAY_AIR * dt;
            }
            if (!std::isfinite(momentum)) momentum = 1.0;
            momentum = std::clamp(momentum, 1.0, MOMENTUM_MAX);
        }

        // ---------- ground/air movement ----------
        if (pstate == ParkourState::Normal) {
            auto [ix, iy] = intendedMove(ks, player);

            if (grounded) {
                player.vx += ix * 48.0 * dt;
                player.vy += iy * 48.0 * dt;

                double speed = std::hypot(player.vx, player.vy);
                if (std::fabs(ix) + std::fabs(iy) < 0.1 && speed > 0.0) {
                    double ns = std::max(0.0, speed - GROUND_STOP_BRAKE * dt);
                    if (ns < STOP_EPS) { player.vx = 0.0; player.vy = 0.0; }
                    else {
                        double s = ns / speed;
                        player.vx *= s; player.vy *= s;
                    }
                }

                double base = 7.2;
                double sprint = (ks[SDL_SCANCODE_LSHIFT] ? 1.8 : 1.0);
                double lim = base * sprint * momentum;

                double sp = std::hypot(player.vx, player.vy);
                if (sp > lim) { player.vx *= lim / sp; player.vy *= lim / sp; }
            } else {
                player.vx += ix * AIR_ACCEL * dt;
                player.vy += iy * AIR_ACCEL * dt;

                double base = 7.2;
                double sprintMul = (ks[SDL_SCANCODE_LSHIFT] ? 1.8 : 1.0);
                double groundCap = base * sprintMul * momentum;

                double airCap = std::max(AIR_SPEED_CAP, groundCap + 1.5);

                double sp = std::hypot(player.vx, player.vy);
                if (sp > airCap) {
                    double s = airCap / sp;
                    player.vx *= s; player.vy *= s;
                }

                vZ -= GRAVITY * dt;
            }
        }

        // ---------- wall-run ----------
        if (pstate == ParkourState::WallRun) {
            if (pZ <= 0.08 && vZ <= 0.0) {
                pstate = ParkourState::Normal;
                alongVel = 0.0;
                wallrunSticky = 0.0;
                popOffWall(player, wallNormalX, wallNormalY);
                wallRelatchLock = 0.12;
                pitchDampenTimer = POST_DETACH_PITCH_DAMP_SEC;
            } else {
                wallrunSticky = std::max(0.0, wallrunSticky - dt);

                Hit h = probeWall(player.x - wallNormalX * 0.10, player.y - wallNormalY * 0.10);
                if (!h.hit) {
                    double aheadX = player.x + wallTanX * 0.30;
                    double aheadY = player.y + wallTanY * 0.30;
                    Hit h2 = probeWall(aheadX, aheadY);
                    if (h2.hit) { h = h2; wallrunSticky = WALLRUN_STICKY_TIME; }
                } else {
                    wallrunSticky = WALLRUN_STICKY_TIME;
                }

                if (!h.hit) {
                    pstate = ParkourState::Normal;
                    alongVel = 0.0;
                    popOffWall(player, wallNormalX, wallNormalY);
                    wallRelatchLock = 0.12;
                    pitchDampenTimer = POST_DETACH_PITCH_DAMP_SEC;
                } else {
                    wallNormalX = h.nx; wallNormalY = h.ny;
                    wallTanX = -h.ny;   wallTanY = h.nx;

                    auto [ix, iy] = intendedMove(ks, player);
                    double inAlong = ix * wallTanX + iy * wallTanY;
                    if (inAlong * alongVel < -0.01) {
                        double s = std::fabs(alongVel);
                        double ns = std::max(0.0, s - WALLRUN_BRAKE * dt);
                        alongVel = (alongVel > 0 ? 1 : -1) * ns;
                    } else {
                        if (std::fabs(inAlong) > 0.05) alongVel += inAlong * WALLRUN_ACCEL * dt;
                        else {
                            double s = std::fabs(alongVel);
                            double ns = std::max(0.0, s - WALLRUN_FRICTION * dt);
                            alongVel = (alongVel > 0 ? 1 : -1) * ns;
                        }
                    }
                    alongVel = std::clamp(alongVel, -WALLRUN_MAX_SPEED, WALLRUN_MAX_SPEED);

                    double px = player.x + wallTanX * alongVel * dt;
                    double py = player.y + wallTanY * alongVel * dt;
                    px -= h.nx * WALL_STICK_PULL;
                    py -= h.ny * WALL_STICK_PULL;

                    double vNorm = player.vx * h.nx + player.vy * h.ny;
                    player.vx -= vNorm * h.nx;
                    player.vy -= vNorm * h.ny;
                    player.vx = wallTanX * alongVel;
                    player.vy = wallTanY * alongVel;

                    tryMove(player, px, py, R_RUN);

                    vZ -= GRAVITY * WALL_SLIDE_GRAVITY * dt;
                    if (vZ < WALL_SLIDE_MAX_FALL) vZ = WALL_SLIDE_MAX_FALL;

                    if (std::fabs(inAlong) < 0.02 && std::fabs(alongVel) < 0.20 && vZ < -0.5) {
                        pstate = ParkourState::Normal;
                        alongVel = 0.0;
                        pitchDampenTimer = POST_DETACH_PITCH_DAMP_SEC;
                    }

                    if (spacePressed) {
                        vZ = JUMP_VEL;

                        double carry = std::clamp(alongVel, -WALL_TAN_CARRY, WALL_TAN_CARRY);
                        player.vx = wallTanX * (carry * 0.50) + h.nx * WALL_JUMP_PUSH;
                        player.vy = wallTanY * (carry * 0.50) + h.ny * WALL_JUMP_PUSH;

                        player.x += h.nx * 0.08;
                        player.y += h.ny * 0.08;

                        double sp = std::hypot(player.vx, player.vy);
                        if (sp > AIR_SPEED_CAP) { double s = AIR_SPEED_CAP / sp; player.vx *= s; player.vy *= s; }

                        pstate = ParkourState::Normal;
                        alongVel = 0.0;
                        wallRelatchLock = 0.28;
                        jumpsLeft = 1;

                        momentum = std::min(MOMENTUM_MAX, momentum + 0.20);
                        pitchDampenTimer = POST_DETACH_PITCH_DAMP_SEC;
                    }
                }
            }
        }

        // ---- latch to wall when airborne ----
        if (!(pZ <= 0.0 && vZ <= 0.0) && pstate == ParkourState::Normal) {
            Hit h = probeWall(player.x, player.y);
            if (h.hit) {
                double sx = -std::sin(player.dir), sy = std::cos(player.dir);
                double side = sx * h.nx + sy * h.ny;
                preRollSign = (side > 0.0 ? -1.0 : +1.0);

                auto [ix, iy] = intendedMove(ks, player);
                double tx = -h.ny, ty = h.nx;    // tangent
                double alongIn = ix * tx + iy * ty;

                double speedFactor = std::clamp(std::hypot(player.vx, player.vy) / WALLRUN_MAX_SPEED, 0.0, 1.0);
                preRollLean = (ROLL_MAX_DEG + 8.0 * (std::fabs(alongIn) * speedFactor));

                if (std::fabs(alongIn) > 0.15) {
                    preRollTimer = PRE_ROLL_LEAD_SEC;
                }
            }
            if (canStartWallRun(h, ks, player)) {
                pstate = ParkourState::WallRun;
                wallNormalX = h.nx; wallNormalY = h.ny;
                wallTanX = -h.ny;   wallTanY = h.nx;
                wallrunSticky = WALLRUN_STICKY_TIME;

                double proj = player.vx * wallTanX + player.vy * wallTanY;
                alongVel = std::clamp(proj, -WALL_LATCH_MAXINIT, WALL_LATCH_MAXINIT);

                yawVel = 0.0f; pitchVel = 0.0f;
            }
        }

        // ---- integrate vertical ----
        pZ += vZ * dt;
        if (pZ <= 0.0) {
            pZ = 0.0; vZ = 0.0;
            if (pstate == ParkourState::WallRun) {
                pstate = ParkourState::Normal;
                alongVel = 0.0;
                popOffWall(player, wallNormalX, wallNormalY);
                wallRelatchLock = 0.12;
                pitchDampenTimer = POST_DETACH_PITCH_DAMP_SEC;
            }
        }

        // ---- integrate horizontal ----
        {
            double px = player.x + player.vx * dt;
            double py = player.y + player.vy * dt;
            tryMove(player, px, py, (pstate == ParkourState::WallRun) ? R_RUN : R_NORMAL);
        }

        // ---------- CAMERA ROLL TARGET ----------
        {
            if (pstate == ParkourState::WallRun) {
                double sx = -std::sin(player.dir), sy = std::cos(player.dir);
                double side = sx * wallNormalX + sy * wallNormalY;
                double speedFactor = std::clamp(std::fabs(alongVel) / WALLRUN_MAX_SPEED, 0.0, 1.0);
                double lean = ROLL_MAX_DEG + 8.0 * speedFactor;
                rollTarget = (side > 0.0 ? -lean : +lean);
            } else {
                if (pstate == ParkourState::WallRun) {
                    double sx = -std::sin(player.dir), sy = std::cos(player.dir);
                    double side = sx * wallNormalX + sy * wallNormalY;
                    double speedFactor = std::clamp(std::fabs(alongVel) / WALLRUN_MAX_SPEED, 0.0, 1.0);
                    double lean = ROLL_MAX_DEG + 8.0 * speedFactor;
                    rollTarget = (side > 0.0 ? -lean : +lean);
                } else if (preRollTimer > 0.0) {
                    rollTarget = preRollSign * preRollLean;
                } else {
                    rollTarget = 0.0;
                }
                double alpha = std::clamp(dt * ROLL_RESP, 0.0, 1.0);
                rollDeg += (rollTarget - rollDeg) * alpha;
            }
            double alpha = std::clamp(dt * ROLL_RESP, 0.0, 1.0);
            rollDeg += (rollTarget - rollDeg) * alpha;
        }

        // ---------- enemies ----------
        double chaseBase = gameOver ? 0.0 : 1.7;
        for (auto& en : enemies) {
            if (!en.alive) continue;

            en.phase += dt;
            en.hitFlash = std::max(0.0, en.hitFlash - dt);

            // NEW: movement speed multiplier from pose (0 if both arms+legs gone)
            double poseMul = Pose::moveSpeedMul(en);
            double chaseSpeed = chaseBase * poseMul;

            double dx = player.x - en.x, dy = player.y - en.y;
            double d = std::hypot(dx, dy); if (d > 1e-6) { dx /= d; dy /= d; }
            double jitter = (poseMul > 0.0 ? 0.6 * std::sin(en.phase * 2.7 + en.phase * 3.1) : 0.0);
            double jx = -dy * jitter, jy = dx * jitter;

            double vx = (dx + jx) * chaseSpeed * dt, vy = (dy + jy) * chaseSpeed * dt;
            double nxp = en.x + vx, nyp = en.y; if (!isWallAt(nxp, nyp)) en.x = nxp;
            nxp = en.x; nyp = en.y + vy; if (!isWallAt(nxp, nyp)) en.y = nyp;
        }

        // --------- melee damage ----------
        if (!gameOver) {
            damageCooldown = std::max(0.0, damageCooldown - dt);
            bool tookDamage = false;

            for (auto& en : enemies) {
                if (!en.alive) continue;
                double dx = en.x - player.x, dy = en.y - player.y;
                double touch = en.radius + PLAYER_RADIUS + ENEMY_MELEE_PAD;
                if ((dx * dx + dy * dy) < (touch * touch) && damageCooldown <= 0.0) {
                    int dmg = MELEE_BASE_DAMAGE;
                    if (en.bothArmsGone()) dmg = MELEE_NO_ARMS;
                    else if (!en.armL || !en.armR) dmg = MELEE_ONE_ARM;
                    health = std::max(0, health - dmg);
                    damageCooldown = 0.6; tookDamage = true;
                }
            }
            if (tookDamage) hitFlash = 0.18;
            hitFlash = std::max(0.0, hitFlash - dt);
            if (health <= 0) { gameOver = true; mouseLeft = false; pstate = ParkourState::Normal; alongVel = 0.0; }
        }

        // ------------------- SHOOT -------------------
        if (!gameOver && mouseCaptured && mouseLeft && fireCooldown <= 0.0) {
            const double fxc = std::cos(player.dir);
            const double fyc = std::sin(player.dir);

            // Muzzle forward so we don't hit ourselves
            const double bx = player.x + fxc * MUZZLE_DIST;
            const double by = player.y + fyc * MUZZLE_DIST;

            // Lock bullet Z to the exact crosshair plane
            const double ppuZ = PIXELS_PER_UNIT_Z * (double(inH) / double(SCREEN_H));
            const double zAtCrosshairWorld = pZ + ((double)inH * 0.5 / ppuZ) * std::tan(pitchRad);

            bullets.push_back({
                bx, by,
                std::cos(player.dir) * BULLET_SPEED,
                std::sin(player.dir) * BULLET_SPEED,
                BULLET_LIFE,
                zAtCrosshairWorld,
                0.0   // zSlope: flat (we keep Z locked)
            });

            fireCooldown = FIRE_DELAY;
        }

        // ---------- bullets update (tunneling-safe + screen-primitive hit test) ----------
        using HitZone = HitTest::Zone;

        // Build a render context once per frame for hit testing
        const double ppuZ_frame = PIXELS_PER_UNIT_Z * (double(inH) / double(SCREEN_H));
        const int horizonBase = inH / 2 + int(std::lround(pZ * ppuZ_frame + (inH / 2.0) * std::tan(pitchRad)));
        HitTest::Context hrc{ inW, inH, fov, ppuZ_frame, horizonBase, pitchRad, &player };

        for (size_t i = 0; i < bullets.size();) {
            Bullet& b = bullets[i];

            const double sx = b.x,  sy = b.y;
            const double nx = b.x + b.vx * dt;
            const double ny = b.y + b.vy * dt;

            bool remove = false;

            const double dx = nx - sx, dy = ny - sy;
            const double dist = std::hypot(dx, dy);
            const double maxStep = std::max(1e-6, (BULLET_RADIUS + BULLET_HIT_PAD) * 0.6);
            const int steps = std::max(1, (int)std::ceil(dist / maxStep));

            const double zFixed = b.z; // Z locked to crosshair plane

            for (int s = 1; s <= steps && !remove; ++s) {
                const double t  = double(s) / steps;
                const double px = sx + dx * t;
                const double py = sy + dy * t;

                // Walls stop bullets
                if (isWallAt(px, py)) { remove = true; break; }

                for (auto& en : enemies) {
                    if (!en.alive) continue;

                    // Early prune — widened to never miss skinny limbs
                    double rr = std::max(en.radius * LIMB_PRUNE_MUL, LIMB_PRUNE_MIN) + BULLET_RADIUS + BULLET_HIT_PAD;
                    double ex = en.x - px, ey = en.y - py;
                    if (ex * ex + ey * ey > rr * rr) continue;

                    // Exact analytic classification in screen space
                    HitZone hz = HitTest::classify(hrc, en, zFixed, px, py);
                    if (hz == HitZone::None) continue;

                    switch (hz) {
                        case HitZone::Head:
                            en.alive = false; en.hitFlash = ENEMY_HIT_FLASH_SEC; score++; break;
                        case HitZone::Body:
                            en.bodyHits++; en.hitFlash = ENEMY_HIT_FLASH_SEC;
                            if (en.bodyHits >= BODY_HITS_TO_KILL) { en.alive = false; score++; }
                            break;
                        case HitZone::ArmL: if (en.armL) { en.armL = false; en.hitFlash = ENEMY_HIT_FLASH_SEC; } break;
                        case HitZone::ArmR: if (en.armR) { en.armR = false; en.hitFlash = ENEMY_HIT_FLASH_SEC; } break;
                        case HitZone::LegL: if (en.legL) { en.legL = false; en.hitFlash = ENEMY_HIT_FLASH_SEC; } break;
                        case HitZone::LegR: if (en.legR) { en.legR = false; en.hitFlash = ENEMY_HIT_FLASH_SEC; } break;
                        default: break;
                    }
                    remove = true; // consume bullet on hit
                    break;
                }
            }

            if (!remove) {
                b.x = nx; b.y = ny;
                b.life -= dt;
                if (b.life <= 0.0) remove = true;
            }

            if (remove) bullets.erase(bullets.begin() + i);
            else        ++i;
        }

        // ---------- RENDER ----------
        void* raw = nullptr; int pitchB = 0;
        SDL_LockTexture(frame, nullptr, &raw, &pitchB);
        {
            Framebuffer fb{ (uint32_t*)raw, pitchB / 4 };
            const double pixelsPerUnitZ = PIXELS_PER_UNIT_Z * (double(inH) / double(SCREEN_H));
            renderer.render(fb, player, enemies, bullets,
                            score, health, maxHealth, showMinimap,
                            pZ, pitchRad,
                            fov, fov,
                            WALL_SCALE, pixelsPerUnitZ,
                            hitFlash,
                            rollDeg);
        }
        SDL_UnlockTexture(frame);

        SDL_RenderClear(rendererSDL);
        SDL_RenderCopy(rendererSDL, frame, nullptr, nullptr);
        SDL_RenderPresent(rendererSDL);

        prevSpace = spaceHeld;
    }

    if (frame) SDL_DestroyTexture(frame);
    SDL_DestroyRenderer(rendererSDL);
    SDL_DestroyWindow(window);
    SDL_Quit();
    return 0;
}
