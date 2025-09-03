#define _USE_MATH_DEFINES
#include <SDL.h>
#include <SDL_image.h>   // <-- SDL_image 1.2+ (uses IMG_Load returning SDL_Surface*)
#include <cmath>
#include <vector>
#include <random>
#include <algorithm>
#include <iostream>
#include <string>

#include "../include/config.h"
#include "../include/types.h"
#include "../include/framebuffer.h"
#include "../include/renderer.h"
#include "../include/world.h"
#include "../include/sprites.h"
#include "../include/hittest.h"
#include "../include/pose.h"

#include "../include/parkour_state.h"
#include "../include/input.h"
#include "../include/camera.h"
#include "../include/wallrun.h"
#include "../include/combat.h"

// --- Local helpers kept here ---
static inline bool blockedAt(double x, double y, double rad) {
    if (isWallAt(x, y)) return true;
    return isWallAt(x + rad, y) || isWallAt(x - rad, y) || isWallAt(x, y + rad) || isWallAt(x, y - rad);
}
static inline void tryMove(Player& pl, double& px, double& py, double rad) {
    if (!blockedAt(px, pl.y, rad)) pl.x = px; else { px = pl.x; pl.vx = 0.0; }
    if (!blockedAt(pl.x, py, rad)) pl.y = py; else { py = pl.y; pl.vy = 0.0; }
}
static void pickInternalSize(int outW, int outH, int& inW, int& inH) {
    const double MAX_PIXELS = 7.5e5;
    double target = double(outW) * double(outH);
    double scale = (target > MAX_PIXELS) ? std::sqrt(MAX_PIXELS / target) : 1.0;
    inW = std::max(360, int(std::round(outW * scale)));
    inH = std::max(200, int(std::round(outH * scale)));
    if (inW & 1) ++inW; if (inH & 1) ++inH;
}

// ---- Overlay: PNG loader using SDL_image ----
static SDL_Texture* loadPngTex(SDL_Renderer* r, const char* path) {
    SDL_Surface* s = IMG_Load(path);
    if (!s) {
        std::cerr << "IMG_Load failed (" << path << "): " << IMG_GetError() << "\n";
        return nullptr;
    }
    SDL_Texture* t = SDL_CreateTextureFromSurface(r, s);
    SDL_FreeSurface(s);
    if (!t) {
        std::cerr << "CreateTextureFromSurface failed (" << path << "): " << SDL_GetError() << "\n";
        return nullptr;
    }
    SDL_SetTextureBlendMode(t, SDL_BLENDMODE_BLEND);
    return t;
}

int main(int, char**) {
    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER | SDL_INIT_EVENTS) != 0) {
        std::cerr << "SDL_Init failed: " << SDL_GetError() << "\n"; return 1;
    }

    // Init SDL_image for PNG
    const int want = IMG_INIT_PNG;
    const int got  = IMG_Init(want);
    if ((got & want) != want) {
        std::cerr << "IMG_Init PNG failed: " << IMG_GetError() << "\n";
        SDL_Quit();
        return 1;
    }

    SDL_Window* window = SDL_CreateWindow(
        "microDoom Parkour",
        SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
        SCREEN_W, SCREEN_H,
        SDL_WINDOW_SHOWN | SDL_WINDOW_ALLOW_HIGHDPI | SDL_WINDOW_RESIZABLE);
    if (!window) { std::cerr << "CreateWindow failed: " << SDL_GetError() << "\n"; IMG_Quit(); SDL_Quit(); return 1; }

    SDL_SetHint(SDL_HINT_MOUSE_RELATIVE_MODE_WARP, "1");

    SDL_Renderer* rendererSDL = SDL_CreateRenderer(
        window, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
    if (!rendererSDL) { std::cerr << "CreateRenderer failed: " << SDL_GetError() << "\n"; IMG_Quit(); SDL_Quit(); return 1; }

    SDL_SetHint(SDL_HINT_RENDER_SCALE_QUALITY, "1");

    int outW=0, outH=0; SDL_GetRendererOutputSize(rendererSDL, &outW, &outH);
    int inW=0, inH=0;   pickInternalSize(outW, outH, inW, inH);

    SDL_Texture* frame = SDL_CreateTexture(rendererSDL, SDL_PIXELFORMAT_ARGB8888,
                                           SDL_TEXTUREACCESS_STREAMING, inW, inH);
    if (!frame) { std::cerr << "CreateTexture failed: " << SDL_GetError() << "\n"; IMG_Quit(); SDL_Quit(); return 1; }

    // --- Renderer must exist before onResize (lambda captures it) ---
    Renderer renderer(inW, inH);

    auto recreateBackbuffer = [&](int iw, int ih) {
        if (frame) SDL_DestroyTexture(frame);
        frame = SDL_CreateTexture(rendererSDL, SDL_PIXELFORMAT_ARGB8888,
                                  SDL_TEXTUREACCESS_STREAMING, iw, ih);
        if (!frame) { std::cerr << "CreateTexture failed: " << SDL_GetError() << "\n"; std::exit(1); }
    };
    auto onResize = [&]() {
        int w=0,h=0; SDL_GetRendererOutputSize(rendererSDL, &w, &h); if (w<=0||h<=0) return;
        int newInW,newInH; pickInternalSize(w,h,newInW,newInH);
        if (newInW!=inW || newInH!=inH) { inW=newInW; inH=newInH; recreateBackbuffer(inW,inH); renderer.resize(inW,inH); }
        outW=w; outH=h;
    };

    buildSprites();
    initWorld();

    // --- Overlay textures (PNG) ---
    SDL_Texture* texIdle  = loadPngTex(rendererSDL, "../animations/guns/idle-finger-guns.png");
    SDL_Texture* texLeft  = loadPngTex(rendererSDL, "../animations/guns/left-finger-guns.png");
    SDL_Texture* texRight = loadPngTex(rendererSDL, "../animations/guns/right-finger-guns.png");
    if (!texIdle || !texLeft || !texRight) {
        std::cerr << "Failed to load overlay textures (PNG via SDL_image). Exiting.\n";
        if (texIdle)  SDL_DestroyTexture(texIdle);
        if (texLeft)  SDL_DestroyTexture(texLeft);
        if (texRight) SDL_DestroyTexture(texRight);
        SDL_DestroyTexture(frame);
        SDL_DestroyRenderer(rendererSDL);
        SDL_DestroyWindow(window);
        IMG_Quit();
        SDL_Quit();
        return 1;
    }

    // --- Game state ---
    Player player;
    std::vector<Enemy> enemies;
    std::vector<Bullet> bullets;

    // Infinite spawn tuning
    double spawnCooldown = 0.0;
    const double SPAWN_EVERY_SEC = 2.0;  // how often to attempt a spawn
    const int    MIN_ALIVE = 16;         // keep at least this many alive
    const int    MAX_ALIVE = 32;         // hard cap to avoid swarms exploding FPS

    // Aim
    double aimT = 0.0;

    // Overlay state
    bool   overlayToggleLR = false;            // flip each shot: left <-> right
    double overlayFlashT   = 0.0;              // time remaining to show L/R instead of idle
    constexpr double OVERLAY_FLASH_SEC = 0.10; // seconds to show L/R after a shot

    // --- Spawning helpers & RNG ---
    std::mt19937 rng((unsigned)SDL_GetTicks());
    std::uniform_int_distribution<int> rx(1, MAP_W - 2);
    std::uniform_int_distribution<int> ry(1, MAP_H - 2);

    auto spotFreeTile = [&](int x, int y){ return worldMap[y][x] == 0; };

    auto makeEnemyAt = [&](double ex, double ey){
        Enemy e{};
        e.x = ex; e.y = ey;
        e.alive = true;
        e.phase = (double)(rng() % 1000) / 1000.0;
        e.radius = ENEMY_RADIUS;
        e.bodyHits = 0;
        e.armL = e.armR = e.legL = e.legR = true;
        e.hitFlash = 0.0;
        return e;
    };

    auto spawnOneEnemy = [&](){
        // try to find a valid tile not too close to player
        int x, y, tries = 0;
        const double MIN_PLAYER_DIST = 7.5; // don't pop right on top of the player
        do {
            x = rx(rng); y = ry(rng); ++tries;
        } while (tries < 400 && (
                 !spotFreeTile(x, y) ||
                 std::hypot((x + 0.5) - player.x, (y + 0.5) - player.y) < MIN_PLAYER_DIST));

        Enemy e = makeEnemyAt(x + 0.5, y + 0.5);

        // Reuse a dead slot if possible to avoid growing the vector forever
        auto it = std::find_if(enemies.begin(), enemies.end(),
                               [](const Enemy& en){ return !en.alive; });
        if (it != enemies.end()) *it = e;
        else enemies.push_back(e);
    };

    bool showMinimap = true; int score = 0;
    int  health = 100, maxHealth = 100;
    double damageCooldown = 0.0, hitFlash = 0.0;
    double fireCooldown = 0.0;

    // vertical motion
    double pZ = 0.0, vZ = 0.0;
    int jumpsLeft = 2;
    const double COYOTE_TIME = 0.10; double coyote = 0.0;

    double doubleJumpEaseT = 0.0; // counts down right after a double jump

    // --- Jump tuning ---
    static constexpr double JUMP_VELOCITY           = 25.5;
    static constexpr double GRAVITY_ACC             = 40.0;
    static constexpr double MAX_FALL_SPEED          = -58.0;
    static constexpr double DOUBLE_JUMP_BONUS    = 3;   // extra pop on 2nd jump

    // --- Horizon coupling (reduce pitchy look during jumps)
    static constexpr double HORIZON_Z_FACTOR     = 0.25;
    static constexpr double AIRBORNE_Z_FACTOR    = 0.20;
    static constexpr double DOUBLE_JUMP_Z_FACTOR = 0.12;
    static constexpr double DOUBLE_JUMP_EASE_SEC = 0.15;

    double momentum = 1.0; const double MOMENTUM_MAX = 1.8;

    ParkourState pstate = ParkourState::Normal;
    WallRunState wr;

    // camera & input
    CameraState cam;
    InputState  input;
    Input::init();

    auto setMouseCaptured = [&](bool cap) {
        SDL_ShowCursor(cap ? SDL_DISABLE : SDL_ENABLE);
        SDL_SetRelativeMouseMode(cap ? SDL_TRUE : SDL_FALSE);
        SDL_SetWindowGrab(window, cap ? SDL_TRUE : SDL_FALSE);
        SDL_CaptureMouse(cap ? SDL_TRUE : SDL_FALSE);
    };
    bool mouseCaptured = true; setMouseCaptured(true);

    auto resetRound = [&](){
        enemies.clear();
        const int ENEMY_COUNT = 16;
        player.x = MAP_W * 0.5; player.y = MAP_H * 0.5; player.dir = -M_PI/2.0; player.vx = player.vy = 0.0;

        // Seed starting wave
        for (int i = 0; i < ENEMY_COUNT; ++i) spawnOneEnemy();

        wr = {}; bullets.clear(); fireCooldown=0.0; score=0;
        pZ=0.0; vZ=0.0; health=maxHealth; damageCooldown=0.0; hitFlash=0.0;
        cam.rollDeg=0.0; jumpsLeft=2; coyote=0.0; momentum=1.0;
    };

    resetRound();

    uint64_t last = SDL_GetPerformanceCounter();
    const double freq = (double)SDL_GetPerformanceFrequency();
    bool running = true, gameOver = false;
    bool prevSpace = false;

    while (running) {
        onResize();

        uint64_t now = SDL_GetPerformanceCounter();
        double dt = double(now - last) / freq; last = now;
        if (dt > 0.05) dt = 0.05;

        // timers
        fireCooldown      = std::max(0.0, fireCooldown - dt);
        damageCooldown    = std::max(0.0, damageCooldown - dt);
        hitFlash          = std::max(0.0, hitFlash - dt);
        WallRun::tickTimers(wr, dt);
        Camera::beginFrame(cam, dt);

        // overlay timer
        overlayFlashT     = std::max(0.0, overlayFlashT - dt);

        // --- Input ---
        Input::beginFrame(input);
        SDL_Event e;
        while (SDL_PollEvent(&e)) {
            if (e.type == SDL_WINDOWEVENT &&
               (e.window.event == SDL_WINDOWEVENT_SIZE_CHANGED || e.window.event == SDL_WINDOWEVENT_RESIZED)) {
                onResize();
            }
            Input::handleEvent(e, input, mouseCaptured, gameOver);
        }
        Input::gatherContinuous(input);

        if (input.toggleMinimap) showMinimap = !showMinimap;
        if (input.toggleInvertY) cam.invertY = !cam.invertY;
        if (input.resetPitch)    cam.pitchRad = 0.0;
        if (input.requestCapture) { mouseCaptured = true;  setMouseCaptured(true); }
        if (input.releaseCapture) { mouseCaptured = false; setMouseCaptured(false); }
        if (input.quitRequested)  running = false;

        // Smooth aim factor
        const double aimTarget = (input.mouseRightDown && mouseCaptured && !gameOver) ? 1.0 : 0.0;
        const double lerpStep  = std::clamp(ADS_LERP_PER_S * dt, 0.0, 1.0);
        aimT += (aimTarget - aimT) * lerpStep;

        // Live FOV and zoom multiplier
        const double useFov  = FOV_BASE + (FOV_ZOOM - FOV_BASE) * aimT;
        const double zoomMul = std::tan(FOV_BASE * 0.5) / std::tan(useFov * 0.5);
        const double wallScaleUsed = WALL_SCALE * zoomMul;

        // Mouse sensitivity while aimed
        const float sensMul = float(1.0 + (ADS_SENS_MUL - 1.0) * aimT);
        const float aimedDX = input.mouseDX * sensMul;
        const float aimedDY = input.mouseDY * sensMul;

        // Update camera look with scaled deltas
        Camera::updateLook(cam, mouseCaptured, gameOver, aimedDX, aimedDY, player, pstate, dt);

        // Input vector (world space)
        double fx = std::cos(player.dir),  fy = std::sin(player.dir);
        double sx = -std::sin(player.dir), sy = std::cos(player.dir);
        double ix = fx * double(input.axisForward) + sx * double(input.axisStrafe);
        double iy = fy * double(input.axisForward) + sy * double(input.axisStrafe);
        const double ilen = std::hypot(ix, iy); if (ilen > 0.0) { ix/=ilen; iy/=ilen; }

        // --- Vertical physics / jumping ---
        const bool spaceHeld = input.spaceHeld;
        const bool spacePressed = (spaceHeld && !prevSpace);
        prevSpace = spaceHeld;

        bool grounded = (pZ <= 0.0 && vZ <= 0.0);
        if (grounded) {
            pZ = 0.0; vZ = 0.0;
            if (pstate == ParkourState::WallRun) { pstate = ParkourState::Normal; wr.alongVel = 0.0; }
            jumpsLeft = 2; coyote = 0.0 + 0.10;
        } else {
            coyote = std::max(0.0, coyote - dt);
        }

        // tick the double-jump ease timer
        doubleJumpEaseT = std::max(0.0, doubleJumpEaseT - dt);

        if (!gameOver && spacePressed) {
            if (grounded || coyote > 0.0) {
                vZ = JUMP_VELOCITY;
                pZ += 1e-6;
                jumpsLeft = 1; coyote = 0.0;
            } else if (jumpsLeft > 0 && pstate != ParkourState::WallRun) {
                vZ = std::max(vZ, JUMP_VELOCITY + DOUBLE_JUMP_BONUS);
                pZ += 1e-6;
                --jumpsLeft;
                doubleJumpEaseT = DOUBLE_JUMP_EASE_SEC;
            }
        }

        // --- Momentum ---
        {
            const double inputMag = std::min(1.0, ilen);
            if (pstate == ParkourState::WallRun) momentum += 2.6 * dt;
            else if (grounded) { momentum += 1.4 * inputMag * dt; if (inputMag < 0.2) momentum -= 1.0 * dt; }
            else momentum -= 0.35 * dt;
            if (!std::isfinite(momentum)) momentum = 1.0;
            momentum = std::clamp(momentum, 1.0, MOMENTUM_MAX);
        }

        // --- Ground/Air movement (non-wallrun) ---
        if (pstate == ParkourState::Normal) {
            if (grounded) {
                player.vx += ix * 48.0 * dt;
                player.vy += iy * 48.0 * dt;

                double speed = std::hypot(player.vx, player.vy);
                if ((std::fabs(ix)+std::fabs(iy)) < 0.1 && speed > 0.0) {
                    double ns = std::max(0.0, speed - 60.0 * dt);
                    if (ns < 0.08) { player.vx=0.0; player.vy=0.0; }
                    else { double s = ns/speed; player.vx*=s; player.vy*=s; }
                }

                const double base = 7.2;
                const double sprint = (input.shiftHeld ? 1.3 : 1.0);
                const double lim = base * sprint * momentum;
                double sp = std::hypot(player.vx, player.vy);
                if (sp > lim) { double s = lim/sp; player.vx*=s; player.vy*=s; }
            } else {
                player.vx += ix * 24.0 * dt; // AIR_ACCEL
                player.vy += iy * 24.0 * dt;

                const double base = 7.2;
                const double sprintMul = (input.shiftHeld ? 1.8 : 1.0);
                const double groundCap = base * sprintMul * momentum;
                const double airCap = std::max(9.0, groundCap + 1.5); // AIR_SPEED_CAP
                const double sp = std::hypot(player.vx, player.vy);
                if (sp > airCap) { const double s = airCap/sp; player.vx*=s; player.vy*=s; }

                vZ -= GRAVITY_ACC * dt;
                if (vZ < MAX_FALL_SPEED) vZ = MAX_FALL_SPEED;
            }
        }

        // --- Wall-run state machine ---
        if (pstate == ParkourState::WallRun) {
            double detachDamp = 0.0;
            const ParkourState ns = WallRun::update(wr, player, dt, pZ, vZ, spacePressed, ix, iy, jumpsLeft, momentum, detachDamp);
            if (ns == ParkourState::Normal) Camera::dampPitchAfterDetach(cam, detachDamp);
            pstate = ns;
        } else if (!(pZ <= 0.0 && vZ <= 0.0)) {
            Hit h = WallRun::probe(player.x, player.y);
            if (h.hit) {
                double sx = -std::sin(player.dir), sy = std::cos(player.dir);
                double side = sx * h.nx + sy * h.ny;
                double sign = (side > 0.0 ? -1.0 : +1.0);
                double alongIn = ix * (-h.ny) + iy * (h.nx);
                double speedFactor = std::clamp(std::hypot(player.vx, player.vy) / WallRun::MAX_SPEED, 0.0, 1.0);
                double lean = Camera::ROLL_MAX_DEG + 8.0 * (std::fabs(alongIn) * speedFactor);
                Camera::cuePreRoll(cam, sign, lean);
            }
            if (WallRun::canStart(h, ix, iy, wr)) {
                WallRun::start(wr, h, player, cam.yawVel, cam.pitchVel);
                pstate = ParkourState::WallRun;
            }
        }

        // Integrate vertical
        pZ += vZ * dt;
        if (pZ <= 0.0) {
            pZ = 0.0; vZ = 0.0;
            if (pstate == ParkourState::WallRun) {
                pstate = ParkourState::Normal; wr.alongVel = 0.0;
                Camera::dampPitchAfterDetach(cam, WallRun::POST_DETACH_PITCH_DAMP_SEC);
            }
        }

        // Integrate horizontal
        { double px = player.x + player.vx * dt, py = player.y + player.vy * dt;
          tryMove(player, px, py, (pstate == ParkourState::WallRun) ? WallRun::R_RUN : WallRun::R_NORMAL); }

        // Camera roll
        Camera::updateRoll(cam, dt, pstate, player, wr.normalX, wr.normalY, wr.alongVel, WallRun::MAX_SPEED);

        // Enemies
        double chaseBase = gameOver ? 0.0 : 1.7;
        for (auto& en : enemies) {
            if (!en.alive) continue;
            en.phase += dt; en.hitFlash = std::max(0.0, en.hitFlash - dt);
            double poseMul = Pose::moveSpeedMul(en);
            double chase = chaseBase * poseMul;
            double dx = player.x - en.x, dy = player.y - en.y;
            double d = std::hypot(dx, dy); if (d>1e-6) { dx/=d; dy/=d; }
            double jitter = (poseMul > 0.0 ? 0.6 * std::sin(en.phase * 2.7 + en.phase * 3.1) : 0.0);
            double jx = -dy * jitter, jy = dx * jitter;
            double vx = (dx + jx) * chase * dt, vy = (dy + jy) * chase * dt;
            double nxp = en.x + vx, nyp = en.y; if (!isWallAt(nxp,nyp)) en.x = nxp;
            nxp = en.x; nyp = en.y + vy; if (!isWallAt(nxp,nyp)) en.y = nyp;
        }

        // Melee
        if (!gameOver) {
            bool tookDamage = false;
            for (auto& en : enemies) {
                if (!en.alive) continue;
                double dx = en.x - player.x, dy = en.y - player.y;
                double touch = en.radius + PLAYER_RADIUS + ENEMY_MELEE_PAD;
                if ((dx*dx + dy*dy) < (touch*touch) && damageCooldown <= 0.0) {
                    int dmg = MELEE_BASE_DAMAGE;
                    if (en.bothArmsGone()) dmg = MELEE_NO_ARMS;
                    else if (!en.armL || !en.armR) dmg = MELEE_ONE_ARM;
                    health = std::max(0, health - dmg);
                    damageCooldown = 0.6; tookDamage = true;
                }
            }
            if (tookDamage) hitFlash = 0.18;
            if (health <= 0) { gameOver = true; input.mouseLeftDown = false; pstate = ParkourState::Normal; wr.alongVel=0.0; }
        }

        // Camera height & horizon for this frame
        const double viewZNow = Camera::viewZ(pZ, pstate);
        const double ppuZ     = PIXELS_PER_UNIT_Z * (double)inH / (double)SCREEN_H;

        // Base factor (reduced coupling while airborne / briefly after double jump)
        double zFactor = HORIZON_Z_FACTOR;
        if (!grounded) zFactor = std::min(zFactor, AIRBORNE_Z_FACTOR);
        if (doubleJumpEaseT > 0.0) zFactor = std::min(zFactor, DOUBLE_JUMP_Z_FACTOR);

        const int horizonBase = Camera::computeHorizon(inH, ppuZ, viewZNow * zFactor, cam.pitchRad);

        // Compute effective roll in radians (renderer adds 180° when upside down)
        auto wrapTwoPi = [](double a){ return std::remainder(a, 2.0 * M_PI); };
        const double pitchWrapped = wrapTwoPi(cam.pitchRad);
        const bool   upsideDown   = std::cos(pitchWrapped) < 0.0;
        const double rollEffRad   = (cam.rollDeg + (upsideDown ? 180.0 : 0.0)) * M_PI / 180.0;

        // Fire & bullets
        if (!gameOver && mouseCaptured) {
            bool fired = Combat::tryFire(bullets, player, input.mouseLeftDown, fireCooldown,
                                         inH, ppuZ, cam.pitchRad, viewZNow, aimT, useFov, rollEffRad);
            if (fired) {
                overlayToggleLR = !overlayToggleLR;   // alternate left/right
                overlayFlashT   = OVERLAY_FLASH_SEC;  // show briefly
            }
        }

        Combat::updateBullets(bullets, enemies, player, dt,
                              inW, inH, useFov, ppuZ, horizonBase,
                              cam.pitchRad, wallScaleUsed);

        // Award points for kills
        for (auto& en : enemies) {
            if (!en.alive && en.hitFlash > -1.0) {
                score += POINTS_PER_KILL;
                en.hitFlash = -1.0; // sentinel
            }
        }

        // --- Infinite spawn ---
        if (!gameOver) {
            spawnCooldown = std::max(0.0, spawnCooldown - dt);

            int alive = 0;
            for (const auto& en : enemies) if (en.alive) ++alive;

            if (alive < MIN_ALIVE) {
                spawnOneEnemy();
                spawnCooldown = 0.25;
            } else if (alive < MAX_ALIVE && spawnCooldown <= 0.0) {
                spawnOneEnemy();
                spawnCooldown = SPAWN_EVERY_SEC;
            }
        }

        // Render
        void* raw = nullptr; int pitchB = 0;
        SDL_LockTexture(frame, nullptr, &raw, &pitchB);
        {
            Framebuffer fb{ (uint32_t*)raw, pitchB / 4 };
            renderer.render(fb, player, enemies, bullets,
                            score, health, maxHealth, showMinimap,
                            viewZNow, cam.pitchRad,
                            FOV_BASE,        // unused legacy param
                            useFov,          // live FOV drives rays
                            wallScaleUsed,   // MUST match hit-test
                            ppuZ,
                            hitFlash,
                            cam.rollDeg);
        }
        SDL_UnlockTexture(frame);

        SDL_RenderClear(rendererSDL);
        SDL_RenderCopy(rendererSDL, frame, nullptr, nullptr);

        // ----- overlay draw (full-screen stretch) -----
        auto drawOverlay = [&](SDL_Texture* tex) {
            if (!tex) return;

            // Stretch to entire output (fills regardless of aspect).
            SDL_Rect dst { 0, 0, outW, outH };
            SDL_RenderCopy(rendererSDL, tex, nullptr, &dst);

            // If you ever want it to rotate with camera roll:
            // SDL_RenderCopyEx(rendererSDL, tex, nullptr, &dst, cam.rollDeg, nullptr, SDL_FLIP_NONE);
        };
        SDL_Texture* overlayTex = (overlayFlashT > 0.0)
            ? (overlayToggleLR ? texRight : texLeft)
            : texIdle;
        drawOverlay(overlayTex);
        // ----------------------------------------

        SDL_RenderPresent(rendererSDL);
    }

    if (frame) SDL_DestroyTexture(frame);
    if (texIdle)  SDL_DestroyTexture(texIdle);
    if (texLeft)  SDL_DestroyTexture(texLeft);
    if (texRight) SDL_DestroyTexture(texRight);

    SDL_DestroyRenderer(rendererSDL);
    SDL_DestroyWindow(window);
    IMG_Quit();
    SDL_Quit();
    return 0;
}
