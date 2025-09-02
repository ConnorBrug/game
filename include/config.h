#ifndef CONFIG_H
#define CONFIG_H

// ---------------- Window / view ----------------
constexpr int    SCREEN_W = 960;
constexpr int    SCREEN_H = 540;

// Wider base FOV (radians) – "more zoomed out"
constexpr double FOV_BASE = 1.7453292519943295; // 100°

// Wall and sprite vertical scale
constexpr double WALL_SCALE   = 9.5;
constexpr double SPRITE_SCALE = 0.75;

// Converts world-Z units to pixels at default window height
// actual per-frame = this * (currentH / SCREEN_H)
constexpr double PIXELS_PER_UNIT_Z = 38.0;

// Player "eye" height above ground; straight-ahead shots hit body
constexpr double SIGHT_Z = 0.90;

// ---------------- Bullets ----------------
constexpr double BULLET_RADIUS   = 0.05;  // physical radius (world units)
constexpr double BULLET_DIAM     = 0.10;  // sprite scale helper (renderer only)
constexpr double BULLET_HIT_PAD  = 0.010; // tiny forgiveness ring
constexpr double BULLET_LIFE_SEC = 2.0;   // timeout
constexpr double BULLET_SPEED    = 40.5;  // moved here

// ---------------- Gun ----------------
constexpr double MUZZLE_DIST = 0.35;      // muzzle forward offset

// ---------------- Sizes ----------------
constexpr double ENEMY_RADIUS   = 0.28;   // base size for stickman width
constexpr double BODY_CORE_R    = 0.20;   // legacy (UI/layout)
constexpr double PLAYER_RADIUS  = 0.18;

constexpr double ENEMY_MELEE_PAD = 0.05;

// ---------------- Hit zones (legacy bands; analytic test is primary) ----------------
// Bullet.z is world-Z at fire time: pZ + SIGHT_Z + aim_tilt_term
constexpr double Z_HEAD_MIN = 1.15; // >= => head
constexpr double Z_LEG_MAX  = 0.55; // <= => legs
constexpr double Z_BODY_MAX = 1.30; // mid band upper

// All horizontal gates below are FRACTIONS of enemy.radius
constexpr double HEAD_HORIZ_FRAC = 0.55;  // ~small head circle radius
constexpr double LEG_HORIZ_FRAC  = 0.45;  // legs near center
constexpr double BODY_CORE_FRAC  = 0.60;  // torso near center

// Arms live mid-Z, outside torso
constexpr double ARM_INNER_FRAC  = 0.65;  // outside torso core
constexpr double ARM_OUTER_FRAC  = 1.35;  // near hands
constexpr double ARM_SIDE_FRAC   = 0.35;  // min lateral (along player-right)

// Let body win when very close to center (prevents false arms)
constexpr double BODY_MARGIN_FRAC = 0.10;

// Early bullet cull for limb checks (world units)
constexpr double LIMB_PRUNE_MIN = 1.25;
constexpr double LIMB_PRUNE_MUL = 2.00;

// ---------------- Damage / effects ----------------
constexpr int    BODY_HITS_TO_KILL = 2;   // body needs two hits
constexpr int    MELEE_BASE_DAMAGE = 10;  // both arms present
constexpr int    MELEE_ONE_ARM     = 6;   // one arm left
constexpr int    MELEE_NO_ARMS     = 2;   // no arms

constexpr double LEG_SLOW_ONE  = 0.45;    // one leg gone
constexpr double LEG_SLOW_BOTH = 0.15;    // both legs gone (crawl)

constexpr double ENEMY_HIT_FLASH_SEC = 0.95;

const int POINTS_PER_KILL = 1; // tweak to taste


// ---------------- Pose (crawl/tilt) ----------------
constexpr double POSE_TILT_DEG        = 14.0;
constexpr double POSE_CRAWL_RISE_SEC  = 0.25;
constexpr double POSE_TILT_LERP_PER_S = 8.0;

// Far clip for billboarded sprites (shared by renderer & hit-test)
constexpr double SPRITE_FAR_CLIP = 72.0;


#endif // CONFIG_H
