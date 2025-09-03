#ifndef CONFIG_H
#define CONFIG_H

// ---------------- Window / view ----------------
constexpr int    SCREEN_W = 960;
constexpr int    SCREEN_H = 540;

// Wider base FOV (radians) – "more zoomed out"
constexpr double FOV_BASE = 1.7453292519943295; // 100°

// ADD: zoomed (ADS) FOV (smaller angle = tighter zoom)
constexpr double FOV_ZOOM = 1.0471975511965976; // 60°

// ADD: mouse sensitivity multiplier while zooming (0..1)
constexpr double ADS_SENS_MUL    = 0.45;

// ADD: how quickly aim eases (higher = snappier)
constexpr double ADS_LERP_PER_S  = 10.0;

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
constexpr double BULLET_DIAM     = 0.05;  // sprite scale helper (renderer only)
constexpr double BULLET_HIT_PAD  = 0.5;   // tiny forgiveness ring
constexpr double BULLET_LIFE_SEC = 5.0;   // timeout
constexpr double BULLET_SPEED    = 200.5;  // moved here

// NEW: fraction of player velocity that projectiles inherit (1 = full)
constexpr double BULLET_INHERIT_VEL = 1.0;

// NEW: treat z<=this as ground contact for bullet removal
constexpr double BULLET_GROUND_EPS = 0.02;

// ---------------- Gun ----------------
// Base forward offset (hip) and ADS multiplier
constexpr double MUZZLE_DIST          = 4;
constexpr double MUZZLE_DIST_ADS_MUL  = 1;

// Horizontal zero distance (hip) and ADS multiplier
constexpr double MUZZLE_CONVERGE_M    = 10.0;
constexpr double CONVERGE_ADS_MUL     = .3;

// Lateral & vertical offsets
constexpr double MUZZLE_SIDE          = 0;
constexpr double MUZZLE_Z_OFF_HIP     = 0;
constexpr double MUZZLE_Z_OFF_ADS     = 0; // closer to scope bore in ADS

// Rate of fire (seconds between shots)
constexpr double FIRE_DELAY      = 0.12;  // ~500 RPM

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
