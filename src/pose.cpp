#include "../include/pose.h"
#include <cmath>
#ifdef _MSC_VER
  #include <corecrt_math_defines.h>
#endif

namespace {
    inline double deg2rad(double d){ return d * M_PI / 180.0; }
    inline bool bothLegsGone(const Enemy& e){ return !e.legL && !e.legR; }
    inline bool anyArmLeft (const Enemy& e){ return  e.armL || e.armR;  }
    inline bool bothArmsGone(const Enemy& e){ return !e.armL && !e.armR; }
}

namespace Pose {

    State classify(const Enemy& en) {
        if (bothLegsGone(en) && bothArmsGone(en)) return State::Immobile;
        if (bothLegsGone(en) && anyArmLeft(en))   return State::Crawl;
        return State::Normal;
    }

    float visibleHeightScale(const Enemy& en) {
        switch (classify(en)) {
            case State::Crawl:    return 0.62f; // a tad shorter so torso really rests
            case State::Immobile: return 0.58f; // stump sinks a touch more
            default:              return 1.0f;
        }
    }

    int groundDropSrcPx(const Enemy& en) {
        switch (classify(en)) {
            case State::Crawl:    return 12; // more than before; ensures contact
            case State::Immobile: return 18; // was 10; bump to kill “floating”
            default:              return 0;
        }
    }

    int crawlPullExtraDropSrcPx(const Enemy& en) {
        if (classify(en) != State::Crawl) return 0;
        // Dip on pull beat, 0..6 px
        const double t = en.phase * 4.0;
        const double dip = 3.0 * (1.0 - std::cos(t));
        return int(std::lround(dip));
    }

    int forwardLeanSrcPx(const Enemy& en) {
        switch (classify(en)) {
            case State::Crawl: {
                // Very subtle lean so the 4px arm reach touches the floor.
                const double base  = 2.0; // was 6.0 — “barely” lean
                const double pulse = 1.0 * (0.5 * (1.0 - std::cos(en.phase * 4.0)));
                return int(std::lround(base + pulse)); // ~2..3 src px
            }
            case State::Immobile:
                return 3;  // slight slump
            default:
                return 0;
        }
    }

    // Keep this as a small sideways wobble; forward lean is handled above.
    double crawlTiltRadians(const Enemy& en) {
        if (classify(en) != State::Crawl) return 0.0;
        const double wobble = 2.0 * std::sin(en.phase * 2.0);
        return -deg2rad(wobble); // negative = bias toward camera in our shear convention
    }

    int armReachSrcPx(const Enemy& en, bool leftArm) {
        if (classify(en) != State::Crawl) return 0;
        if ((leftArm && !en.armL) || (!leftArm && !en.armR)) return 0;

        // Alternate arms: π out of phase.
        const double A = 4.0;     // max reach (source px) – slightly more than before
        const double w = 4.0;     // cadence
        const double phase = leftArm ? 0.0 : M_PI;
        double t = en.phase * w + phase;
        double reach01 = 0.5 * (1.0 - std::cos(t)); // 0..1 (easey)
        return int(std::lround(A * reach01));
    }

    double moveSpeedMul(const Enemy& en) {
        switch (classify(en)) {
            case State::Normal:   return 1.0;
            case State::Crawl:    return 0.55;
            case State::Immobile: return 0.0;
        }
        return 1.0;
    }
}
