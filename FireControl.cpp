#include "FireControl.h"

#include <cmath>
#include <iostream>

FireControl::FireControl(const FireControlConfig& cfg)
    : cfg_{cfg},
      aimedDebouncer_{cfg.aimDebounceSamples, DebounceEdge::Rising},
      lastShotAt_{std::chrono::steady_clock::now() - std::chrono::seconds(10)}
{
}

void FireControl::SetEnabled(bool enabled)
{
    enabled_ = enabled;
}

bool FireControl::IsEnabled() const
{
    return enabled_;
}

FireControlOutput FireControl::Update(
    bool targetValid,
    bool aimValid,
    bool isAimed,
    double yawErrDeg,
    double pitchErrDeg,
    std::chrono::steady_clock::time_point now)
{
    const bool targetMovedAway = aimValid &&
        std::hypot(yawErrDeg, pitchErrDeg) > cfg_.rearmAngleDeg;
    bool aimed = false;
    if (!targetValid || targetMovedAway)
    {
        aimedDebouncer_.reset(false);
    }
    else
    {
        aimed = aimedDebouncer_.update(isAimed);
    }

    if (aimed)
    {
        ++aimedFrames_;
        const auto sinceLastShot =
            std::chrono::duration_cast<std::chrono::milliseconds>(
                now - lastShotAt_).count();
        if (enabled_ && aimedFrames_ >= cfg_.aimFrames && shotArmed_ &&
            sinceLastShot >= cfg_.shotIntervalMs)
        {
            shotArmed_ = false;
            lastShotAt_ = now;
            ++shotsFired_;
            pushRetracting_ = true;
            pushRetractStartedAt_ = now;
            std::cout << "[FIRE] 第 " << shotsFired_ << " 发\n";
        }
    }
    else
    {
        aimedFrames_ = 0;
        shotArmed_ = true;
    }

    if (pushRetracting_ &&
        std::chrono::duration_cast<std::chrono::milliseconds>(
            now - pushRetractStartedAt_).count() >= cfg_.pushRetractMs)
    {
        pushRetracting_ = false;
    }

    return {enabled_, shotsFired_,
            static_cast<std::uint8_t>(pushRetracting_ ? 0 : 1)};
}
