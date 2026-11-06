#ifndef FIRE_CONTROL_H_
#define FIRE_CONTROL_H_

#include "Debouncer.h"

#include <chrono>
#include <cstdint>

struct FireControlConfig
{
    unsigned int aimDebounceSamples = 10;
    int aimFrames = 3;
    int shotIntervalMs = 800;
    int pushRetractMs = 200;
    double rearmAngleDeg = 2.2;
};

struct FireControlOutput
{
    bool enabled = false;
    int shotsFired = 0;
    std::uint8_t pushFlag = 1;
};

class FireControl
{
public:
    explicit FireControl(const FireControlConfig& cfg = {});

    void SetEnabled(bool enabled);
    bool IsEnabled() const;

    FireControlOutput Update(
        bool targetValid,
        bool aimValid,
        bool isAimed,
        double yawErrDeg,
        double pitchErrDeg,
        std::chrono::steady_clock::time_point now);

private:
    FireControlConfig cfg_;
    Debouncer aimedDebouncer_;
    int aimedFrames_ = 0;
    int shotsFired_ = 0;
    bool enabled_ = false;
    bool shotArmed_ = true;
    bool pushRetracting_ = false;
    std::chrono::steady_clock::time_point lastShotAt_;
    std::chrono::steady_clock::time_point pushRetractStartedAt_;
};

#endif // FIRE_CONTROL_H_
