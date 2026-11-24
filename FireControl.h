#ifndef FIRE_CONTROL_H
#define FIRE_CONTROL_H


#include <chrono>
#include <cmath>
#include <iostream>
#include <Serial.h>
#include <mutex>
#include <thread>
#include <atomic>
#include "Debouncer.h"
#include "MathUtils.h"
#include "Constants.h"

// 控制台
static bool kEnableFire = false;  // 开火总开关

// 跨帧要记住的
 static Debouncer aimedDebouncer(kAIM_STABLE_FRAMES, DebounceEdge::Rising);
 static int  g_shotsFired = 0;
 static bool g_pushRetracting = false;
 static std::chrono::steady_clock::time_point g_pushRetractStartedAt;
 static bool g_aimedStable = false;
 static bool g_fireArmed = true;

bool ifInTol(float yaw_error, float pitch_error)
{
    return std::abs(yaw_error) <= kAIM_TOLERANCE && std::abs(pitch_error) <= kAIM_TOLERANCE;
}



static uint8_t updateFireControl(bool targetValid,
                                 bool feedbackValid,
                                 const AxisData& desiredAngles,
                                 const TRoMaC::GimbalFeedback& feedback,
                                 std::chrono::steady_clock::time_point now)
{
    const bool validAimData = targetValid && feedbackValid &&
        std::isfinite(desiredAngles.yaw) &&
        std::isfinite(desiredAngles.pitch) &&
        std::isfinite(feedback.yaw_angle) &&
        std::isfinite(feedback.pitch_angle);

    if (!validAimData)
    {
        g_aimedStable = false;
        aimedDebouncer.reset(false);
    }
    else
    {
        const double yawError = desiredAngles.yaw - static_cast<double>(feedback.yaw_angle);
        const double pitchError = desiredAngles.pitch - static_cast<double>(feedback.pitch_angle);
        g_aimedStable = ifInTol(static_cast<float>(yawError), static_cast<float>(pitchError));
        g_aimedStable = aimedDebouncer.update(g_aimedStable);
    }

    if (!g_aimedStable)
    {
        g_fireArmed = true;
    }

    if (kEnableFire && g_aimedStable && g_fireArmed && !g_pushRetracting)
    {
        g_fireArmed = false;
        g_pushRetracting = true;
        g_pushRetractStartedAt = now;
        ++g_shotsFired;
        std::cout << "[FIRE] 第 " << g_shotsFired << " 发\n";
    }

    if (g_pushRetracting &&
        now - g_pushRetractStartedAt >= std::chrono::milliseconds(kPushRetractMs))
    {
        g_pushRetracting = false;
    }

    return g_pushRetracting ? 0 : 1;
}
#endif // FIRE_CONTROL_H