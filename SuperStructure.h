#ifndef SUPER_STRUCTURE_H_
#define SUPER_STRUCTURE_H_

#include "Serial.h"

#include <cmath>
#include <optional>

struct axisData
{
    double yaw;
    double pitch;

    axisData(double yawValue = 0.0, double pitchValue = 0.0)
        : yaw(yawValue), pitch(pitchValue)
    {}
};

struct SuperStructureConfig
{
    double zeroYawDeg = 0.0;
    double zeroPitchDeg = 0.0;
    double aimToleranceDeg = 0.6;
    int stayTimeMs = 1000;
    bool zeroOnLost = false;
};

struct OutputData
{
    axisData axis;
    axisData aimErrorDeg;
    bool aimValid = false;
};

enum class State
{
    aiming,
    aimed,
    zeroing,
    zeroed,
    lost
};

class SuperStructure
{
public:
    explicit SuperStructure(const SuperStructureConfig& cfg = {})
        : cfg_{cfg}
    {
        output_.axis = axisData(cfg_.zeroYawDeg, cfg_.zeroPitchDeg);
    }

    void setAimTarget(const std::optional<axisData>& aimTargetDeg)
    {
        if (aimTargetDeg &&
            (!std::isfinite(aimTargetDeg->yaw) ||
             !std::isfinite(aimTargetDeg->pitch)))
        {
            aimTargetDeg_.reset();
            return;
        }
        aimTargetDeg_ = aimTargetDeg;
    }

    void periodic(const TRoMaC::GimbalFeedback& feedback,
                  int now,
                  bool feedbackValid = true)
    {
        feedbackValid_ = feedbackValid &&
            std::isfinite(feedback.yaw_angle) &&
            std::isfinite(feedback.pitch_angle);
        if (feedbackValid_)
        {
            feedbackDeg_ = axisData(feedback.yaw_angle, feedback.pitch_angle);
        }

        const bool tgtValid = aimTargetDeg_.has_value();
        output_.aimValid = tgtValid && feedbackValid_;
        output_.aimErrorDeg = output_.aimValid
            ? axisData(aimTargetDeg_->yaw - feedbackDeg_.yaw,
                       aimTargetDeg_->pitch - feedbackDeg_.pitch)
            : axisData();

        switch (state_)
        {
        case State::aiming:
            if (!tgtValid)
            {
                state_ = State::lost;
                tgtLostAt_ = now;
            }
            else if (output_.aimValid && isWithinAimTolerance())
            {
                state_ = State::aimed;
            }
            break;

        case State::aimed:
            if (!tgtValid)
            {
                state_ = State::lost;
                tgtLostAt_ = now;
            }
            else if (!output_.aimValid || !isWithinAimTolerance())
            {
                state_ = State::aiming;
            }
            break;

        case State::zeroed:
            if (tgtValid)
            {
                state_ = State::aiming;
            }
            break;

        case State::zeroing:
            if (tgtValid)
            {
                state_ = State::aiming;
            }
            else if (feedbackValid_ &&
                     std::fabs(feedbackDeg_.yaw - cfg_.zeroYawDeg) <=
                         cfg_.aimToleranceDeg &&
                     std::fabs(feedbackDeg_.pitch - cfg_.zeroPitchDeg) <=
                         cfg_.aimToleranceDeg)
            {
                state_ = State::zeroed;
            }
            break;

        case State::lost:
            if (tgtValid)
            {
                state_ = State::aiming;
            }
            else if (cfg_.zeroOnLost && now - tgtLostAt_ > cfg_.stayTimeMs)
            {
                state_ = State::zeroing;
            }
            break;
        }

        if (state_ == State::aiming || state_ == State::aimed)
        {
            output_.axis = feedbackValid_ && tgtValid
                ? *aimTargetDeg_
                : feedbackDeg_;
        }
        else if (state_ == State::zeroing || state_ == State::zeroed)
        {
            output_.axis = feedbackValid_
                ? axisData(cfg_.zeroYawDeg, cfg_.zeroPitchDeg)
                : feedbackDeg_;
        }
        else
        {
            output_.axis = feedbackDeg_;
        }
    }

    const OutputData& getoutput() const
    {
        return output_;
    }

    const char* getstateName() const
    {
        switch (state_)
        {
        case State::aiming: return "aiming";
        case State::aimed: return "aimed";
        case State::zeroing: return "zeroing";
        case State::zeroed: return "zeroed";
        case State::lost: return "lost";
        }
        return "unknown";
    }

    bool isAimed() const
    {
        return state_ == State::aimed && output_.aimValid;
    }

private:
    SuperStructureConfig cfg_;
    State state_ = State::lost;
    OutputData output_;
    axisData feedbackDeg_;
    std::optional<axisData> aimTargetDeg_;
    int tgtLostAt_ = 0;
    bool feedbackValid_ = false;

    bool isWithinAimTolerance() const
    {
        const double yawErr = std::remainder(output_.aimErrorDeg.yaw, 360.0);
        const double pitchErr = std::remainder(output_.aimErrorDeg.pitch, 360.0);
        return std::fabs(yawErr) <= cfg_.aimToleranceDeg &&
               std::fabs(pitchErr) <= cfg_.aimToleranceDeg;
    }
};

#endif // SUPER_STRUCTURE_H_
