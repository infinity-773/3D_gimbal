#ifndef SUPER_STRUCTURE_H_
#define SUPER_STRUCTURE_H_

#include "PIDController.h"
#include "MathUtils.h"
#include "Serial.h"
#include "TargetContainer.h"

#include <cmath>

struct axisData
{
    double yaw;
    double pitch;

    axisData(double yawValue = 0.0, double pitchValue = 0.0)
        : yaw(yawValue), pitch(pitchValue)
    {}
};

enum class ControlOutputMode
{
    Position,
    Incremental
};

struct SuperStructureConfig
{
    double zeroYaw;
    double zeroPitch;
    double aimU;
    double aimV;
    double focalLengthPx;
    double yawSign;
    double pitchSign;
    double yawBias;
    double pitchBias;
    double deadbandPx;
    double aimTolerancePx;
    double integralEnableOffsetPx;
    int stayTimeMs;
    PIDConfig yawPid;
    PIDConfig pitchPid;

    SuperStructureConfig()
        : zeroYaw(0.0),
          zeroPitch(-3.0),
          aimU(640.0),
          aimV(360.0),
          focalLengthPx(1300.0),
          yawSign(1.0),
          pitchSign(1.0),
          yawBias(0.0),
          pitchBias(0.0),
          deadbandPx(2.0),
          aimTolerancePx(10.0),
          integralEnableOffsetPx(40.0),
          stayTimeMs(1000),
          yawPid(),
          pitchPid()
    {}
};

struct OutputData
{
    axisData axis;
    axisData targetOffsetPx;
    axisData targetAngleError;
    bool fire;
    ControlOutputMode mode;

    OutputData()
        : axis(),
          targetOffsetPx(),
          targetAngleError(),
          fire(false),
          mode(ControlOutputMode::Position)
    {}
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
    explicit SuperStructure(
        const SuperStructureConfig& config = SuperStructureConfig(),
        ControlOutputMode outputMode = ControlOutputMode::Position)
        : config_(config),
          yawController_(config.yawPid),
          pitchController_(config.pitchPid),
          state_(State::zeroed),
          outputMode_(outputMode),
          targetLostAt_(0),
          feedbackValid_(false),
          incrementalBiasPending_(false)
    {
        output_.mode = outputMode_;
        output_.axis = axisData(config_.zeroYaw, config_.zeroPitch);
    }

    void updateBoxes(const BoxContainer& boxes)
    {
        boxes_ = boxes;
    }

    void setOutputMode(ControlOutputMode outputMode)
    {
        if (outputMode_ != outputMode)
        {
            outputMode_ = outputMode;
            resetControllers();
            incrementalBiasPending_ =
                outputMode_ == ControlOutputMode::Incremental &&
                (state_ == State::aiming || state_ == State::aimed);
        }
    }

    void periodic(const TRoMaC::GimbalFeedback& feedback,
                  int now,
                  double dtSeconds,
                  bool feedbackValid = true)
    {
        //更新反馈数值
        feedbackValid_ = feedbackValid &&
            std::isfinite(feedback.yaw_angle) && std::isfinite(feedback.pitch_angle);
        if (feedbackValid_)
        {
            feedbackData_ = axisData(feedback.yaw_angle, feedback.pitch_angle);
        }
        //获取目标像素点
        const bool targetValid = boxes_.size() == 1;
        const cv::Point2d target = targetValid ? boxes_.getCenter(0) : cv::Point2d();
        updateTargetErrors(targetValid, target);

        switch (state_)
        {
        case State::aiming: //定义：有目标且未对准
            if (!targetValid)
            {
                state_ = State::lost;
                targetLostAt_ = now;
                resetControllers();
                incrementalBiasPending_ = false;
            }
            else if (feedbackValid_ && isWithinAimTolerance(target))
            {
                state_ = State::aimed;
            }
            break;

        case State::aimed: //定义：有目标且已对准
            if (!targetValid)
            {
                state_ = State::lost;
                targetLostAt_ = now;
                incrementalBiasPending_ = false;
            }
            else if (feedbackValid_ && !isWithinAimTolerance(target))
            {
                state_ = State::aiming;
                resetControllers();
            }
            break;

        case State::zeroed: //定义：无目标且已回正
            if (targetValid)
            {
                enterAiming();
            }
            break;

        case State::zeroing:   //定义：无目标且未回正
            if (targetValid)
            {
                enterAiming();
            }
            else if (feedbackValid_ &&
                     std::fabs(feedbackData_.yaw - config_.zeroYaw) < 0.5 &&
                     std::fabs(feedbackData_.pitch - config_.zeroPitch) < 0.5)
            {
                state_ = State::zeroed;
                resetControllers();
            }
            break;

        case State::lost://定义：无目标，等待一段事件后回正
            if (targetValid)
            {
                enterAiming();
            }
            else if (now - targetLostAt_ > config_.stayTimeMs)
            {
                state_ = State::zeroing;
                resetControllers();
            }
            break;
        }

        output_.fire = false;
        output_.mode = outputMode_;
        if (!feedbackValid_)
        {
            output_.axis = axisData();
            resetControllers();
            return;
        }

        if ((state_ == State::aiming || state_ == State::aimed) && targetValid)
        {
            output_.axis = getTrackingOutput(dtSeconds);
        }
        else if (state_ == State::zeroing)
        {
            if (outputMode_ == ControlOutputMode::Position)
            {
                output_.axis = axisData(config_.zeroYaw, config_.zeroPitch);
            }
            else
            {
                output_.axis = axisData(
                    yawController_.update(config_.zeroYaw - feedbackData_.yaw,
                                          dtSeconds, PIDMode::Incremental),
                    pitchController_.update(config_.zeroPitch - feedbackData_.pitch,
                                            dtSeconds, PIDMode::Incremental));
            }
        }
        else if (state_ == State::zeroed &&
                 outputMode_ == ControlOutputMode::Position)
        {
            output_.axis = axisData(config_.zeroYaw, config_.zeroPitch);
        }
        else if (outputMode_ == ControlOutputMode::Position)
        {
            output_.axis = feedbackData_;
        }
        else
        {
            output_.axis = axisData();
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
        return state_ == State::aimed && boxes_.size() == 1 && feedbackValid_;
    }

private:
    SuperStructureConfig config_;
    PIDController yawController_;
    PIDController pitchController_;
    State state_;
    ControlOutputMode outputMode_;
    OutputData output_;
    axisData feedbackData_;
    int targetLostAt_;
    bool feedbackValid_;
    bool incrementalBiasPending_;
    BoxContainer boxes_;

    void resetControllers()
    {
        yawController_.reset();
        pitchController_.reset();
    }

    void enterAiming()
    {
        state_ = State::aiming;
        resetControllers();
        incrementalBiasPending_ = outputMode_ == ControlOutputMode::Incremental;
    }

    void updateTargetErrors(bool targetValid, const cv::Point2d& target)
    {
        if (!targetValid)
        {
            output_.targetOffsetPx = axisData();
            output_.targetAngleError = axisData();
            return;
        }

        output_.targetOffsetPx = axisData(
            target.x - config_.aimU,
            target.y - config_.aimV);
        output_.targetAngleError = axisData(
            pixelToAngle(output_.targetOffsetPx.yaw, config_.focalLengthPx) *
                config_.yawSign,
            -pixelToAngle(output_.targetOffsetPx.pitch, config_.focalLengthPx) *
                config_.pitchSign);
    }

    double yawTargetError() const
    {
        return -output_.targetAngleError.yaw;
    }

    double pitchTargetError() const
    {
        return -output_.targetAngleError.pitch;
    }

    bool isWithinAimTolerance(const cv::Point2d& target) const
    {
        return std::fabs(target.x - config_.aimU) <= config_.aimTolerancePx &&
               std::fabs(target.y - config_.aimV) <= config_.aimTolerancePx;
    }

    axisData getTrackingOutput(double dtSeconds)
    {
        const PIDMode pidMode = outputMode_ == ControlOutputMode::Position
            ? PIDMode::Position
            : PIDMode::Incremental;
        const double yawPixelError = output_.targetOffsetPx.yaw;
        const double pitchPixelError = output_.targetOffsetPx.pitch;
        const bool integralEnabled =
            std::hypot(yawPixelError, pitchPixelError) <=
            config_.integralEnableOffsetPx;
        double yawCorrection = 0.0;
        if (std::fabs(yawPixelError) <= config_.deadbandPx)
        {
            yawController_.reset();
        }
        else
        {
            yawCorrection = yawController_.update(
                yawTargetError(), dtSeconds, pidMode, integralEnabled);
        }

        double pitchCorrection = 0.0;
        if (std::fabs(pitchPixelError) <= config_.deadbandPx)
        {
            pitchController_.reset();
        }
        else
        {
            pitchCorrection = pitchController_.update(
                pitchTargetError(), dtSeconds, pidMode, integralEnabled);
        }

        if (outputMode_ == ControlOutputMode::Position)
        {
            return axisData(feedbackData_.yaw + yawCorrection + config_.yawBias,
                            feedbackData_.pitch + pitchCorrection + config_.pitchBias);
        }

        if (incrementalBiasPending_)
        {
            // Incremental commands accumulate at the actuator, so apply the fixed offset once.
            yawCorrection += config_.yawBias;
            pitchCorrection += config_.pitchBias;
            incrementalBiasPending_ = false;
        }
        return axisData(yawCorrection, pitchCorrection);
    }

};

#endif // SUPER_STRUCTURE_H_
