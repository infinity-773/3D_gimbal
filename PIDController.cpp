#include "PIDController.h"

#include "MathUtils.h"

#include <cmath>
#include <stdexcept>

PIDController::PIDController(const PIDConfig& config)
    : config_(config),
      integral_(0.0),
      previousError_(0.0),
      previousPreviousError_(0.0),
      previousOutput_(0.0),
      incrementalIntegral_(0.0),
      hasPreviousError_(false),
      hasPreviousPreviousError_(false)
{
    if (!std::isfinite(config_.kp) || !std::isfinite(config_.ki) ||
        !std::isfinite(config_.kd) || !std::isfinite(config_.integralMin) ||
        !std::isfinite(config_.integralMax) || !std::isfinite(config_.outputMin) ||
        !std::isfinite(config_.outputMax) || config_.integralMin > config_.integralMax ||
        config_.outputMin > config_.outputMax)
    {
        throw std::invalid_argument("Invalid PID configuration");
    }
}

double PIDController::update(double error,
                             double dtSeconds,
                             PIDMode mode,
                             bool integralEnabled)
{
    if (!std::isfinite(error) || !std::isfinite(dtSeconds) || dtSeconds <= 0.0)
    {
        throw std::invalid_argument("PID update requires a finite error and positive dt");
    }

    return mode == PIDMode::Position
        ? updatePosition(error, dtSeconds, integralEnabled)
        : updateIncremental(error, dtSeconds, integralEnabled);
}

void PIDController::reset()
{
    integral_ = 0.0;
    previousError_ = 0.0;
    previousPreviousError_ = 0.0;
    previousOutput_ = 0.0;
    incrementalIntegral_ = 0.0;
    hasPreviousError_ = false;
    hasPreviousPreviousError_ = false;
}

double PIDController::updatePosition(double error,
                                     double dtSeconds,
                                     bool integralEnabled)
{
    const double derivative = hasPreviousError_
        ? (error - previousError_) / dtSeconds
        : 0.0;
    const double candidateIntegral = integralEnabled
        ? clampValue(integral_ + error * dtSeconds,
                     config_.integralMin, config_.integralMax)
        : 0.0;
    const double candidateOutput =
        config_.kp * error + config_.ki * candidateIntegral + config_.kd * derivative;

    if (!integralEnabled)
    {
        integral_ = 0.0;
    }
    else if ((candidateOutput >= config_.outputMin && candidateOutput <= config_.outputMax) ||
        (candidateOutput > config_.outputMax && error < 0.0) ||
        (candidateOutput < config_.outputMin && error > 0.0))
    {
        integral_ = candidateIntegral;
    }

    const double output = clampValue(
        config_.kp * error + config_.ki * integral_ + config_.kd * derivative,
        config_.outputMin, config_.outputMax);
    previousError_ = error;
    hasPreviousError_ = true;
    return output;
}

double PIDController::updateIncremental(double error,
                                        double dtSeconds,
                                        bool integralEnabled)
{
    if (!integralEnabled)
    {
        previousOutput_ = clampValue(
            previousOutput_ - incrementalIntegral_,
            config_.outputMin, config_.outputMax);
        incrementalIntegral_ = 0.0;
    }

    const double previousError = hasPreviousError_ ? previousError_ : error;
    const double previousPreviousError =
        hasPreviousPreviousError_ ? previousPreviousError_ : previousError;
    const double proportionalDelta = config_.kp * (error - previousError);
    const double integralDelta = integralEnabled ? config_.ki * error * dtSeconds : 0.0;
    const double derivativeDelta =
        config_.kd * (error - 2.0 * previousError + previousPreviousError) / dtSeconds;
    const double outputWithoutIntegral =
        previousOutput_ + proportionalDelta + derivativeDelta;
    const double candidateOutput = outputWithoutIntegral + integralDelta;

    const bool acceptIntegral = integralEnabled &&
        ((candidateOutput >= config_.outputMin && candidateOutput <= config_.outputMax) ||
         (candidateOutput > config_.outputMax && integralDelta < 0.0) ||
         (candidateOutput < config_.outputMin && integralDelta > 0.0));
    const double appliedIntegralDelta = acceptIntegral ? integralDelta : 0.0;
    const double deltaOutput =
        proportionalDelta + appliedIntegralDelta + derivativeDelta;

    const double limitedOutput = clampValue(
        previousOutput_ + deltaOutput, config_.outputMin, config_.outputMax);
    const double appliedDelta = limitedOutput - previousOutput_;
    previousOutput_ = limitedOutput;
    incrementalIntegral_ += appliedIntegralDelta;
    previousPreviousError_ = previousError;
    hasPreviousPreviousError_ = hasPreviousError_;
    previousError_ = error;
    hasPreviousError_ = true;
    return appliedDelta;
}
