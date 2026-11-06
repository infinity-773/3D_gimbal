#ifndef PID_CONTROLLER_H_
#define PID_CONTROLLER_H_

struct PIDConfig
{
    double kp;
    double ki;
    double kd;
    double integralMin;
    double integralMax;
    double outputMin;
    double outputMax;

    PIDConfig(double proportionalGain = 0.1,
              double integralGain = 0.3,
              double derivativeGain = 0.0,
              double minimumIntegral = -3.0,
              double maximumIntegral = 3.0,
              double minimumOutput = -4.0,
              double maximumOutput = 4.0)
        : kp(proportionalGain),
          ki(integralGain),
          kd(derivativeGain),
          integralMin(minimumIntegral),
          integralMax(maximumIntegral),
          outputMin(minimumOutput),
          outputMax(maximumOutput)
    {}
};

enum class PIDMode
{
    Position,
    Incremental
};

class PIDController
{
public:
    explicit PIDController(const PIDConfig& config = PIDConfig());

    double update(double error, double dtSeconds, PIDMode mode,
                  bool integralEnabled = true);
    void reset();

private:
    PIDConfig config_;
    double integral_;
    double previousError_;
    double previousPreviousError_;
    double previousOutput_;
    double incrementalIntegral_;
    bool hasPreviousError_;
    bool hasPreviousPreviousError_;

    double updatePosition(double error, double dtSeconds, bool integralEnabled);
    double updateIncremental(double error, double dtSeconds, bool integralEnabled);
};

#endif // PID_CONTROLLER_H_
