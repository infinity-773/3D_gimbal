#ifndef TARGET_TRACER_H
#define TARGET_TRACER_H

#include <vector>
#include <Constants.h>

enum class targetNum{
    first,
    second,
    third,
    fourth,
    fifth,
};

struct Target {
    targetNum num;
    frc::Translation2d position; 
};

class TargetTracer {
public:
    TargetTracer();
    void chooseTarget(targetNum target);

private:
    std::vector<targetNum> targets;
};



#endif // TARGET_TRACER_H