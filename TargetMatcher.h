#ifndef TARGET_MATCHER_H
#define TARGET_MATCHER_H

#include "Constants.h"
#include "MathUtils.h"

#include <array>
#include <limits>
#include <optional>
#include <vector>

class TargetMatcher
{
public:
    void initialize(const frc::Translation3d& modelCenter,
                    const frc::Translation3d& firstPosition,
                    int now)
    {
        center_ = modelCenter;
        lastUpdateAt_ = now;
        currentIndex_.reset();
        reset();

        const auto rotation = ToYZ(firstPosition, center_).Angle();
        if (!rotation)
        {
            return;
        }
        for (std::size_t i = 0; i < targets_.size(); ++i)
        {
            targets_[i].rotation = frc::Rotation2d{
                rotation->Degrees() + units::degree_t{static_cast<double>(i) * 72.0}};
        }
    }

    std::optional<frc::Translation3d> select(
        const std::vector<frc::Translation3d>& positions,
        int now,
        double omega)
    {
        currentIndex_.reset();
        updateAngles(now, omega);
        if (positions.empty())
        {
            return std::nullopt;
        }
        if (allActive())
        {
            reset();
        }

        const bool singleTarget = positions.size() == 1;
        for (const auto& position : positions)
        {
            const auto index = match(position);
            if (!index || (!singleTarget && targets_[*index].active))
            {
                continue;
            }
            currentIndex_ = index;
            targets_[*index].active = false;
            return position;
        }
        return std::nullopt;
    }

    void markFired()
    {
        if (!currentIndex_)
        {
            return;
        }
        targets_[*currentIndex_].active = true;
        if (allActive())
        {
            reset();
        }
    }

    void reset()
    {
        for (auto& target : targets_)
        {
            target.active = false;
        }
    }

private:
    struct Target
    {
        frc::Rotation2d rotation;
        bool active = false;
    };

    void updateAngles(int now, double omega)
    {
        const double dt = static_cast<double>(now - lastUpdateAt_) * 0.001;
        const frc::Rotation2d rotation{units::radian_t{dt * omega}};
        for (auto& target : targets_)
        {
            target.rotation = target.rotation + rotation;
        }
        lastUpdateAt_ = now;
    }

    std::optional<std::size_t> match(const frc::Translation3d& position) const
    {
        const auto rotation = ToYZ(position, center_).Angle();
        if (!rotation)
        {
            return std::nullopt;
        }
        std::optional<std::size_t> bestIndex;
        auto minDelta = units::degree_t{std::numeric_limits<double>::max()};
        for (std::size_t i = 0; i < targets_.size(); ++i)
        {
            const auto delta = absRotation(*rotation - targets_[i].rotation).Degrees();
            if (delta <= kTARGET_MATCH_TOLERANCE && delta < minDelta)
            {
                minDelta = delta;
                bestIndex = i;
            }
        }
        return bestIndex;
    }

    bool allActive() const
    {
        for (const auto& target : targets_)
        {
            if (!target.active)
            {
                return false;
            }
        }
        return true;
    }

    std::array<Target, 5> targets_;
    frc::Translation3d center_;
    int lastUpdateAt_ = 0;
    std::optional<std::size_t> currentIndex_;
};

#endif // TARGET_MATCHER_H