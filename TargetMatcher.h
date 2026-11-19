#ifndef TARGET_MATCHER_H
#define TARGET_MATCHER_H

#include "Constants.h"
#include "MathUtils.h"

#include <algorithm>
#include <array>
#include <limits>
#include <optional>
#include <utility>
#include <vector>

struct TargetMarker
{
    frc::Translation3d position;
    bool active = false;
};

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
            initialized_ = false;
            return;
        }
        initialized_ = true;
        for (std::size_t i = 0; i < targets_.size(); ++i)
        {
            targets_[i].rotation = frc::Rotation2d{
                rotation->Degrees() + units::degree_t{static_cast<double>(i) * 72.0}};
        }
    }

    void updateCenter(const frc::Translation3d& modelCenter)
    {
        center_ = modelCenter;
    }

    std::optional<frc::Translation3d> select(
        const std::vector<frc::Translation3d>& positions,
        int now = 0, double omega = 0.0)
    {
        currentIndex_.reset();
        advanceRotations(now, omega);
        updatePendingHit(now);
        if (!initialized_ || positions.empty())
        {
            return std::nullopt;
        }

        if (allActive())
        {
            reset();
        }

        std::optional<std::size_t> anchorIndex;
        const frc::Translation3d* anchorPosition = nullptr;
        double anchorDelta = std::numeric_limits<double>::max();
        for (const auto& position : positions)
        {
            const auto candidate = match(position);
            if (!candidate)
            {
                continue;
            }
            const auto [index, delta] = *candidate;
            if (delta < anchorDelta)
            {
                anchorDelta = delta;
                anchorIndex = index;
                anchorPosition = &position;
            }
        }

        if (!anchorIndex)
        {
            return std::nullopt;
        }
        syncAngles(*anchorPosition, *anchorIndex);

        std::optional<std::size_t> bestIndex;
        const frc::Translation3d* bestPosition = nullptr;
        double bestDelta = std::numeric_limits<double>::max();
        for (const auto& position : positions)
        {
            const auto candidate = match(position);
            if (!candidate)
            {
                continue;
            }
            const auto [index, delta] = *candidate;
            if ((recordHitTargets_ && targets_[index].active) || delta >= bestDelta)
            {
                continue;
            }
            bestDelta = delta;
            bestIndex = index;
            bestPosition = &position;
        }

        if (!bestIndex)
        {
            return std::nullopt;
        }
        currentIndex_ = bestIndex;
        return *bestPosition;
    }

    void markFiredAt(int now)
    {
        if (!recordHitTargets_ || !currentIndex_)
        {
            return;
        }
        pendingHitIndex_ = currentIndex_;
        pendingHitAt_ = now;
    }

    void setRecordHitTargets(bool enabled)
    {
        recordHitTargets_ = enabled;
        if (!enabled)
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
        pendingHitIndex_.reset();
        pendingHitAt_ = 0;
    }

    std::vector<TargetMarker> markers(double radius) const
    {
        std::vector<TargetMarker> result;
        if (!initialized_)
        {
            return result;
        }

        result.reserve(targets_.size());
        for (const auto& target : targets_)
        {
            const frc::Translation2d offset{
                units::meter_t{radius * target.rotation.Cos()},
                units::meter_t{radius * target.rotation.Sin()}};
            result.push_back({ToXYZ(offset, center_), target.active});
        }
        return result;
    }

    bool initialized() const
    {
        return initialized_;
    }

private:
    struct Target
    {
        frc::Rotation2d rotation;
        bool active = false;
    };

    void advanceRotations(int now, double omega )
    {
        if (!initialized_ || now <= lastUpdateAt_ || !std::isfinite(omega))
        {
            lastUpdateAt_ = std::max(lastUpdateAt_, now);
            return;
        }

        const units::radian_t delta{(anti_ ? 1 : -1) * omega * (now - lastUpdateAt_) / 1000.0};
        for (auto& target : targets_)
            target.rotation = target.rotation.RotateBy(frc::Rotation2d{delta});
        lastUpdateAt_ = now;
    }

    void updatePendingHit(int now)
    {
        if (!pendingHitIndex_ || !recordHitTargets_ ||
            now - pendingHitAt_ < kTARGET_HIT_CONFIRM_MS)
        {
            return;
        }

        targets_[*pendingHitIndex_].active = true;
        pendingHitIndex_.reset();
        pendingHitAt_ = 0;
        if (allActive())
        {
            reset();
        }
    }

    void syncAngles(const frc::Translation3d& observed, std::size_t observedIndex)
    {
        const auto rotation = ToYZ(observed, center_).Angle();
        if (!rotation)
        {
            return;
        }

        for (std::size_t i = 0; i < targets_.size(); ++i)
        {
            const double slotOffset =
                static_cast<double>(static_cast<int>(i) -
                                     static_cast<int>(observedIndex)) * 72.0;
            targets_[i].rotation = frc::Rotation2d{
                rotation->Degrees() + units::degree_t{slotOffset}};
        }
    }

    std::optional<std::pair<std::size_t, double>> match(
        const frc::Translation3d& position) const
    {
        const auto rotation = ToYZ(position, center_).Angle();
        if (!rotation)
        {
            return std::nullopt;
        }
        std::optional<std::pair<std::size_t, double>> bestMatch;
        double minDelta = std::numeric_limits<double>::max();
        for (std::size_t i = 0; i < targets_.size(); ++i)
        {
            const double delta =
                absRotation(*rotation - targets_[i].rotation).Degrees().value();
            if (delta <= kTARGET_MATCH_TOLERANCE.value() && delta < minDelta)
            {
                minDelta = delta;
                bestMatch = std::pair{i, delta};
            }
        }
        return bestMatch;
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
    std::optional<std::size_t> pendingHitIndex_;
    int pendingHitAt_ = 0;
    bool initialized_ = false;
    bool recordHitTargets_ = true;
    bool anti_ = true;
};

#endif // TARGET_MATCHER_H