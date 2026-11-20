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
    bool lastFired = false;
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

        const auto rotation = posToangle(firstPosition);
        if (!rotation)
        {
            initialized_ = false;
            return;
        }
        initialized_ = true;
        lastFiredIndex_.reset();
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


    std::optional<frc::Translation3d> selectAndUpdate(
        const std::vector<frc::Translation3d>& positions,
        int now = 0,
        double omega = 0.0)
    {
        advanceRotations(now, omega);
        if (positions.empty() || positions.size() > 2)
        {
            updatePendingHit(now, {-1, -1});
            observedIndices_ = {-1, -1};
            return std::nullopt;
        }

        syncAngles(positions);
        std::array<int, 2> indices{-1, -1};
        for (std::size_t i = 0; i < positions.size(); ++i)
        {
            const auto matchResult = match(positions[i]);
            if (!matchResult)
            {
                updatePendingHit(now, {-1, -1});
                observedIndices_ = {-1, -1};
                    return std::nullopt;
            }
            indices[i] = static_cast<int>(*matchResult);
        }

        if (indices[0] == indices[1] && indices[1] >= 0)
        {
            observedIndices_ = {-1, -1};
            return std::nullopt;
        }

        if (updatePendingHit(now, indices))
            return selectedPosition(indices, positions, last_fired_id);

        int selectedIndex = -1;
        for (int index : indices)
        {
            if (index < 0 || targets_[index].active)
                continue;
            if (selectedIndex < 0 || index == last_fired_id)
                selectedIndex = index;
        }

        if (selectedIndex < 0 && last_fired_id >= 0 &&
            static_cast<std::size_t>(last_fired_id) < targets_.size() &&
            !targets_[last_fired_id].active && containsIndex(indices, last_fired_id))
        {
            selectedIndex = last_fired_id;
        }

        if (selectedIndex >= 0)
            last_fired_id = selectedIndex;
        observedIndices_ = indices;
        return selectedPosition(indices, positions, selectedIndex);
    }

    // 兼容名称；现在与默认逻辑完全相同。
    std::optional<frc::Translation3d> selectAndUpdateByRefresh(
        const std::vector<frc::Translation3d>& positions,
        int now = 0,
        double omega = 0.0)
    {
        return selectAndUpdate(positions, now, omega);
    }

    void markFired()
    {
        markFired(lastUpdateAt_);
    }

    void markFired(int now)
    {
        if (!recordHitTargets_ || last_fired_id < 0 ||
            static_cast<std::size_t>(last_fired_id) >= targets_.size())
        {
            return;
        }

        const auto index = static_cast<std::size_t>(last_fired_id);
        pendingHitIndex_ = index;
        pendingHitAt_ = now;
        pendingPreviousIndices_ = observedIndices_;
        lastFiredIndex_ = index;
    }

    void markFiredByRefresh(int now)
    {
        markFired(now);
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
            target.active = false;
        observedIndices_ = {-1, -1};
        pendingPreviousIndices_ = {-1, -1};
        pendingHitIndex_.reset();
        pendingHitAt_ = 0;
        last_fired_id = -1;
        lastFiredIndex_.reset();
    }

    std::vector<TargetMarker> markers(double radius) const
    {
        std::vector<TargetMarker> result;
        if (!initialized_)
        {
            return result;
        }

        result.reserve(targets_.size());
        for (std::size_t i = 0; i < targets_.size(); ++i)
        {
            const auto& target = targets_[i];
            const frc::Translation2d offset{
                units::meter_t{radius * target.rotation.Cos()},
                units::meter_t{radius * target.rotation.Sin()}};
            result.push_back({ToXYZ(offset, center_), target.active,
                              lastFiredIndex_ == i});
        }
        return result;
    }

    bool initialized() const
    {
        return initialized_;
    }

private:

    static bool containsIndex(const std::array<int, 2>& indices, int index)
    {
        return indices[0] == index || indices[1] == index;
    }

    std::optional<frc::Translation3d> selectedPosition(
        const std::array<int, 2>& indices,
        const std::vector<frc::Translation3d>& positions,
        int selectedIndex) const
    {
        for (std::size_t i = 0; i < positions.size(); ++i)
        {
            if (indices[i] == selectedIndex)
                return positions[i];
        }
        return std::nullopt;
    }

    bool updatePendingHit(
        int now,
        const std::array<int, 2>& observed)
    {
        if (!pendingHitIndex_)
            return false;

        const std::size_t firedIndex = *pendingHitIndex_;
        const int elapsedMs = now - pendingHitAt_;
        int refreshedInactive = -1;
        if (elapsedMs <= kACTIVE_CONFIRM_MS)
        {
            for (int index : observed)
            {
                if (index < 0 || static_cast<std::size_t>(index) == firedIndex ||
                    containsIndex(pendingPreviousIndices_, index))
                {
                    continue;
                }
                if (!targets_[index].active || activeCount() == 4)
                {
                    refreshedInactive = index;
                    break;
                }
            }
        }

        if (refreshedInactive >= 0)
        {
            targets_[firedIndex].active = true;
            pendingHitIndex_.reset();
            observedIndices_ = observed;
            if (allActive())
            {
                resetRound();
                return true;
            }
            targets_[refreshedInactive].active = false;
            last_fired_id = refreshedInactive;
            return true;
        }

        if (elapsedMs >= kACTIVE_CONFIRM_MS)
        {
            // 这次发射没有引起新的 inactive 目标，判定未命中。
            pendingHitIndex_.reset();
            targets_[firedIndex].active = false;
        }
        return false;
    }

    void resetRound()
    {
        for (auto& target : targets_)
            target.active = false;
        observedIndices_ = {-1, -1};
        pendingPreviousIndices_ = {-1, -1};
        pendingHitIndex_.reset();
        pendingHitAt_ = 0;
        last_fired_id = -1;
    }

    struct Target
    {
        frc::Rotation2d rotation;
        bool active = false;
    };

    std::optional<frc::Rotation2d> posToangle(const frc::Translation3d& position) const
    {
        return ToYZ(position, center_).Angle();
    }

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


    bool syncAngles(const std::vector<frc::Translation3d>& positions)
    {
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
            const auto rotation = posToangle(position);
            if (!rotation)
            {
                continue;
            }
            const auto index = *candidate;
            const double delta =
                absRotation(*rotation - targets_[index].rotation).Degrees().value();
            if (delta < anchorDelta)
            {
                anchorDelta = delta;
                anchorIndex = index;
                anchorPosition = &position;
            }
        }

        if (!anchorIndex)
        {
            return false;
        }

        const auto rotation = posToangle(*anchorPosition);
        if (!rotation)
        {
            return false;
        }

        for (std::size_t i = 0; i < targets_.size(); ++i)
        {
            const double slotOffset =
                static_cast<double>(static_cast<int>(i) -
                                     static_cast<int>(*anchorIndex)) * 72.0;
            targets_[i].rotation = frc::Rotation2d{
                rotation->Degrees() + units::degree_t{slotOffset}};
        }
        return true;
    }

    std::optional<std::size_t> match(
        const frc::Translation3d& position) const
    {
        const auto rotation = posToangle(position);
        if (!rotation)
        {
            return std::nullopt;
        }
        std::optional<std::size_t> bestMatch;
        double minDelta = std::numeric_limits<double>::max();
        for (std::size_t i = 0; i < targets_.size(); ++i)
        {
            const double delta =
                absRotation(*rotation - targets_[i].rotation).Degrees().value();
            if (delta <= kTARGET_MATCH_TOLERANCE.value() && delta < minDelta)
            {
                minDelta = delta;
                bestMatch = i;
            }
        }
        return bestMatch;
    }

    int activeCount() const
    {
        return static_cast<int>(std::count_if(
            targets_.begin(), targets_.end(),
            [](const Target& target) { return target.active; }));
    }

    bool allActive() const
    {
        return activeCount() == static_cast<int>(targets_.size());
    }

    std::array<Target, 5> targets_;
    frc::Translation3d center_;
    int lastUpdateAt_ = 0;
    std::optional<std::size_t> currentIndex_;
    std::optional<std::size_t> pendingHitIndex_;
    int pendingHitAt_ = 0;
    std::array<int, 2> pendingPreviousIndices_{-1, -1};
    std::array<int, 2> observedIndices_{-1, -1};
    std::optional<std::size_t> lastFiredIndex_;
    bool initialized_ = false;
    bool recordHitTargets_ = true;
    bool anti_ = true;
    int last_fired_id = -1;
};

#endif // TARGET_MATCHER_H