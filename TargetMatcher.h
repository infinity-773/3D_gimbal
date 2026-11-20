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
#include <Debouncer.h>

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

        const auto rotation = posToangle(firstPosition);
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


    std::optional<frc::Translation3d> selectAndUpdate(const std::vector<frc::Translation3d>& positions, int now = 0, double omega = 0.0){
        static std::array<int, 2> lastIndices{-1, -1};
        advanceRotations(now, omega);

        if (positions.empty() || positions.size() > 2)
        {
            lastIndices = {-1, -1};
            return std::nullopt;
        }
        syncAngles(positions);

        if(positions.size() == 1){
            lastIndices = {-1, -1};
            reset();
            auto matchResult = match(positions.front());
            if (!matchResult) {
                return std::nullopt;
            }else{
                last_fired_id = *matchResult;
                return positions.front();
            }
        }

        std::array<int, 2> indices{-1, -1};
        for(int i = 0; i < positions.size(); ++i){
            auto matchResult = match(positions[i]);
            if (!matchResult) {
                lastIndices = {-1, -1};
                return std::nullopt;
            }
            indices[i] = *matchResult;
        }
        //
        if(indices[0] == last_fired_id)
        {   if(targets_[indices[0]].active == false){ return positions[0]; }
                targets_[last_fired_id].setState(true);
                last_fired_id = indices[1];
                lastIndices = indices;
                return positions[1];
        }else if(indices[1] == last_fired_id)
        {
            if(targets_[indices[1]].active == false){ return positions[1]; }
            targets_[last_fired_id].setState(true);
            last_fired_id = indices[0];
            lastIndices = indices;
            return positions[0];
        }

        if(targets_[indices[0]].active == targets_[indices[1]].active){
            //上一帧没有两个
            if(lastIndices[0]==-1 or lastIndices[1]==-1)
            {
                lastIndices = indices;
                return std::nullopt;
            }// 还没有变化
            if(lastIndices[0] == indices[0] && lastIndices[1] == indices[1]){
                return std::nullopt;
            }
            //id0先变化
            if(lastIndices[0] != indices[0] && lastIndices[1] == indices[1]){
                targets_[indices[0]].setState(true);
                targets_[indices[1]].setState(false);
                last_fired_id = indices[0];
                lastIndices = indices;
                return positions[0];
            //id1先变化
            }else if(lastIndices[0] == indices[0] && lastIndices[1] != indices[1]){
                targets_[indices[0]].setState(false);
                targets_[indices[1]].setState(true);
                last_fired_id = indices[1];
                lastIndices = indices;
                return positions[1];
            }
        }


        return std::nullopt;

    }


    void markFired()
    {
        if (!recordHitTargets_ || last_fired_id < 0 ||
            static_cast<std::size_t>(last_fired_id) >= targets_.size())
        {
            return;
        }
        targets_[static_cast<std::size_t>(last_fired_id)].hasFired = true;
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
        bool hasFired = false;
        Debouncer activeDebouncer = Debouncer(kTARGET_HIT_CONFIRM_FRAMES , DebounceEdge::Rising);

        void setState(bool isactive)
        {
            if (!hasFired)
            {
                return;
            }
            const bool wasActive = active;
             active = activeDebouncer.update(isactive);
             if (!wasActive && active)
             {
                hasFired = false;
             }
        }
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
    int last_fired_id = -1;
};

#endif // TARGET_MATCHER_H