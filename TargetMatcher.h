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


    std::optional<frc::Translation3d> selectAndUpdate(const std::vector<frc::Translation3d>& positions, int now = 0, double omega = 0.0){
        advanceRotations(now, omega);
        confirmFiredTargets();

        if (positions.empty() || positions.size() > 2)
        {
            lastIndices = {-1, -1};
            return std::nullopt;
        }
        syncAngles(positions);

        if(positions.size() == 1){
            lastIndices = {-1, -1};
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

        if(indices[0] == last_fired_id)
        {
            if(targets_[indices[0]].active == false){
                lastIndices = indices;
                return positions[0];
            }
            targets_[last_fired_id].setState(true);
                last_fired_id = indices[1];
                lastIndices = indices;
                return positions[1];
        }else if(indices[1] == last_fired_id)
        {
            if(targets_[indices[1]].active == false){
                lastIndices = indices;
                return positions[1];
            }
            targets_[last_fired_id].setState(true);
            last_fired_id = indices[0];
            lastIndices = indices;
            return positions[0];
        }else if(targets_[indices[0]].active == false && targets_[indices[1]].active == true)
        {
            lastIndices = indices;
            last_fired_id = indices[0];
            return positions[0];
        }else if(targets_[indices[0]].active == true && targets_[indices[1]].active == false)
        {
            lastIndices = indices;
            last_fired_id = indices[1];
            return positions[1];
        }else

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
                targets_[lastIndices[0]].active = true;
                targets_[indices[0]].active = false;
                last_fired_id = indices[0];
                lastIndices = indices;
                return positions[0];
            //id1先变化
            }else if(lastIndices[0] == indices[0] && lastIndices[1] != indices[1]){
                targets_[lastIndices[1]].active = true;
                targets_[indices[1]].active = false;
                last_fired_id = indices[1];
                lastIndices = indices;
                return positions[1];
            }
        }


        return std::nullopt;

    }

    // 使用“其他目标重新出现”的事件确认命中，不改变 selectAndUpdate 的原逻辑。
    std::optional<frc::Translation3d> selectAndUpdateByRefresh(
        const std::vector<frc::Translation3d>& positions,
        int now = 0,
        double omega = 0.0)
    {
        const auto selected = selectAndUpdate(positions, now, omega);
        std::array<int, 2> observed{-1, -1};
        for (std::size_t i = 0; i < positions.size() && i < observed.size(); ++i)
        {
            const auto index = match(positions[i]);
            if (index)
                observed[i] = static_cast<int>(*index);
        }
        updateRefreshConfirmation(now, observed);
        return selected;
    }

    void markFiredByRefresh(int now)
    {
        if (!recordHitTargets_ || last_fired_id < 0 ||
            static_cast<std::size_t>(last_fired_id) >= targets_.size())
        {
            return;
        }
        const auto index = static_cast<std::size_t>(last_fired_id);
        targets_[index].hasFired = true;
        targets_[index].activeDebouncer.reset(false);
        pendingRefreshHitIndex_ = index;
        pendingRefreshHitAt_ = now;
        pendingRefreshPrevious_ = refreshObserved_;
        lastFiredIndex_ = index;
    }


    void markFired()
    {
        if (!recordHitTargets_ || last_fired_id < 0 ||
            static_cast<std::size_t>(last_fired_id) >= targets_.size())
        {
            return;
        }
        const auto index = static_cast<std::size_t>(last_fired_id);
        targets_[index].hasFired = true;
        targets_[index].activeDebouncer.reset(false);
        lastFiredIndex_ = index;
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
            target.hasFired = false;
            target.activeDebouncer.reset(false);
        }
        lastIndices = {-1, -1};
        refreshObserved_ = {-1, -1};
        pendingRefreshPrevious_ = {-1, -1};
        pendingRefreshHitIndex_.reset();
        pendingRefreshHitAt_ = 0;
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

    void updateRefreshConfirmation(int now, const std::array<int, 2>& observed)
    {
        if (pendingRefreshHitIndex_)
        {
            const std::size_t firedIndex = *pendingRefreshHitIndex_;
            const int elapsedMs = now - pendingRefreshHitAt_;

            if (elapsedMs <= kACTIVE_CONFIRM_MS)
            {
                for (const int index : observed)
                {
                    if (index < 0 || static_cast<std::size_t>(index) == firedIndex ||
                        containsIndex(pendingRefreshPrevious_, index))
                    {
                        continue;
                    }

                    targets_[firedIndex].active = true;
                    targets_[firedIndex].hasFired = false;
                    targets_[firedIndex].activeDebouncer.reset(true);
                    targets_[static_cast<std::size_t>(index)].active = false;
                    pendingRefreshHitIndex_.reset();
                    if (allActive())
                        resetRound();
                    break;
                }
            }

            if (pendingRefreshHitIndex_ && elapsedMs >= kACTIVE_CONFIRM_MS)
            {
                // 默认仍通过 confirmActive；整个观察窗口内没有其他目标刷新/出现时撤销。
                targets_[firedIndex].active = false;
                targets_[firedIndex].hasFired = false;
                targets_[firedIndex].activeDebouncer.reset(false);
                pendingRefreshHitIndex_.reset();
            }
        }

        refreshObserved_ = observed;
        pendingRefreshPrevious_ = observed;
    }

    void confirmFiredTargets()
    {
        for (auto& target : targets_)
        {
            if (target.hasFired)
                target.setState(true);
        }
        if (allActive())
            resetRound();
    }

    void resetRound()
    {
        for (auto& target : targets_)
        {
            target.active = false;
            target.hasFired = false;
            target.activeDebouncer.reset(false);
        }
        lastIndices = {-1, -1};
        refreshObserved_ = {-1, -1};
        pendingRefreshPrevious_ = {-1, -1};
        pendingRefreshHitIndex_.reset();
        pendingRefreshHitAt_ = 0;
        last_fired_id = -1;
    }

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
    std::optional<std::size_t> lastFiredIndex_;
    std::optional<std::size_t> pendingRefreshHitIndex_;
    std::array<int, 2> refreshObserved_{-1, -1};
    std::array<int, 2> pendingRefreshPrevious_{-1, -1};
    int pendingRefreshHitAt_ = 0;
    std::array<int, 2> lastIndices{-1, -1};
    bool initialized_ = false;
    bool recordHitTargets_ = true;
    bool anti_ = false;
    int last_fired_id = -1;
};

#endif // TARGET_MATCHER_H