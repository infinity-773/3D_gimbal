#ifndef TARGET_MATCHER_SWITCH_H
#define TARGET_MATCHER_SWITCH_H

#include "Constants.h"
#include "Debouncer.h"
#include "MathUtils.h"
#include "TargetMatcher.h"

#include <algorithm>
#include <array>
#include <limits>
#include <optional>
#include <vector>

// 单目标连续出现多久后视为机械状态已重置。
inline constexpr int kSINGLE_TARGET_RESET_FRAMES = 10;
// 击发后该时间内出现新亮起的未激活目标则确认命中并切换；超时未跳变则允许补发。
inline constexpr int kSWITCH_SHOT_WAIT_MS = 600;
// 同一目标未命中时最多补发到这个发数，之后等目标变化，避免目标切换后弹仓打空。
inline constexpr int kSWITCH_MAX_SHOTS_PER_TARGET = 2;

// 通过观察“目标跳变”确认命中的五槽位匹配器。
// 这里只负责选择目标和记录 active 状态，不计算角速度、提前量或火控。
class TargetMatcherSwitch
{
public:
    void initialize(const frc::Translation3d& modelCenter,
                    const frc::Translation3d& firstPosition,
                    int now)
    {
        center_ = modelCenter;
        lastUpdateAt_ = now;
        reset();

        const auto rotation = posToAngle(firstPosition);
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

    // 圆心拟合持续变化时同步更新画面和角度匹配使用的圆心。
    void updateCenter(const frc::Translation3d& modelCenter)
    {
        center_ = modelCenter;
    }

    // 每帧输入当前可见的 1～2 个目标，返回本帧允许瞄准的目标。
    std::optional<frc::Translation3d> selectAndUpdate(
        const std::vector<frc::Translation3d>& positions,
        int now = 0,
        double omega = 0.0)
    {
        // 五个槽位先随角速度旋转，再用当前观测角度修正预测位置。
        advanceRotations(now, omega);

        // 只有连续看到单目标达到要求帧数，才把它视为新一轮的开始。
        const bool singleStable = singleTargetDebouncer_.update(positions.size() == 1);
        if (!singleStable_ && singleStable)
            resetRoundState();
        singleStable_ = singleStable;

        // 原能量机关规则最多同时有效观察两个目标；异常数量不参与选择。
        if (positions.empty() || positions.size() > 2)
        {
            previousIndices_ = {-1, -1};
            return std::nullopt;
        }
        syncAngles(positions);

        const auto indices = matchPositions(positions);
        if (!indices)
        {
            previousIndices_ = {-1, -1};
            return std::nullopt;
        }

        if (pendingShot_)
        {
            const int elapsedMs = now - pendingShot_->firedAt;

            if (elapsedMs < kSWITCH_SHOT_WAIT_MS)
            {
                const int refreshed = findRefreshedInactive(
                    *indices, pendingShot_->visibleAtFire);
                previousIndices_ = *indices;
                if (refreshed < 0)
                    return std::nullopt;

                targets_[pendingShot_->targetIndex].active = true;
                pendingShot_.reset();
                shotTarget_.reset();
                shotCount_ = 0;
                if (allActive())
                    resetRoundState();
                selectedIndex_ = refreshed;
                previousIndices_ = *indices;
                return positionForIndex(*indices, positions, refreshed);
            }

            // 未跳变视为未命中：不操作，原目标可继续被选中补发。
            pendingShot_.reset();
        }

        // 已击发目标熄灭后重新亮起，视为新目标，重新计数。
        if (shotTarget_ && !containsIndex(*indices, static_cast<int>(*shotTarget_)))
        {
            shotTarget_.reset();
            shotCount_ = 0;
        }

        const int selected = chooseInactiveTarget(*indices);
        previousIndices_ = *indices;
        if (selected < 0)
            return std::nullopt;
        if (shotTarget_ && *shotTarget_ == static_cast<std::size_t>(selected) &&
            shotCount_ >= kSWITCH_MAX_SHOTS_PER_TARGET)
        {
            return std::nullopt;
        }
        selectedIndex_ = static_cast<std::size_t>(selected);
        return positionForIndex(*indices, positions, selected);
    }

    // 火控实际击发后调用，开启 0.6s 的“等待跳变”窗口。
    void markFired(int now)
    {
        if (!recordHitTargets_ || !selectedIndex_ ||
            *selectedIndex_ >= targets_.size())
        {
            return;
        }

        lastFiredIndex_ = selectedIndex_;
        if (shotTarget_ == selectedIndex_)
        {
            ++shotCount_;
        }
        else
        {
            shotTarget_ = selectedIndex_;
            shotCount_ = 1;
        }
        pendingShot_ = PendingShot{
            .targetIndex = *selectedIndex_,
            .firedAt = now,
            .visibleAtFire = previousIndices_};
    }

    void markFired()
    {
        markFired(lastUpdateAt_);
    }

    void setRecordHitTargets(bool enabled)
    {
        recordHitTargets_ = enabled;
        if (!enabled)
            reset();
    }

    void reset()
    {
        resetRoundState();
        lastFiredIndex_.reset();
        singleTargetDebouncer_.reset(false);
        singleStable_ = false;
    }

    std::vector<TargetMarker> markers(double radius) const
    {
        std::vector<TargetMarker> result;
        if (!initialized_)
            return result;

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
    struct Target
    {
        frc::Rotation2d rotation;
        bool active = false;
    };

    struct PendingShot
    {
        std::size_t targetIndex;
        int firedAt;
        std::array<int, 2> visibleAtFire;
    };

    void resetRoundState()
    {
        for (auto& target : targets_)
            target.active = false;
        selectedIndex_.reset();
        pendingShot_.reset();
        shotTarget_.reset();
        shotCount_ = 0;
        previousIndices_ = {-1, -1};
    }

    std::optional<std::array<int, 2>> matchPositions(
        const std::vector<frc::Translation3d>& positions) const
    {
        std::array<int, 2> indices{-1, -1};
        for (std::size_t i = 0; i < positions.size(); ++i)
        {
            const auto matchResult = match(positions[i]);
            if (!matchResult)
                return std::nullopt;
            indices[i] = static_cast<int>(*matchResult);
        }
        if (indices[0] >= 0 && indices[0] == indices[1])
            return std::nullopt;
        return indices;
    }

    int chooseInactiveTarget(const std::array<int, 2>& indices) const
    {
        // 两个都未激活时记录可能丢了，刚击发过的那个更可能是已命中的。
        if (shotTarget_)
        {
            for (int index : indices)
            {
                if (index >= 0 && !targets_[index].active &&
                    static_cast<std::size_t>(index) != *shotTarget_)
                {
                    return index;
                }
            }
        }
        for (int index : indices)
        {
            if (index >= 0 && !targets_[index].active &&
                !containsIndex(previousIndices_, index))
            {
                return index;
            }
        }
        if (selectedIndex_ && containsIndex(indices,
                                            static_cast<int>(*selectedIndex_)) &&
            !targets_[*selectedIndex_].active)
        {
            return static_cast<int>(*selectedIndex_);
        }
        for (int index : indices)
        {
            if (index >= 0 && !targets_[index].active)
                return index;
        }
        return -1;
    }

    int findRefreshedInactive(const std::array<int, 2>& observed,
                              const std::array<int, 2>& visibleAtFire) const
    {
        for (int index : observed)
        {
            if (index < 0 || targets_[index].active ||
                containsIndex(visibleAtFire, index))
            {
                continue;
            }
            return index;
        }
        return -1;
    }

    static bool containsIndex(const std::array<int, 2>& indices, int index)
    {
        return indices[0] == index || indices[1] == index;
    }

    std::optional<frc::Translation3d> positionForIndex(
        const std::array<int, 2>& indices,
        const std::vector<frc::Translation3d>& positions,
        int index) const
    {
        for (std::size_t i = 0; i < positions.size(); ++i)
        {
            if (indices[i] == index)
                return positions[i];
        }
        return std::nullopt;
    }

    bool allActive() const
    {
        return std::all_of(targets_.begin(), targets_.end(),
            [](const Target& target) { return target.active; });
    }

    std::optional<frc::Rotation2d> posToAngle(
        const frc::Translation3d& position) const
    {
        return ToYZ(position, center_).Angle();
    }

    void advanceRotations(int now, double omega)
    {
        if (!initialized_ || now <= lastUpdateAt_ || !std::isfinite(omega))
        {
            lastUpdateAt_ = std::max(lastUpdateAt_, now);
            return;
        }

        const units::radian_t delta{
            (anti_ ? 1 : -1) * omega * (now - lastUpdateAt_) / 1000.0};
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
                continue;
            const auto rotation = posToAngle(position);
            if (!rotation)
                continue;
            const auto index = *candidate;
            const double delta = absRotation(
                *rotation - targets_[index].rotation).Degrees().value();
            if (delta < anchorDelta)
            {
                anchorDelta = delta;
                anchorIndex = index;
                anchorPosition = &position;
            }
        }

        if (!anchorIndex)
            return false;
        const auto rotation = posToAngle(*anchorPosition);
        if (!rotation)
            return false;

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
        const auto rotation = posToAngle(position);
        if (!rotation)
            return std::nullopt;

        std::optional<std::size_t> bestMatch;
        double minDelta = std::numeric_limits<double>::max();
        for (std::size_t i = 0; i < targets_.size(); ++i)
        {
            const double delta = absRotation(
                *rotation - targets_[i].rotation).Degrees().value();
            if (delta <= kTARGET_MATCH_TOLERANCE.value() && delta < minDelta)
            {
                minDelta = delta;
                bestMatch = i;
            }
        }
        return bestMatch;
    }

    std::array<Target, 5> targets_;
    frc::Translation3d center_;
    int lastUpdateAt_ = 0;
    bool initialized_ = false;
    bool recordHitTargets_ = true;
    bool anti_ = true;
    Debouncer singleTargetDebouncer_{
        kSINGLE_TARGET_RESET_FRAMES, DebounceEdge::Rising};
    bool singleStable_ = false;
    std::optional<std::size_t> selectedIndex_;
    std::optional<std::size_t> lastFiredIndex_;
    std::optional<PendingShot> pendingShot_;
    std::optional<std::size_t> shotTarget_;
    int shotCount_ = 0;
    std::array<int, 2> previousIndices_{-1, -1};
};

#endif // TARGET_MATCHER_SWITCH_H
