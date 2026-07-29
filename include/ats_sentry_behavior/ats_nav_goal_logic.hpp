// Copyright 2026
//
// ATS 导航 action 的纯逻辑判据（P6 二.1）。
//
// 这里只放不依赖 ROS 通信的判据，目的是让「终端成功从哪里来」「航点如何推进」
// 可以被 gtest 直接覆盖，不需要真实 action server：
//
//  1. 终端成功只能来自 `ats_navigation_interfaces/action/NavigateToPose` 的
//     `result_code == RESULT_SUCCEEDED`。位置容差（`IsPathGoalReached`）只能
//     作为辅助判据，禁止替代 action 的终端成功。
//  2. 目标管理器只暴露单目标 action（`/ats_navigate_to_pose`），没有
//     through-poses 等价接口，因此多航点路径必须由行为层的航点状态机逐点下发。
//  3. 航点状态机必须能区分「同一条路径的重复下发」与「真的换了一条路径」，
//     否则每个 tick 都会抢占当前目标，实车上会表现为反复起停。

#ifndef ATS_SENTRY_BEHAVIOR__ATS_NAV_GOAL_LOGIC_HPP_
#define ATS_SENTRY_BEHAVIOR__ATS_NAV_GOAL_LOGIC_HPP_

#include <cstddef>
#include <cstdint>

#include "ats_navigation_interfaces/action/navigate_to_pose.hpp"
#include "ats_sentry_behavior/decision_utils.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

namespace ats_sentry_behavior::decision
{

/// 与 `ats_navigation_interfaces/action/NavigateToPose` 的 result 常量一一对应。
enum class AtsNavOutcome : uint8_t {
  kSucceeded = 0,
  kCanceled = 1,
  kPreempted = 2,
  kTimeout = 3,
  kMapUnready = 4,
  kPlanningFailed = 5,
  kTfFailed = 6,
  kInternalError = 7,
  /// 服务端给出了本行为层未知的 result_code：一律按失败处理，不猜测语义。
  kUnknown = 255,
};

inline AtsNavOutcome atsNavOutcomeFromResultCode(uint8_t result_code)
{
  using Result = ats_navigation_interfaces::action::NavigateToPose::Result;
  switch (result_code) {
    case Result::RESULT_SUCCEEDED:
      return AtsNavOutcome::kSucceeded;
    case Result::RESULT_CANCELED:
      return AtsNavOutcome::kCanceled;
    case Result::RESULT_PREEMPTED:
      return AtsNavOutcome::kPreempted;
    case Result::RESULT_TIMEOUT:
      return AtsNavOutcome::kTimeout;
    case Result::RESULT_MAP_UNREADY:
      return AtsNavOutcome::kMapUnready;
    case Result::RESULT_PLANNING_FAILED:
      return AtsNavOutcome::kPlanningFailed;
    case Result::RESULT_TF_FAILED:
      return AtsNavOutcome::kTfFailed;
    case Result::RESULT_INTERNAL_ERROR:
      return AtsNavOutcome::kInternalError;
    default:
      return AtsNavOutcome::kUnknown;
  }
}

/// 只有 `kSucceeded` 是终端成功；其余全部是失败，禁止按「接近终点」宽容处理。
inline bool atsNavOutcomeIsSuccess(AtsNavOutcome outcome)
{
  return outcome == AtsNavOutcome::kSucceeded;
}

inline const char * atsNavOutcomeName(AtsNavOutcome outcome)
{
  switch (outcome) {
    case AtsNavOutcome::kSucceeded:
      return "SUCCEEDED 终端成功";
    case AtsNavOutcome::kCanceled:
      return "CANCELED 被取消";
    case AtsNavOutcome::kPreempted:
      return "PREEMPTED 被抢占";
    case AtsNavOutcome::kTimeout:
      return "TIMEOUT 超时";
    case AtsNavOutcome::kMapUnready:
      return "MAP_UNREADY 地图未就绪";
    case AtsNavOutcome::kPlanningFailed:
      return "PLANNING_FAILED 规划失败";
    case AtsNavOutcome::kTfFailed:
      return "TF_FAILED 坐标变换失败";
    case AtsNavOutcome::kInternalError:
      return "INTERNAL_ERROR 服务端内部错误";
    default:
      return "UNKNOWN 未知 result_code";
  }
}

/// 路径更新的处理结果。
enum class AtsWaypointUpdate : uint8_t {
  /// 与当前路径等价：保持进度，不得重新下发目标。
  kUnchanged = 0,
  /// 新路径：进度清零，必须取消当前目标并从第一个航点重新下发。
  kRestarted = 1,
};

/// 多航点路径在单目标 action 上的推进状态机。
///
/// 只在 `onWaypointSucceeded()` 被调用时前进，而该调用只允许由
/// `result_code == RESULT_SUCCEEDED` 触发，因此「整条路径完成」是一串
/// 服务端终端成功的合成结果，不含任何位置容差判断。
class AtsWaypointSequencer
{
public:
  explicit AtsWaypointSequencer(double hold_tolerance = 0.25) : hold_tolerance_(hold_tolerance) {}

  void setHoldTolerance(double hold_tolerance) { hold_tolerance_ = hold_tolerance; }
  double holdTolerance() const { return hold_tolerance_; }

  /// 判断给定路径是否就是当前正在执行的那一条（容差内等价）。
  bool matches(const nav_msgs::msg::Path & path) const
  {
    if (!has_path_ || path.poses.empty()) {
      return false;
    }
    return pathEquivalent(path_, path, hold_tolerance_);
  }

  AtsWaypointUpdate updatePath(const nav_msgs::msg::Path & path)
  {
    if (path.poses.empty()) {
      reset();
      return AtsWaypointUpdate::kRestarted;
    }
    if (matches(path)) {
      // 只刷新 header 与姿态数值，不动进度：同一导航意图的重复下发。
      path_ = path;
      return AtsWaypointUpdate::kUnchanged;
    }
    path_ = path;
    has_path_ = true;
    index_ = 0;
    finished_ = false;
    return AtsWaypointUpdate::kRestarted;
  }

  void reset()
  {
    path_ = nav_msgs::msg::Path();
    has_path_ = false;
    index_ = 0;
    finished_ = false;
  }

  /// 一个航点拿到服务端终端成功后前进；越过最后一个航点才算整条路径完成。
  void onWaypointSucceeded()
  {
    if (!has_path_ || finished_) {
      return;
    }
    ++index_;
    if (index_ >= path_.poses.size()) {
      finished_ = true;
    }
  }

  /// 任何非成功结局都清掉进度：下一拍必须重新从第一个航点下发。
  void onFailure()
  {
    index_ = 0;
    finished_ = false;
    has_path_ = false;
    path_ = nav_msgs::msg::Path();
  }

  bool valid() const { return has_path_ && !path_.poses.empty(); }
  bool finished() const { return finished_; }
  std::size_t index() const { return index_; }
  std::size_t size() const { return path_.poses.size(); }
  const nav_msgs::msg::Path & path() const { return path_; }

  /// 仅在 `valid() && !finished()` 时有效。
  const geometry_msgs::msg::PoseStamped & currentWaypoint() const { return path_.poses.at(index_); }

private:
  nav_msgs::msg::Path path_;
  bool has_path_ = false;
  bool finished_ = false;
  std::size_t index_ = 0;
  double hold_tolerance_ = 0.25;
};

}  // namespace ats_sentry_behavior::decision

#endif  // ATS_SENTRY_BEHAVIOR__ATS_NAV_GOAL_LOGIC_HPP_
