// Copyright 2026
//
// ATS 导航 action 的行为树节点（P6 二.1）。
//
// 与被它替代的 `SendNavThroughPoses` 的关键差别：
//
//  1. 派生自 `BT::RosActionNode`，tick 非阻塞。旧节点是 `BT::SyncActionNode`，
//     在 tick 里调用 `wait_for_action_server()`，属于「sync tick 内无界等待」。
//  2. `halt()` 由框架自动 `cancelGoal()` 再回调 `onHalt()`，因此上层
//     `ReactiveFallback` 抢占时目标会真的被取消，而不是留在服务端继续跑。
//  3. 终端成功只来自 `result_code == RESULT_SUCCEEDED`。feedback 里的
//     `distance_remaining` 只做进度日志，禁止用来提前判成功。
//  4. 多航点路径由 `decision::AtsWaypointSequencer` 逐点下发；目标管理器只暴露
//     单目标 action，没有 through-poses 等价接口。
//
// 迟到 result 的处理由框架负责：`goal_options.result_callback` 只接受
// `goal_handle_->get_goal_id() == result.goal_id` 的样本，重新 IDLE 后
// `result_` 会被清空，所以上一轮的迟到结果不会污染新一轮。

#ifndef ATS_SENTRY_BEHAVIOR__PLUGINS__ACTION__SEND_ATS_NAV_GOAL_HPP_
#define ATS_SENTRY_BEHAVIOR__PLUGINS__ACTION__SEND_ATS_NAV_GOAL_HPP_

#include <memory>
#include <string>

#include "ats_navigation_interfaces/action/navigate_to_pose.hpp"
#include "ats_sentry_behavior/ats_nav_goal_logic.hpp"
#include "behaviortree_ros2/bt_action_node.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

namespace ats_sentry_behavior
{

class SendAtsNavGoalAction
: public BT::RosActionNode<ats_navigation_interfaces::action::NavigateToPose>
{
public:
  using AtsNavigateToPose = ats_navigation_interfaces::action::NavigateToPose;

  SendAtsNavGoalAction(
    const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params);

  static BT::PortsList providedPorts();

  /// 在框架 tick 之前处理两件框架管不了的事：
  ///  - 整条路径已经拿到全部 `RESULT_SUCCEEDED`：直接 SUCCESS，不再下发目标；
  ///  - 执行期间路径换成了另一条：先取消当前目标，再从新路径第一个航点重新下发。
  BT::NodeStatus tick() override;

  bool setGoal(Goal & goal) override;

  BT::NodeStatus onResultReceived(const WrappedResult & wr) override;

  BT::NodeStatus onFeedback(const std::shared_ptr<const Feedback> feedback) override;

  BT::NodeStatus onFailure(BT::ActionNodeErrorCode error) override;

  /// 兜住框架 `cancelGoal()` 在陈旧 goal handle 上抛出的
  /// `rclcpp_action::exceptions::UnknownGoalHandleError`。
  ///
  /// 上游 `BT::RosActionNode` 在重新进入 IDLE 时只清 `goal_received_`、
  /// `future_goal_handle_`、`result_`，不清 `goal_handle_`（该成员是 private，
  /// 派生类无法复位）。于是"上一个目标已终止 + 新目标的 handle 还没到"这个窗口里，
  /// `halt()` 会去 cancel 一个客户端已经忘记的 handle 并抛异常。
  /// 这个异常正好落在停车链的入口（halt -> cancel -> `ExecutionCommand STOP`），
  /// 未捕获会直接终止行为树节点进程，因此必须在这里吞掉并保证进度被清空。
  void halt() override;

  void onHalt() override;

private:
  /// 从端口读取路径，并把航点状态机推进到「本次应下发哪个航点」。
  /// 返回 false 表示端口缺失或路径为空，调用方必须让本次 tick 失败。
  bool refreshWaypoint(geometry_msgs::msg::PoseStamped & waypoint);

  decision::AtsWaypointSequencer sequencer_;
  /// 服务端超时：<=0 时按 action 定义交由服务端默认值决定。
  double goal_timeout_s_ = 0.0;
  std::string planning_frame_ = "map";
  /// 本轮 tick 实际下发的航点序号，仅用于日志。
  std::size_t sent_index_ = 0;
  std::size_t sent_total_ = 0;
};

}  // namespace ats_sentry_behavior

#endif  // ATS_SENTRY_BEHAVIOR__PLUGINS__ACTION__SEND_ATS_NAV_GOAL_HPP_
