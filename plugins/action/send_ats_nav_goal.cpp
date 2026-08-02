// Copyright 2026
//
// 见 `include/ats_sentry_behavior/plugins/action/send_ats_nav_goal.hpp`。

#include "ats_sentry_behavior/plugins/action/send_ats_nav_goal.hpp"

#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/exceptions.hpp"

namespace ats_sentry_behavior
{

SendAtsNavGoalAction::SendAtsNavGoalAction(
  const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params)
: RosActionNode<AtsNavigateToPose>(name, conf, params)
{
  double hold_tolerance = sequencer_.holdTolerance();
  std::string action_server = "/ats_navigate_to_pose";
  if (auto node = node_.lock()) {
    node->get_parameter("decision.decision_config.active_goal_hold_tolerance", hold_tolerance);
    node->get_parameter("decision.decision_config.ats_goal_timeout_s", goal_timeout_s_);
    node->get_parameter("decision.decision_config.ats_action_server", action_server);
    node->get_parameter("decision.pose.expected_frame", planning_frame_);
  }
  sequencer_.setHoldTolerance(hold_tolerance);

  // `RegisterPlugins` 不提供 default_port_value，因此没有 action_name 端口时
  // 客户端会是空的，基类 tick 会直接抛 RuntimeError。这里用参数补上唯一权威的
  // 正式 action 名字由根 node_params.yaml 的 decision_config 给出。
  if (action_server.empty()) {
    RCLCPP_ERROR(
      logger(),
      "decision.decision_config.ats_action_server 为空：SendAtsNavGoal 无法确定 action 名字。");
  } else {
    setActionName(action_server);
  }
}

BT::PortsList SendAtsNavGoalAction::providedPorts()
{
  return providedBasicPorts(
    {BT::InputPort<nav_msgs::msg::Path>("path", "{decision_path}", "待执行的决策路径"),
     BT::OutputPort<bool>(
       "goal_succeeded", "{decision_nav_goal_succeeded}",
       "整条路径的所有航点都拿到 RESULT_SUCCEEDED 才为 true")});
}

bool SendAtsNavGoalAction::refreshWaypoint(geometry_msgs::msg::PoseStamped & waypoint)
{
  auto path = getInput<nav_msgs::msg::Path>("path");
  if (!path || path->poses.empty()) {
    RCLCPP_ERROR(logger(), "SendAtsNavGoal 没有拿到有效的 path 输入。");
    return false;
  }

  const auto update = sequencer_.updatePath(*path);
  if (update == decision::AtsWaypointUpdate::kRestarted) {
    RCLCPP_INFO(
      logger(), "SendAtsNavGoal 收到新路径，共 %zu 个航点，从第 1 个开始下发。", sequencer_.size());
  }
  if (!sequencer_.valid()) {
    return false;
  }
  if (sequencer_.finished()) {
    // 整条路径已在上一轮完成：不重复下发，交由 tick 判定 SUCCESS。
    return false;
  }

  waypoint = sequencer_.currentWaypoint();
  sent_index_ = sequencer_.index();
  sent_total_ = sequencer_.size();
  return true;
}

BT::NodeStatus SendAtsNavGoalAction::tick()
{
  auto path = getInput<nav_msgs::msg::Path>("path");
  if (!path || path->poses.empty()) {
    if (status() == BT::NodeStatus::RUNNING) {
      // 路径消失时必须真的把服务端目标取消掉，不能只让本节点返回失败。
      halt();
    }
    sequencer_.onFailure();
    setOutput("goal_succeeded", false);
    RCLCPP_ERROR(logger(), "SendAtsNavGoal 没有拿到有效的 path 输入：判失败并取消目标。");
    return BT::NodeStatus::FAILURE;
  }

  const bool same_path = sequencer_.matches(*path);

  if (status() == BT::NodeStatus::RUNNING && !same_path) {
    // 执行中换了另一条路径：先取消当前目标（halt() 会调用 cancelGoal），
    // 再回到 IDLE，由基类在本拍重新下发新路径的第一个航点。
    RCLCPP_INFO(logger(), "SendAtsNavGoal 路径被替换：取消当前目标后重新下发。");
    halt();
    resetStatus();
  }

  if (same_path && sequencer_.finished()) {
    // 整条路径的每个航点都拿到过 RESULT_SUCCEEDED，这是唯一的终端成功来源。
    setOutput("goal_succeeded", true);
    return BT::NodeStatus::SUCCESS;
  }

  if (BT::isStatusCompleted(status())) {
    // 上一拍已经收敛（例如单个航点完成）。父节点未必会替我们复位，这里显式回到
    // IDLE，避免基类 tick 在既非 IDLE 也非 RUNNING 时空转返回 RUNNING。
    resetStatus();
  }

  return RosActionNode<AtsNavigateToPose>::tick();
}

bool SendAtsNavGoalAction::setGoal(Goal & goal)
{
  geometry_msgs::msg::PoseStamped waypoint;
  if (!refreshWaypoint(waypoint)) {
    return false;
  }

  goal.goal_pose = waypoint;
  if (goal.goal_pose.header.frame_id.empty()) {
    goal.goal_pose.header.frame_id = planning_frame_;
  }
  goal.goal_pose.header.stamp = now();

  // timeout <= 0 时按 action 定义使用服务端默认值，不在行为层另造一套超时。
  if (goal_timeout_s_ > 0.0) {
    const auto duration = rclcpp::Duration::from_seconds(goal_timeout_s_);
    goal.timeout = duration;
  } else {
    goal.timeout.sec = 0;
    goal.timeout.nanosec = 0U;
  }

  setOutput("goal_succeeded", false);
  RCLCPP_INFO(
    logger(), "下发 ATS 导航目标 %zu/%zu：(%.2f, %.2f) frame=%s timeout=%.2f s", sent_index_ + 1,
    sent_total_, goal.goal_pose.pose.position.x, goal.goal_pose.pose.position.y,
    goal.goal_pose.header.frame_id.c_str(), goal_timeout_s_);
  return true;
}

BT::NodeStatus SendAtsNavGoalAction::onResultReceived(const WrappedResult & wr)
{
  const auto outcome = decision::atsNavOutcomeFromResultCode(wr.result->result_code);
  if (!decision::atsNavOutcomeIsSuccess(outcome)) {
    // 终端成功只认 RESULT_SUCCEEDED，其余一律失败，不做「接近终点」的宽容判定。
    sequencer_.onFailure();
    setOutput("goal_succeeded", false);
    RCLCPP_WARN(
      logger(), "ATS 导航目标结束于 %s：%s。行为层判失败并清空航点进度。",
      decision::atsNavOutcomeName(outcome), wr.result->message.c_str());
    return BT::NodeStatus::FAILURE;
  }

  sequencer_.onWaypointSucceeded();
  const bool finished = sequencer_.finished();
  setOutput("goal_succeeded", finished);
  if (finished) {
    RCLCPP_INFO(
      logger(), "ATS 导航整条路径完成：%zu 个航点全部拿到 RESULT_SUCCEEDED。", sent_total_);
  } else {
    // 单目标 action 没有 through-poses 等价接口，因此本节点在下一拍重新进入
    // IDLE 时下发下一个航点；此处返回 SUCCESS 但 goal_succeeded 仍为 false。
    RCLCPP_INFO(
      logger(), "航点 %zu/%zu 完成，下一拍下发第 %zu 个。", sent_index_ + 1, sent_total_,
      sequencer_.index() + 1);
  }
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus SendAtsNavGoalAction::onFeedback(const std::shared_ptr<const Feedback> feedback)
{
  // feedback 只做进度观测：state 与 distance_remaining 都不得用于判定终端成功。
  RCLCPP_DEBUG(
    logger(), "ATS 导航进度：state=%u 剩余 %.2f m 已用 %.2f s",
    static_cast<unsigned>(feedback->state), feedback->distance_remaining, feedback->elapsed_sec);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus SendAtsNavGoalAction::onFailure(BT::ActionNodeErrorCode error)
{
  const char * detail = "未知错误";
  switch (error) {
    case BT::SERVER_UNREACHABLE:
      detail = "action server 不可达：目标管理器未启动或话题名不一致";
      break;
    case BT::SEND_GOAL_TIMEOUT:
      detail = "发送目标超时：server_timeout 内没拿到 goal handle";
      break;
    case BT::GOAL_REJECTED_BY_SERVER:
      detail = "目标被服务端拒绝";
      break;
    case BT::ACTION_ABORTED:
      detail = "服务端 abort：按 result_code 判失败";
      break;
    case BT::ACTION_CANCELLED:
      detail = "目标被取消";
      break;
    case BT::INVALID_GOAL:
      detail = "目标无效：路径缺失、为空或整条路径已完成";
      break;
  }
  sequencer_.onFailure();
  setOutput("goal_succeeded", false);
  RCLCPP_ERROR(logger(), "SendAtsNavGoal 失败（%s）：%s", BT::toStr(error), detail);
  return BT::NodeStatus::FAILURE;
}

void SendAtsNavGoalAction::halt()
{
  try {
    RosActionNode<AtsNavigateToPose>::halt();
  } catch (const rclcpp_action::exceptions::UnknownGoalHandleError & e) {
    // 基类的 `goal_handle_` 是 private，重新进入 IDLE 时不会被清空，因此
    // 「上一个目标已终止 + 新目标 handle 尚未到达」的窗口里 cancelGoal 必然抛。
    // 这里必须吞掉：halt 是停车链的入口，抛出去会让整棵树的这一拍炸掉，
    // 反而拿不到 `ExecutionCommand STOP`。同时手工补 `onHalt()` 的清理动作，
    // 因为异常发生在基类调用 `onHalt()` 之前。
    RCLCPP_WARN(
      logger(), "SendAtsNavGoal halt 时目标句柄已失效（%s）：按已取消处理并清空航点进度。",
      e.what());
    onHalt();
    resetStatus();
  }
}

void SendAtsNavGoalAction::onHalt()
{
  // 框架已在 halt() 中先调用 cancelGoal()。这里只清进度，保证被抢占后重新
  // 进入时从第一个航点重新下发，不复用旧的执行授权。
  sequencer_.onFailure();
  setOutput("goal_succeeded", false);
  RCLCPP_INFO(logger(), "SendAtsNavGoal 被 halt：已取消目标并清空航点进度。");
}

}  // namespace ats_sentry_behavior

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(ats_sentry_behavior::SendAtsNavGoalAction, "SendAtsNavGoal");
