// Copyright 2026
//
// 对照 profile 的 `SendNavThroughPoses` 的 halt/cancel 语义测试（P6 二.5）。
//
// 该节点原为 `BT::SyncActionNode`：`async_send_goal()` 之后立刻返回 SUCCESS，
// 而 `SyncActionNode::halt()` 是 `final`（只做 resetStatus）。两者叠加的后果是
// 主树 halt 掉导航分支时，Nav2 侧的目标仍在执行，
// 「授权 STOP -> 串口零速度」这条链在对照 profile 下整条失效。
//
// 本测试起真的 `nav2_msgs/action/NavigateToPose` 与 `NavigateThroughPoses`
// 服务端，覆盖：
//   1. 目标在飞行中时节点必须是 RUNNING（否则 halt 不会调用 onHalted）；
//   2. `haltTree()` 必须让服务端真的收到 cancel；
//   3. 服务端不存在时按超时判 FAILURE，且 tick 内不阻塞（无界等待回归）。

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "ats_sentry_behavior/plugins/action/send_nav_through_poses.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "nav2_msgs/action/navigate_through_poses.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace
{

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using NavigateThroughPoses = nav2_msgs::action::NavigateThroughPoses;
using namespace std::chrono_literals;

constexpr char kToPoseName[] = "/nav2_to_pose_halt_test";
constexpr char kThroughPosesName[] = "/nav2_through_poses_halt_test";

/// 只 hold 不收尾的 Nav2 服务端：用于制造「目标正在执行」的状态。
template <typename ActionT>
class HoldingServer
{
public:
  using GoalHandle = rclcpp_action::ServerGoalHandle<ActionT>;

  HoldingServer(const rclcpp::Node::SharedPtr & node, const std::string & name)
  {
    server_ = rclcpp_action::create_server<ActionT>(
      node, name,
      [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const typename ActionT::Goal>) {
        ++goals_received_;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [this](const std::shared_ptr<GoalHandle>) {
        ++cancels_received_;
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](const std::shared_ptr<GoalHandle> handle) {
        // 与 test_send_ats_nav_goal_action.cpp 同样的理由：不能 detach，
        // detach 的线程持有裸 `this`，TearDown 销毁本对象后就是 use-after-free。
        std::lock_guard<std::mutex> lock(workers_mutex_);
        workers_.emplace_back([this, handle]() { execute(handle); });
      });
  }

  ~HoldingServer()
  {
    shutting_down_ = true;
    std::vector<std::thread> workers;
    {
      std::lock_guard<std::mutex> lock(workers_mutex_);
      workers.swap(workers_);
    }
    for (auto & worker : workers) {
      if (worker.joinable()) {
        worker.join();
      }
    }
  }

  int goalsReceived() const { return goals_received_; }
  int cancelsReceived() const { return cancels_received_; }

private:
  void execute(const std::shared_ptr<GoalHandle> handle)
  {
    auto result = std::make_shared<typename ActionT::Result>();
    // `shutting_down_` 是必需的退出条件，否则析构函数会死锁在 join 上。
    while (!shutting_down_ && !handle->is_canceling() && rclcpp::ok()) {
      std::this_thread::sleep_for(5ms);
    }
    if (handle->is_canceling()) {
      handle->canceled(result);
      return;
    }
    handle->abort(result);
  }

  typename rclcpp_action::Server<ActionT>::SharedPtr server_;
  std::mutex workers_mutex_;
  std::vector<std::thread> workers_;
  std::atomic<bool> shutting_down_{false};
  std::atomic<int> goals_received_{0};
  std::atomic<int> cancels_received_{0};
};

nav_msgs::msg::Path makePath(std::size_t count)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  for (std::size_t i = 0; i < count; ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.pose.position.x = static_cast<double>(i) * 2.0;
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }
  return path;
}

class SendNavThroughPosesHaltTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  void SetUp() override
  {
    server_node_ = std::make_shared<rclcpp::Node>("nav2_halt_test_server");
    to_pose_server_ = std::make_unique<HoldingServer<NavigateToPose>>(server_node_, kToPoseName);
    through_poses_server_ =
      std::make_unique<HoldingServer<NavigateThroughPoses>>(server_node_, kThroughPosesName);
    server_executor_ = std::make_unique<rclcpp::executors::MultiThreadedExecutor>();
    server_executor_->add_node(server_node_);
    server_thread_ = std::thread([this]() { server_executor_->spin(); });

    client_node_ = std::make_shared<rclcpp::Node>("nav2_halt_test_client");
    client_node_->declare_parameter(
      "decision.decision_config.nav2_action_server", std::string(kThroughPosesName));
    client_node_->declare_parameter(
      "decision.decision_config.nav2_to_pose_action_server", std::string(kToPoseName));
    client_node_->declare_parameter("decision.decision_config.action_server_wait_timeout_s", 0.5);

    client_executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    client_executor_->add_node(client_node_);
    client_thread_ = std::thread([this]() { client_executor_->spin(); });

    factory_.registerNodeType<ats_sentry_behavior::SendNavThroughPosesAction>(
      "SendNavThroughPoses");
  }

  void TearDown() override
  {
    tree_.reset();
    client_executor_->cancel();
    if (client_thread_.joinable()) {
      client_thread_.join();
    }
    // 先析构服务端（此时 server executor 仍在 spin，收尾调用仍然有效），再停 executor。
    to_pose_server_.reset();
    through_poses_server_.reset();
    server_executor_->cancel();
    if (server_thread_.joinable()) {
      server_thread_.join();
    }
    server_node_.reset();
    client_node_.reset();
  }

  void makeTree()
  {
    static constexpr char kXml[] = R"(
      <root BTCPP_format="4">
        <BehaviorTree ID="MainTree">
          <Sequence>
            <SendNavThroughPoses path="{decision_path}"
                                 goal_succeeded="{decision_nav_goal_succeeded}"/>
          </Sequence>
        </BehaviorTree>
      </root>)";
    blackboard_ = BT::Blackboard::create();
    blackboard_->set("node", client_node_);
    blackboard_->set("decision_nav_goal_succeeded", false);
    tree_ = std::make_unique<BT::Tree>(factory_.createTreeFromText(kXml, blackboard_));
  }

  void setPath(const nav_msgs::msg::Path & path) { blackboard_->set("decision_path", path); }

  BT::NodeStatus tickUntil(BT::NodeStatus wanted, std::chrono::milliseconds timeout = 3000ms)
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    BT::NodeStatus status = BT::NodeStatus::IDLE;
    do {
      status = tree_->tickExactlyOnce();
      if (status == wanted) {
        return status;
      }
      std::this_thread::sleep_for(10ms);
    } while (std::chrono::steady_clock::now() < deadline);
    return status;
  }

  rclcpp::Node::SharedPtr server_node_;
  rclcpp::Node::SharedPtr client_node_;
  std::unique_ptr<HoldingServer<NavigateToPose>> to_pose_server_;
  std::unique_ptr<HoldingServer<NavigateThroughPoses>> through_poses_server_;
  std::unique_ptr<rclcpp::executors::MultiThreadedExecutor> server_executor_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> client_executor_;
  std::thread server_thread_;
  std::thread client_thread_;
  BT::BehaviorTreeFactory factory_;
  BT::Blackboard::Ptr blackboard_;
  std::unique_ptr<BT::Tree> tree_;
};

}  // namespace

// 单点路径（NavigateToPose）：目标在飞行中必须是 RUNNING，halt 必须真的 cancel。
TEST_F(SendNavThroughPosesHaltTest, HaltCancelsSinglePoseGoal)
{
  makeTree();
  setPath(makePath(1));

  ASSERT_EQ(tickUntil(BT::NodeStatus::RUNNING), BT::NodeStatus::RUNNING);
  const auto deadline = std::chrono::steady_clock::now() + 3000ms;
  while (to_pose_server_->goalsReceived() < 1 && std::chrono::steady_clock::now() < deadline) {
    tree_->tickExactlyOnce();
    std::this_thread::sleep_for(10ms);
  }
  ASSERT_EQ(to_pose_server_->goalsReceived(), 1);
  ASSERT_EQ(to_pose_server_->cancelsReceived(), 0);

  tree_->haltTree();

  const auto cancel_deadline = std::chrono::steady_clock::now() + 2000ms;
  while (to_pose_server_->cancelsReceived() < 1 &&
         std::chrono::steady_clock::now() < cancel_deadline) {
    std::this_thread::sleep_for(10ms);
  }
  EXPECT_EQ(to_pose_server_->cancelsReceived(), 1);
}

// 多点路径（NavigateThroughPoses）：同样必须可 halt。
TEST_F(SendNavThroughPosesHaltTest, HaltCancelsMultiPoseGoal)
{
  makeTree();
  setPath(makePath(3));

  ASSERT_EQ(tickUntil(BT::NodeStatus::RUNNING), BT::NodeStatus::RUNNING);
  const auto deadline = std::chrono::steady_clock::now() + 3000ms;
  while (through_poses_server_->goalsReceived() < 1 &&
         std::chrono::steady_clock::now() < deadline) {
    tree_->tickExactlyOnce();
    std::this_thread::sleep_for(10ms);
  }
  ASSERT_EQ(through_poses_server_->goalsReceived(), 1);

  tree_->haltTree();

  const auto cancel_deadline = std::chrono::steady_clock::now() + 2000ms;
  while (through_poses_server_->cancelsReceived() < 1 &&
         std::chrono::steady_clock::now() < cancel_deadline) {
    std::this_thread::sleep_for(10ms);
  }
  EXPECT_EQ(through_poses_server_->cancelsReceived(), 1);
}

// 空路径：判失败，且不得下发目标。
TEST_F(SendNavThroughPosesHaltTest, EmptyPathFailsWithoutSendingGoal)
{
  makeTree();
  setPath(nav_msgs::msg::Path());

  EXPECT_EQ(tickUntil(BT::NodeStatus::FAILURE), BT::NodeStatus::FAILURE);
  EXPECT_EQ(to_pose_server_->goalsReceived(), 0);
  EXPECT_EQ(through_poses_server_->goalsReceived(), 0);
}

// 回归：server 不可用时按超时判 FAILURE，且单拍不得阻塞到超时时长。
//
// 原实现在 tick 内调用 `wait_for_action_server(2.0s)`，那是 sync tick 内的
// 无界等待：最坏每拍阻塞 2 s，整棵树的反应性一起被拖死。
TEST_F(SendNavThroughPosesHaltTest, MissingServerTimesOutWithoutBlockingTick)
{
  client_node_->set_parameter(rclcpp::Parameter(
    "decision.decision_config.nav2_to_pose_action_server", std::string("/nav2_absent_server")));
  makeTree();
  setPath(makePath(1));

  const auto first_tick_start = std::chrono::steady_clock::now();
  const auto first_status = tree_->tickExactlyOnce();
  const auto first_tick_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                               std::chrono::steady_clock::now() - first_tick_start)
                               .count();

  // 第一拍不判失败（还在等待窗口内），但也绝不能阻塞掉整个等待窗口。
  EXPECT_EQ(first_status, BT::NodeStatus::RUNNING);
  EXPECT_LT(first_tick_ms, 200);

  // 超过 action_server_wait_timeout_s=0.5 后必须判 FAILURE。
  EXPECT_EQ(tickUntil(BT::NodeStatus::FAILURE, 3000ms), BT::NodeStatus::FAILURE);
}
