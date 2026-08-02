// Copyright 2026
//
// `SendAtsNavGoal` 的 action 语义测试（P6 二.5）。
//
// 这里起一个真的 `ats_navigation_interfaces/action/NavigateToPose` 服务端，
// 因此覆盖的是端到端 action 行为，而不是重写一遍判据：
//
//  1. halt/cancel：节点 RUNNING 时 halt 必须让服务端真的收到 cancel 请求
//     （这是 `ExecutionCommand STOP -> 串口零速度` 的唯一触发入口）；
//  2. 迟到 result：被 halt/取消的那一轮的 result 到达后，不得污染新一轮，
//     也不得把 `goal_succeeded` 置成 true；
//  3. result_code 映射：只有 `RESULT_SUCCEEDED` 才是终端成功，
//     `RESULT_TIMEOUT` / `RESULT_MAP_UNREADY` 等一律 FAILURE；
//  4. 航点状态机：多航点路径逐点下发，只有全部航点成功才输出
//     `goal_succeeded=true`；中途失败必须从第一个航点重来。
//  5. 服务端重启：旧 action server incarnation 的未完成目标不得污染重启后的
//     新路径；在服务端缺席时 halt 仍是确定性的停车入口。
//
// 注意：本测试不判定任何位置容差。终端成功只来自服务端 result_code。

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "ats_navigation_interfaces/action/navigate_to_pose.hpp"
#include "ats_sentry_behavior/plugins/action/send_ats_nav_goal.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace
{

using AtsNavigateToPose = ats_navigation_interfaces::action::NavigateToPose;
using GoalHandle = rclcpp_action::ServerGoalHandle<AtsNavigateToPose>;
using namespace std::chrono_literals;

constexpr char kActionName[] = "/ats_navigate_to_pose_test";

/// 可编程的 ATS 导航服务端。
///
/// `hold_` 为 true 时接受目标后不产出 result，用于制造"正在执行"的状态；
/// 之后可以 `releaseWithCode()` 让它给出指定 result_code，或者等客户端 cancel。
class ScriptedNavServer
{
public:
  explicit ScriptedNavServer(const rclcpp::Node::SharedPtr & node)
  {
    startServing(node);
  }

  void stopServing()
  {
    server_.reset();
  }

  void startServing(const rclcpp::Node::SharedPtr & node)
  {
    const std::uint64_t incarnation = ++incarnation_;
    server_ = rclcpp_action::create_server<AtsNavigateToPose>(
      node, kActionName,
      [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const AtsNavigateToPose::Goal>) {
        ++goals_received_;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [this](const std::shared_ptr<GoalHandle>) {
        ++cancels_received_;
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this, incarnation](const std::shared_ptr<GoalHandle> handle) {
        // 不能 detach：detach 后的线程持有裸 `this`，而 TearDown 会在
        // executor cancel 之后直接销毁本对象，线程再访问 `hold_`/`result_code_`
        // 就是 use-after-free（表现为约 20% 的 `free(): invalid pointer` 或段错误）。
        // 改为登记到 `workers_`，由析构函数统一 join。
        std::lock_guard<std::mutex> lock(workers_mutex_);
        workers_.emplace_back([this, handle, incarnation]() { execute(handle, incarnation); });
      });
  }

  ~ScriptedNavServer()
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

  void setHold(bool hold) { hold_ = hold; }
  void setResultCode(uint8_t code) { result_code_ = code; }
  /// 让当前被 hold 的目标以当前 `result_code_` 收尾。
  void release() { release_ = true; }

  int goalsReceived() const { return goals_received_; }
  int cancelsReceived() const { return cancels_received_; }

private:
  void execute(const std::shared_ptr<GoalHandle> handle, std::uint64_t incarnation)
  {
    auto result = std::make_shared<AtsNavigateToPose::Result>();
    result->result_code = result_code_;
    result->message = "test";

    // `shutting_down_` 是必需的退出条件：析构函数要 join 这些线程，
    // 若 hold 中的目标既没 release 也没 cancel，缺少该条件会死锁在 join 上。
    while (
      hold_ && !release_ && !shutting_down_ && incarnation == incarnation_ &&
      !handle->is_canceling() && rclcpp::ok())
    {
      std::this_thread::sleep_for(5ms);
    }
    release_ = false;

    // 旧服务端已经退出时不得给它的 goal handle 补 result。这样测试的重启
    // 边界与正式 Goal Manager 重启相同：旧 incarnation 只能失效，不能复活。
    if (shutting_down_ || incarnation != incarnation_) {
      return;
    }

    if (handle->is_canceling()) {
      result->result_code = AtsNavigateToPose::Result::RESULT_CANCELED;
      handle->canceled(result);
      return;
    }
    result->result_code = result_code_;
    if (result_code_ == AtsNavigateToPose::Result::RESULT_SUCCEEDED) {
      handle->succeed(result);
    } else {
      // 服务端用 abort 上报非成功结局；行为层仍然按 result_code 判失败。
      handle->abort(result);
    }
  }

  rclcpp_action::Server<AtsNavigateToPose>::SharedPtr server_;
  std::mutex workers_mutex_;
  std::vector<std::thread> workers_;
  std::atomic<bool> shutting_down_{false};
  std::atomic<bool> hold_{false};
  std::atomic<bool> release_{false};
  std::atomic<std::uint64_t> incarnation_{0};
  std::atomic<uint8_t> result_code_{AtsNavigateToPose::Result::RESULT_SUCCEEDED};
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

class SendAtsNavGoalTest : public ::testing::Test
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
    server_node_ = std::make_shared<rclcpp::Node>("ats_nav_test_server");
    server_ = std::make_unique<ScriptedNavServer>(server_node_);
    server_executor_ = std::make_unique<rclcpp::executors::MultiThreadedExecutor>();
    server_executor_->add_node(server_node_);
    server_thread_ = std::thread([this]() { server_executor_->spin(); });

    client_node_ = std::make_shared<rclcpp::Node>("ats_nav_test_client");
    // `SendAtsNavGoal` 的构造函数从这些参数读取唯一权威的 action 名字与容差，
    // 与正式根 node_params.yaml 的 `decision_config` 同名。
    client_node_->declare_parameter(
      "decision.decision_config.ats_action_server", std::string(kActionName));
    client_node_->declare_parameter("decision.decision_config.active_goal_hold_tolerance", 0.25);
    client_node_->declare_parameter("decision.decision_config.ats_goal_timeout_s", 0.0);
    client_node_->declare_parameter("decision.pose.expected_frame", std::string("map"));

    BT::RosNodeParams params;
    params.nh = client_node_;
    params.server_timeout = 1000ms;
    params.wait_for_server_timeout = 2000ms;
    factory_.registerNodeType<ats_sentry_behavior::SendAtsNavGoalAction>("SendAtsNavGoal", params);
  }

  void TearDown() override
  {
    tree_.reset();
    // 顺序有意如此：先让 server_ 析构 join 掉所有 execute 线程（此时 executor
    // 仍在 spin，`handle->succeed()/abort()` 仍能正常完成），再停 executor。
    // 反过来先 cancel executor 会让仍在收尾的 handle 调用打在已停的 executor 上。
    server_.reset();
    server_executor_->cancel();
    if (server_thread_.joinable()) {
      server_thread_.join();
    }
    server_node_.reset();
    client_node_.reset();
  }

  void makeTree()
  {
    // 必须包一层控制节点，和官方树一致：`Tree::haltTree()` 会先 halt 根节点、
    // 再对所有节点做一次"以防万一"的 haltNode 遍历。控制节点在 haltChild 里
    // 会把子节点 resetStatus 成 IDLE，因此第二次遍历对 action 节点是空操作。
    // 如果让 action 节点自己当根，第二次 haltNode 会在目标已终止后再 cancel 一次。
    static constexpr char kXml[] = R"(
      <root BTCPP_format="4">
        <BehaviorTree ID="MainTree">
          <Sequence>
            <SendAtsNavGoal path="{decision_path}"
                            goal_succeeded="{decision_nav_goal_succeeded}"/>
          </Sequence>
        </BehaviorTree>
      </root>)";
    blackboard_ = BT::Blackboard::create();
    blackboard_->set("decision_nav_goal_succeeded", false);
    tree_ = std::make_unique<BT::Tree>(factory_.createTreeFromText(kXml, blackboard_));
  }

  void setPath(const nav_msgs::msg::Path & path) { blackboard_->set("decision_path", path); }

  bool goalSucceeded()
  {
    bool value = false;
    if (!blackboard_->get("decision_nav_goal_succeeded", value)) {
      return false;
    }
    return value;
  }

  /// tick 直到状态不再是 RUNNING，或者超过 `timeout`。
  BT::NodeStatus tickUntilSettled(std::chrono::milliseconds timeout = 3000ms)
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    BT::NodeStatus status = BT::NodeStatus::IDLE;
    do {
      status = tree_->tickExactlyOnce();
      if (status != BT::NodeStatus::RUNNING) {
        return status;
      }
      std::this_thread::sleep_for(10ms);
    } while (std::chrono::steady_clock::now() < deadline);
    return status;
  }

  /// tick 直到服务端收到至少 `count` 个目标（用于确认真的进入了执行中状态）。
  void tickUntilGoalsReceived(int count, std::chrono::milliseconds timeout = 3000ms)
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (server_->goalsReceived() < count && std::chrono::steady_clock::now() < deadline) {
      tree_->tickExactlyOnce();
      std::this_thread::sleep_for(10ms);
    }
  }

  rclcpp::Node::SharedPtr server_node_;
  rclcpp::Node::SharedPtr client_node_;
  std::unique_ptr<ScriptedNavServer> server_;
  std::unique_ptr<rclcpp::executors::MultiThreadedExecutor> server_executor_;
  std::thread server_thread_;
  BT::BehaviorTreeFactory factory_;
  BT::Blackboard::Ptr blackboard_;
  std::unique_ptr<BT::Tree> tree_;
};

}  // namespace

// 单航点拿到 `RESULT_SUCCEEDED`：整条路径完成，`goal_succeeded` 置 true。
TEST_F(SendAtsNavGoalTest, SingleWaypointSucceeds)
{
  server_->setResultCode(AtsNavigateToPose::Result::RESULT_SUCCEEDED);
  makeTree();
  setPath(makePath(1));

  EXPECT_EQ(tickUntilSettled(), BT::NodeStatus::SUCCESS);
  EXPECT_TRUE(goalSucceeded());
  EXPECT_EQ(server_->goalsReceived(), 1);
}

// 非 SUCCEEDED 的 result_code 一律 FAILURE，不做"接近终点"的宽容判定。
TEST_F(SendAtsNavGoalTest, NonSucceededResultCodeIsFailure)
{
  server_->setResultCode(AtsNavigateToPose::Result::RESULT_MAP_UNREADY);
  makeTree();
  setPath(makePath(1));

  EXPECT_EQ(tickUntilSettled(), BT::NodeStatus::FAILURE);
  EXPECT_FALSE(goalSucceeded());
}

TEST_F(SendAtsNavGoalTest, TimeoutResultCodeIsFailure)
{
  server_->setResultCode(AtsNavigateToPose::Result::RESULT_TIMEOUT);
  makeTree();
  setPath(makePath(1));

  EXPECT_EQ(tickUntilSettled(), BT::NodeStatus::FAILURE);
  EXPECT_FALSE(goalSucceeded());
}

// halt 必须让服务端真的收到 cancel 请求，而不是只让本节点返回失败。
TEST_F(SendAtsNavGoalTest, HaltCancelsGoalOnServer)
{
  server_->setHold(true);
  makeTree();
  setPath(makePath(1));

  tickUntilGoalsReceived(1);
  ASSERT_EQ(server_->goalsReceived(), 1);
  ASSERT_EQ(server_->cancelsReceived(), 0);

  tree_->haltTree();

  EXPECT_EQ(server_->cancelsReceived(), 1);
  EXPECT_FALSE(goalSucceeded());
}

// 迟到 result：被取消的那一轮的结果不得污染新一轮，也不得置成终端成功。
TEST_F(SendAtsNavGoalTest, LateResultFromCanceledGoalDoesNotLeak)
{
  server_->setHold(true);
  makeTree();
  setPath(makePath(1));

  tickUntilGoalsReceived(1);
  ASSERT_EQ(server_->goalsReceived(), 1);
  tree_->haltTree();
  ASSERT_EQ(server_->cancelsReceived(), 1);
  ASSERT_FALSE(goalSucceeded());

  // 旧目标的 result 此刻可能还在路上。新一轮换成会成功的服务端行为，
  // 断言新一轮拿到的是自己的 result，且服务端确实收到了第二个目标。
  server_->setHold(false);
  server_->setResultCode(AtsNavigateToPose::Result::RESULT_SUCCEEDED);
  setPath(makePath(1));

  EXPECT_EQ(tickUntilSettled(), BT::NodeStatus::SUCCESS);
  EXPECT_TRUE(goalSucceeded());
  EXPECT_EQ(server_->goalsReceived(), 2);
}

// 多航点：逐点下发，只有全部航点拿到终端成功才输出 goal_succeeded=true。
TEST_F(SendAtsNavGoalTest, MultiWaypointPathAdvancesOnePointPerGoal)
{
  server_->setResultCode(AtsNavigateToPose::Result::RESULT_SUCCEEDED);
  makeTree();
  const auto path = makePath(3);
  setPath(path);

  // 第一个航点完成：SUCCESS，但整条路径还没完。
  EXPECT_EQ(tickUntilSettled(), BT::NodeStatus::SUCCESS);
  EXPECT_FALSE(goalSucceeded());
  EXPECT_EQ(server_->goalsReceived(), 1);

  EXPECT_EQ(tickUntilSettled(), BT::NodeStatus::SUCCESS);
  EXPECT_FALSE(goalSucceeded());
  EXPECT_EQ(server_->goalsReceived(), 2);

  // 第三个航点完成：整条路径完成。
  EXPECT_EQ(tickUntilSettled(), BT::NodeStatus::SUCCESS);
  EXPECT_TRUE(goalSucceeded());
  EXPECT_EQ(server_->goalsReceived(), 3);

  // 路径没变时不得重复下发：已完成的路径直接判成功。
  EXPECT_EQ(tickUntilSettled(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(server_->goalsReceived(), 3);
}

// 中途失败必须清空进度：下一轮从第一个航点重新下发，不复用旧进度。
TEST_F(SendAtsNavGoalTest, FailureMidPathRestartsFromFirstWaypoint)
{
  server_->setResultCode(AtsNavigateToPose::Result::RESULT_SUCCEEDED);
  makeTree();
  setPath(makePath(3));

  ASSERT_EQ(tickUntilSettled(), BT::NodeStatus::SUCCESS);
  ASSERT_EQ(server_->goalsReceived(), 1);

  server_->setResultCode(AtsNavigateToPose::Result::RESULT_PLANNING_FAILED);
  EXPECT_EQ(tickUntilSettled(), BT::NodeStatus::FAILURE);
  EXPECT_FALSE(goalSucceeded());
  ASSERT_EQ(server_->goalsReceived(), 2);

  // 失败已清空进度：同一条路径会从第一个航点重新开始，
  // 因此要再拿 3 次终端成功才算整条路径完成。
  server_->setResultCode(AtsNavigateToPose::Result::RESULT_SUCCEEDED);
  ASSERT_EQ(tickUntilSettled(), BT::NodeStatus::SUCCESS);
  EXPECT_FALSE(goalSucceeded());
  ASSERT_EQ(tickUntilSettled(), BT::NodeStatus::SUCCESS);
  EXPECT_FALSE(goalSucceeded());
  ASSERT_EQ(tickUntilSettled(), BT::NodeStatus::SUCCESS);
  EXPECT_TRUE(goalSucceeded());
  EXPECT_EQ(server_->goalsReceived(), 5);
}

// 执行中换成另一条路径：必须先 cancel 旧目标，再从新路径第一个航点下发。
TEST_F(SendAtsNavGoalTest, PathReplacementCancelsRunningGoal)
{
  server_->setHold(true);
  makeTree();
  setPath(makePath(2));

  tickUntilGoalsReceived(1);
  ASSERT_EQ(server_->goalsReceived(), 1);
  ASSERT_EQ(server_->cancelsReceived(), 0);

  auto other = makePath(2);
  other.poses.front().pose.position.y = 9.0;
  other.poses.back().pose.position.y = 9.0;
  setPath(other);
  tree_->tickExactlyOnce();

  EXPECT_EQ(server_->cancelsReceived(), 1);
  EXPECT_FALSE(goalSucceeded());
}

// 回归：陈旧 goal handle 上的 halt 不得抛异常。
//
// 上游 `BT::RosActionNode` 的 `goal_handle_` 是 private，重新进入 IDLE 时
// 只清 `goal_received_`/`future_goal_handle_`/`result_`，不清 `goal_handle_`。
// 于是"路径替换刚取消了旧目标、新目标的 handle 还没回来"这个窗口里再 halt
// 一次，框架会去 cancel 客户端已经忘记的 handle 并抛
// `UnknownGoalHandleError`。halt 是 `ExecutionCommand STOP -> 串口零速度`
// 这条链的唯一入口，异常逃出去会让这一拍的停车动作整体失效，
// 因此本节点必须吞掉它并保证进度被清空。
TEST_F(SendAtsNavGoalTest, HaltOnStaleGoalHandleDoesNotThrow)
{
  server_->setHold(true);
  makeTree();
  setPath(makePath(2));

  tickUntilGoalsReceived(1);
  ASSERT_EQ(server_->goalsReceived(), 1);

  // 换路径：这一拍内先 cancel 旧目标，再下发新路径的第一个航点。
  auto other = makePath(2);
  other.poses.front().pose.position.y = 9.0;
  setPath(other);
  tree_->tickExactlyOnce();
  ASSERT_EQ(server_->cancelsReceived(), 1);

  // 紧接着 halt：新目标的 handle 尚未就绪，走的正是陈旧 handle 分支。
  ASSERT_NO_THROW(tree_->haltTree());
  EXPECT_FALSE(goalSucceeded());
}

// 服务端进程重启相当于 action server 的新 incarnation。重启前的 goal 不得在
// 新服务端出现后复活；调用 halt 时也不得因旧 handle 抛异常而中断停车路径。
TEST_F(SendAtsNavGoalTest, ServerRestartRejectsStaleGoalIncarnation)
{
  server_->setHold(true);
  makeTree();
  setPath(makePath(1));

  tickUntilGoalsReceived(1);
  ASSERT_EQ(server_->goalsReceived(), 1);

  server_->stopServing();
  ASSERT_NO_THROW(tree_->haltTree());
  EXPECT_FALSE(goalSucceeded());

  server_->startServing(server_node_);
  server_->setHold(false);
  server_->setResultCode(AtsNavigateToPose::Result::RESULT_SUCCEEDED);
  setPath(makePath(1));

  EXPECT_EQ(tickUntilSettled(), BT::NodeStatus::SUCCESS);
  EXPECT_TRUE(goalSucceeded());
  EXPECT_EQ(server_->goalsReceived(), 2);
}

// 空路径：判失败，且不得向服务端下发目标。
TEST_F(SendAtsNavGoalTest, EmptyPathFailsWithoutSendingGoal)
{
  makeTree();
  setPath(nav_msgs::msg::Path());

  EXPECT_EQ(tickUntilSettled(), BT::NodeStatus::FAILURE);
  EXPECT_FALSE(goalSucceeded());
  EXPECT_EQ(server_->goalsReceived(), 0);
}
