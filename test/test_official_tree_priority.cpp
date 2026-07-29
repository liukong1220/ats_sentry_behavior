// Copyright 2026
//
// 官方 RMUC 主树优先级与 halt 传播测试（P6 二.5）。
//
// 这个测试直接加载仓库里真实的 `behavior_trees/rmuc_2026_mapping.xml`，
// 只把叶子节点换成可编程的桩节点，因此它验证的是官方树本身的结构约束，
// 不是测试里另写一份等价树。覆盖 XML 头部注释声称的四件事：
//
//  1. 开局门 `IsGameStart` 判 FAILURE 时，导航分支根本不会被 tick；
//  2. 导航正在 RUNNING 时开局门翻成 FAILURE，`SendAtsNavGoal` 必须收到 halt
//     （halt 里才有 cancelGoal -> `ExecutionCommand STOP` -> 串口零速度）；
//  3. 视觉接管子树优先于裁判决策子树，且抢占时会 halt 正在跑的导航；
//  4. 巡逻游标只能在 `SendAtsNavGoal` 返回终端成功之后才推进。
//
// 桩节点不依赖 ROS 通信：本测试只判定控制流，不判定 action 语义。
// action 语义（cancel、迟到 result、result_code 映射）在
// `test_send_ats_nav_goal_action.cpp` 与 `test_ats_nav_goal_logic.cpp` 里覆盖。

#include <gtest/gtest.h>

#include <map>
#include <string>

#include "behaviortree_cpp/bt_factory.h"

namespace
{

/// 每个桩节点实例的脚本：返回什么状态，被 tick 过几次，被 halt 过几次。
struct StubScript
{
  BT::NodeStatus status = BT::NodeStatus::SUCCESS;
  int tick_count = 0;
  int halt_count = 0;
};

/// 按节点实例名索引脚本。官方树里 `IsGameStatus` 出现两次，
/// 分别叫 `IsGameStart` 与 `IsMappingEndgame`，所以必须按实例名区分。
class StubBook
{
public:
  static StubBook & instance()
  {
    static StubBook book;
    return book;
  }

  void reset() { scripts_.clear(); }

  void setStatus(const std::string & name, BT::NodeStatus status)
  {
    scripts_[name].status = status;
  }

  BT::NodeStatus onTick(const std::string & name)
  {
    auto & script = scripts_[name];
    ++script.tick_count;
    return script.status;
  }

  void onHalt(const std::string & name) { ++scripts_[name].halt_count; }

  int tickCount(const std::string & name) { return scripts_[name].tick_count; }
  int haltCount(const std::string & name) { return scripts_[name].halt_count; }

private:
  std::map<std::string, StubScript> scripts_;
};

/// 同步桩：一次 tick 立即返回脚本状态，用于所有条件节点与非导航动作节点。
class StubSync : public BT::SyncActionNode
{
public:
  StubSync(const std::string & name, const BT::NodeConfig & conf) : BT::SyncActionNode(name, conf)
  {
  }

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override { return StubBook::instance().onTick(name()); }
};

/// 异步桩：用于 `SendAtsNavGoal`。可以停在 RUNNING，并记录 halt 次数，
/// 这样"抢占是否真的 halt 了导航"就变成可断言的事实而不是推断。
class StubAsync : public BT::StatefulActionNode
{
public:
  StubAsync(const std::string & name, const BT::NodeConfig & conf)
  : BT::StatefulActionNode(name, conf)
  {
  }

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus onStart() override { return StubBook::instance().onTick(name()); }
  BT::NodeStatus onRunning() override { return StubBook::instance().onTick(name()); }
  void onHalted() override { StubBook::instance().onHalt(name()); }
};

/// 把官方树里所有自研叶子节点注册成桩，端口一律用"接受任意属性"的方式绕过：
/// 桩节点声明 0 个端口，XML 里的多余属性会被 BehaviorTree.CPP 当成错误，
/// 因此这里逐个节点声明它在官方树里实际用到的属性名。
void registerStubs(BT::BehaviorTreeFactory & factory)
{
  auto sync_with_ports = [&factory](const std::string & id, const BT::PortsList & ports) {
    factory.registerBuilder(
      BT::CreateManifest<StubSync>(id, ports),
      [](const std::string & name, const BT::NodeConfig & conf) {
        return std::make_unique<StubSync>(name, conf);
      });
  };
  auto in = [](const char * key) { return BT::InputPort<std::string>(key); };
  auto out = [](const char * key) { return BT::OutputPort<std::string>(key); };

  sync_with_ports("IsAttacked", {in("key_port")});
  sync_with_ports("IsDecisionInputSource", {in("source")});
  sync_with_ports("IsFriendlyOutpostDestroyed", {in("all_robot_hp"), in("robot_status")});
  sync_with_ports(
    "IsGameStatus",
    {in("expected_game_progress"), in("key_port"), in("max_remain_time"), in("min_remain_time")});
  sync_with_ports("IsRobotResourceMode", {in("robot_status"), in("state")});
  sync_with_ports("IsRobotResupplyRequired", {in("robot_status")});
  sync_with_ports(
    "IsVisionTargetValid", {in("key_port"), in("require_nav_hold"), in("timeout_s"),
                            out("gimbal_yaw"), out("gimbal_pitch"), out("target_id")});
  sync_with_ports(
    "AdvancePatrolCursor", {in("next_cursor"), in("next_direction"), in("goal_succeeded"),
                            out("patrol_cursor"), out("patrol_direction")});
  sync_with_ports(
    "LoadCsvWaypoints",
    {in("csv_file"), in("patrol_mode"), in("patrol_cursor"), in("patrol_direction"), out("path"),
     out("next_cursor"), out("next_direction")});
  sync_with_ports(
    "PublishGimbalAbsolute",
    {in("duration"), in("gimbal_pitch"), in("gimbal_yaw"), in("topic_name")});
  sync_with_ports("PublishRobotMode", {in("duration"), in("mode"), in("topic_name")});
  sync_with_ports("PublishSpinSpeed", {in("duration"), in("spin_speed"), in("topic_name")});
  sync_with_ports("SelectSupplyPoint", {out("path")});
  sync_with_ports(
    "SelectVisionFollowPath",
    {in("current_costmap"), in("current_pose"), in("key_port"), out("path")});

  // 唯一的异步桩：官方树的统一导航执行接口。
  factory.registerBuilder(
    BT::CreateManifest<StubAsync>(
      "SendAtsNavGoal",
      {BT::InputPort<std::string>("path"), BT::OutputPort<std::string>("goal_succeeded")}),
    [](const std::string & name, const BT::NodeConfig & conf) {
      return std::make_unique<StubAsync>(name, conf);
    });
}

std::string officialTreePath()
{
  return std::string(ROOT_DIR) + "behavior_trees/rmuc_2026_mapping.xml";
}

/// 官方树的全部默认值都来自 `@` 前缀的全局黑板项，测试里补齐即可，
/// 数值本身不影响控制流断言（桩节点不读它们的语义）。
BT::Tree makeOfficialTree(BT::BehaviorTreeFactory & factory)
{
  auto blackboard = BT::Blackboard::create();
  for (const char * key :
       {"referee_gameStatus", "referee_robotStatus", "referee_allRobotHP", "sp_vision_target",
        "decision_robot_mode_topic", "decision_spin_topic", "decision_gimbal_topic",
        "decision_vision_timeout_s", "decision_hit_spin_speed", "decision_default_spin_speed",
        "decision_game_start_max_remain_time", "decision_game_start_min_remain_time",
        "decision_game_start_expected_progress", "decision_rmuc_csv_file",
        "decision_rmuc_patrol_csv_file", "decision_rmuc_endgame_time_threshold",
        "decision_current_pose", "nav_globalCostmap"}) {
    blackboard->set(key, std::string("stub"));
  }
  factory.registerBehaviorTreeFromFile(officialTreePath());
  return factory.createTree("rmuc_2026_mapping", blackboard);
}

class OfficialTreePriority : public ::testing::Test
{
protected:
  void SetUp() override
  {
    StubBook::instance().reset();
    registerStubs(factory_);
  }

  BT::BehaviorTreeFactory factory_;
};

}  // namespace

// 开局门 FAILURE 时，`ReactiveSequence` 必须在第一个子节点就短路：
// 导航节点一次都不能被 tick，否则实车会在开局信号未到时就拿到执行授权。
TEST_F(OfficialTreePriority, GameStartGateBlocksNavigationBranch)
{
  auto & book = StubBook::instance();
  book.setStatus("IsGameStart", BT::NodeStatus::FAILURE);
  auto tree = makeOfficialTree(factory_);

  tree.tickExactlyOnce();

  EXPECT_EQ(book.tickCount("IsGameStart"), 1);
  // 开局未到：只允许发姿态与自旋（在 ForceFailure 里），不允许下发导航目标。
  EXPECT_EQ(book.tickCount("SendAtsNavGoal"), 0);
  EXPECT_GE(book.tickCount("PublishRobotMode"), 1);
}

// 导航已经 RUNNING 后开局门翻成 FAILURE：`SendAtsNavGoal` 必须被 halt。
// halt 是 cancelGoal 的唯一入口，也是
// `ExecutionCommand STOP -> emergency_stop=true -> /cmd_vel_mpc=0 ->
// /motion_control=0 -> 串口零速度` 这条链的起点。
TEST_F(OfficialTreePriority, GameStartGateFlipHaltsRunningNavigation)
{
  auto & book = StubBook::instance();
  book.setStatus("IsGameStart", BT::NodeStatus::SUCCESS);
  book.setStatus("IsRobotResourceMode", BT::NodeStatus::FAILURE);  // 关掉视觉接管与 defend
  book.setStatus("IsVisionTargetValid", BT::NodeStatus::FAILURE);
  book.setStatus("IsRobotResupplyRequired", BT::NodeStatus::FAILURE);
  book.setStatus("IsMappingEndgame", BT::NodeStatus::FAILURE);
  book.setStatus("IsFriendlyOutpostDestroyed", BT::NodeStatus::FAILURE);
  book.setStatus("SendAtsNavGoal", BT::NodeStatus::RUNNING);
  auto tree = makeOfficialTree(factory_);

  tree.tickExactlyOnce();
  ASSERT_GE(book.tickCount("SendAtsNavGoal"), 1);
  ASSERT_EQ(book.haltCount("SendAtsNavGoal"), 0);

  book.setStatus("IsGameStart", BT::NodeStatus::FAILURE);
  tree.tickExactlyOnce();

  EXPECT_EQ(book.haltCount("SendAtsNavGoal"), 1);
}

// 视觉接管子树在 `ReactiveFallback` 的第一位：条件成立时裁判决策分支不得被 tick。
TEST_F(OfficialTreePriority, VisionOverrideTakesPriorityOverRefereeBranch)
{
  auto & book = StubBook::instance();
  book.setStatus("IsGameStart", BT::NodeStatus::SUCCESS);
  book.setStatus("IsRobotResourceMode", BT::NodeStatus::SUCCESS);
  book.setStatus("IsVisionTargetValid", BT::NodeStatus::SUCCESS);
  book.setStatus("SendAtsNavGoal", BT::NodeStatus::RUNNING);
  auto tree = makeOfficialTree(factory_);

  tree.tickExactlyOnce();

  EXPECT_GE(book.tickCount("SelectVisionFollowPath"), 1);
  EXPECT_GE(book.tickCount("SendAtsNavGoal"), 1);
  // 视觉分支 RUNNING 时，裁判决策链的入口节点一次都不该被 tick。
  EXPECT_EQ(book.tickCount("IsRobotResupplyRequired"), 0);
  EXPECT_EQ(book.tickCount("LoadCsvWaypoints"), 0);
}

// 视觉接管在裁判导航 RUNNING 期间抬头：`ReactiveFallback` 必须 halt 后面的分支，
// 导航目标才会真的被取消，而不是留在目标管理器里继续执行旧授权。
TEST_F(OfficialTreePriority, VisionOverridePreemptsRunningRefereeNavigation)
{
  auto & book = StubBook::instance();
  book.setStatus("IsGameStart", BT::NodeStatus::SUCCESS);
  book.setStatus("IsRobotResourceMode", BT::NodeStatus::FAILURE);
  book.setStatus("IsVisionTargetValid", BT::NodeStatus::FAILURE);
  book.setStatus("IsRobotResupplyRequired", BT::NodeStatus::FAILURE);
  book.setStatus("IsMappingEndgame", BT::NodeStatus::FAILURE);
  book.setStatus("IsFriendlyOutpostDestroyed", BT::NodeStatus::FAILURE);
  book.setStatus("SendAtsNavGoal", BT::NodeStatus::RUNNING);
  auto tree = makeOfficialTree(factory_);

  tree.tickExactlyOnce();
  ASSERT_GE(book.tickCount("LoadCsvWaypoints"), 1);
  ASSERT_EQ(book.haltCount("SendAtsNavGoal"), 0);

  // 视觉抬头。`IsRobotResourceMode` 同时也是 `rmuc_publish_navigation_mode` 的
  // defend 判据，这里让它成立不影响本断言：defend 分支不含导航节点。
  book.setStatus("IsRobotResourceMode", BT::NodeStatus::SUCCESS);
  book.setStatus("IsVisionTargetValid", BT::NodeStatus::SUCCESS);
  tree.tickExactlyOnce();

  EXPECT_GE(book.haltCount("SendAtsNavGoal"), 1);
}

// 巡逻游标只能被 ATS action 的终端成功驱动：`SendAtsNavGoal` 仍 RUNNING 时，
// `AdvancePatrolCursor` 不得被 tick（否则会用位置容差之外的东西提前推进航点）。
TEST_F(OfficialTreePriority, PatrolCursorAdvancesOnlyAfterTerminalSuccess)
{
  auto & book = StubBook::instance();
  book.setStatus("IsGameStart", BT::NodeStatus::SUCCESS);
  book.setStatus("IsRobotResourceMode", BT::NodeStatus::FAILURE);
  book.setStatus("IsVisionTargetValid", BT::NodeStatus::FAILURE);
  book.setStatus("IsRobotResupplyRequired", BT::NodeStatus::FAILURE);
  book.setStatus("IsMappingEndgame", BT::NodeStatus::SUCCESS);  // 走 patrol 分支
  book.setStatus("SendAtsNavGoal", BT::NodeStatus::RUNNING);
  auto tree = makeOfficialTree(factory_);

  tree.tickExactlyOnce();
  ASSERT_GE(book.tickCount("SendAtsNavGoal"), 1);
  EXPECT_EQ(book.tickCount("AdvancePatrolCursor"), 0);

  book.setStatus("SendAtsNavGoal", BT::NodeStatus::SUCCESS);
  tree.tickExactlyOnce();

  EXPECT_GE(book.tickCount("AdvancePatrolCursor"), 1);
}
