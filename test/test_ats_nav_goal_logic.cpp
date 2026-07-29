// Copyright 2026
//
// ATS 导航 action 判据与航点状态机测试（P6 二.1/二.5）。覆盖：
//  - result_code 映射：只有 RESULT_SUCCEEDED 是终端成功
//  - 未知 result_code 一律按失败处理
//  - 航点状态机：同一路径不重置进度，换路径必须重启
//  - 逐点推进：只有全部航点成功才算整条路径完成
//  - 失败与 halt 清空进度，下一轮从第一个航点重新下发

#include <gtest/gtest.h>

#include "ats_sentry_behavior/ats_nav_goal_logic.hpp"

using ats_sentry_behavior::decision::AtsNavOutcome;
using ats_sentry_behavior::decision::atsNavOutcomeFromResultCode;
using ats_sentry_behavior::decision::atsNavOutcomeIsSuccess;
using ats_sentry_behavior::decision::AtsWaypointSequencer;
using ats_sentry_behavior::decision::AtsWaypointUpdate;
using AtsResult = ats_navigation_interfaces::action::NavigateToPose::Result;

namespace
{

nav_msgs::msg::Path makePath(const std::vector<std::pair<double, double>> & points)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  for (const auto & point : points) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.pose.position.x = point.first;
    pose.pose.position.y = point.second;
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }
  return path;
}

}  // namespace

TEST(AtsNavOutcome, OnlySucceededIsTerminalSuccess)
{
  EXPECT_TRUE(atsNavOutcomeIsSuccess(atsNavOutcomeFromResultCode(AtsResult::RESULT_SUCCEEDED)));
  for (const uint8_t code :
       {AtsResult::RESULT_CANCELED, AtsResult::RESULT_PREEMPTED, AtsResult::RESULT_TIMEOUT,
        AtsResult::RESULT_MAP_UNREADY, AtsResult::RESULT_PLANNING_FAILED,
        AtsResult::RESULT_TF_FAILED, AtsResult::RESULT_INTERNAL_ERROR}) {
    EXPECT_FALSE(atsNavOutcomeIsSuccess(atsNavOutcomeFromResultCode(code)))
      << "result_code=" << static_cast<unsigned>(code) << " 不得被判成终端成功";
  }
}

TEST(AtsNavOutcome, ResultCodeMappingIsExact)
{
  EXPECT_EQ(atsNavOutcomeFromResultCode(AtsResult::RESULT_CANCELED), AtsNavOutcome::kCanceled);
  EXPECT_EQ(atsNavOutcomeFromResultCode(AtsResult::RESULT_PREEMPTED), AtsNavOutcome::kPreempted);
  EXPECT_EQ(atsNavOutcomeFromResultCode(AtsResult::RESULT_TIMEOUT), AtsNavOutcome::kTimeout);
  EXPECT_EQ(atsNavOutcomeFromResultCode(AtsResult::RESULT_MAP_UNREADY), AtsNavOutcome::kMapUnready);
  EXPECT_EQ(
    atsNavOutcomeFromResultCode(AtsResult::RESULT_PLANNING_FAILED), AtsNavOutcome::kPlanningFailed);
  EXPECT_EQ(atsNavOutcomeFromResultCode(AtsResult::RESULT_TF_FAILED), AtsNavOutcome::kTfFailed);
  EXPECT_EQ(
    atsNavOutcomeFromResultCode(AtsResult::RESULT_INTERNAL_ERROR), AtsNavOutcome::kInternalError);
}

TEST(AtsNavOutcome, UnknownResultCodeIsFailure)
{
  // 服务端将来新增 result_code 时，行为层必须按失败处理，不得猜测语义。
  const auto outcome = atsNavOutcomeFromResultCode(200);
  EXPECT_EQ(outcome, AtsNavOutcome::kUnknown);
  EXPECT_FALSE(atsNavOutcomeIsSuccess(outcome));
}

TEST(AtsWaypointSequencer, EmptyPathIsInvalid)
{
  AtsWaypointSequencer sequencer(0.25);
  EXPECT_EQ(sequencer.updatePath(nav_msgs::msg::Path()), AtsWaypointUpdate::kRestarted);
  EXPECT_FALSE(sequencer.valid());
  EXPECT_FALSE(sequencer.finished());
}

TEST(AtsWaypointSequencer, SamePathDoesNotResetProgress)
{
  AtsWaypointSequencer sequencer(0.25);
  const auto path = makePath({{1.0, 0.0}, {2.0, 0.0}, {3.0, 0.0}});
  ASSERT_EQ(sequencer.updatePath(path), AtsWaypointUpdate::kRestarted);
  sequencer.onWaypointSucceeded();
  ASSERT_EQ(sequencer.index(), 1U);

  // 容差内的同一条路径重复下发：进度必须保持，否则实车会反复抢占同一目标。
  const auto jittered = makePath({{1.05, 0.0}, {2.05, 0.0}, {3.05, 0.0}});
  EXPECT_EQ(sequencer.updatePath(jittered), AtsWaypointUpdate::kUnchanged);
  EXPECT_EQ(sequencer.index(), 1U);
  EXPECT_DOUBLE_EQ(sequencer.currentWaypoint().pose.position.x, 2.05);
}

TEST(AtsWaypointSequencer, DifferentPathRestartsFromFirstWaypoint)
{
  AtsWaypointSequencer sequencer(0.25);
  ASSERT_EQ(
    sequencer.updatePath(makePath({{1.0, 0.0}, {2.0, 0.0}})), AtsWaypointUpdate::kRestarted);
  sequencer.onWaypointSucceeded();
  ASSERT_EQ(sequencer.index(), 1U);

  EXPECT_EQ(
    sequencer.updatePath(makePath({{-4.0, 1.0}, {-5.0, 1.0}})), AtsWaypointUpdate::kRestarted);
  EXPECT_EQ(sequencer.index(), 0U);
  EXPECT_FALSE(sequencer.finished());
  EXPECT_DOUBLE_EQ(sequencer.currentWaypoint().pose.position.x, -4.0);
}

TEST(AtsWaypointSequencer, DifferentLengthIsNotEquivalent)
{
  AtsWaypointSequencer sequencer(0.25);
  ASSERT_EQ(
    sequencer.updatePath(makePath({{1.0, 0.0}, {2.0, 0.0}})), AtsWaypointUpdate::kRestarted);
  EXPECT_EQ(sequencer.updatePath(makePath({{1.0, 0.0}})), AtsWaypointUpdate::kRestarted);
  EXPECT_EQ(sequencer.size(), 1U);
}

TEST(AtsWaypointSequencer, FinishesOnlyAfterEveryWaypointSucceeds)
{
  AtsWaypointSequencer sequencer(0.25);
  ASSERT_EQ(
    sequencer.updatePath(makePath({{1.0, 0.0}, {2.0, 0.0}, {3.0, 0.0}})),
    AtsWaypointUpdate::kRestarted);

  sequencer.onWaypointSucceeded();
  EXPECT_FALSE(sequencer.finished());
  EXPECT_DOUBLE_EQ(sequencer.currentWaypoint().pose.position.x, 2.0);

  sequencer.onWaypointSucceeded();
  EXPECT_FALSE(sequencer.finished());
  EXPECT_DOUBLE_EQ(sequencer.currentWaypoint().pose.position.x, 3.0);

  sequencer.onWaypointSucceeded();
  EXPECT_TRUE(sequencer.finished());
  // 已完成后再收到成功不得越界。
  sequencer.onWaypointSucceeded();
  EXPECT_EQ(sequencer.index(), 3U);
  EXPECT_TRUE(sequencer.finished());
}

TEST(AtsWaypointSequencer, FailureClearsProgress)
{
  AtsWaypointSequencer sequencer(0.25);
  const auto path = makePath({{1.0, 0.0}, {2.0, 0.0}});
  ASSERT_EQ(sequencer.updatePath(path), AtsWaypointUpdate::kRestarted);
  sequencer.onWaypointSucceeded();
  ASSERT_EQ(sequencer.index(), 1U);

  // 任何非 RESULT_SUCCEEDED 结局与 halt 都走这条路径：清空进度。
  sequencer.onFailure();
  EXPECT_FALSE(sequencer.valid());
  EXPECT_EQ(sequencer.index(), 0U);
  // 同一条路径重新出现时必须被当成新路径，从第一个航点重新下发。
  EXPECT_EQ(sequencer.updatePath(path), AtsWaypointUpdate::kRestarted);
  EXPECT_DOUBLE_EQ(sequencer.currentWaypoint().pose.position.x, 1.0);
}

TEST(AtsWaypointSequencer, FinishedPathStaysFinishedUntilPathChanges)
{
  AtsWaypointSequencer sequencer(0.25);
  const auto path = makePath({{1.0, 0.0}});
  ASSERT_EQ(sequencer.updatePath(path), AtsWaypointUpdate::kRestarted);
  sequencer.onWaypointSucceeded();
  ASSERT_TRUE(sequencer.finished());

  // 同一条路径继续被 tick：仍然是完成态，不得重新下发目标。
  EXPECT_EQ(sequencer.updatePath(path), AtsWaypointUpdate::kUnchanged);
  EXPECT_TRUE(sequencer.finished());
  EXPECT_TRUE(sequencer.matches(path));

  EXPECT_EQ(sequencer.updatePath(makePath({{9.0, 9.0}})), AtsWaypointUpdate::kRestarted);
  EXPECT_FALSE(sequencer.finished());
}

TEST(AtsWaypointSequencer, HoldToleranceIsConfigurable)
{
  AtsWaypointSequencer sequencer(0.05);
  ASSERT_EQ(sequencer.updatePath(makePath({{1.0, 0.0}})), AtsWaypointUpdate::kRestarted);
  // 0.1 m 偏移超过 0.05 m 容差：视为新路径。
  EXPECT_EQ(sequencer.updatePath(makePath({{1.1, 0.0}})), AtsWaypointUpdate::kRestarted);
  sequencer.setHoldTolerance(0.25);
  EXPECT_EQ(sequencer.updatePath(makePath({{1.2, 0.0}})), AtsWaypointUpdate::kUnchanged);
}
