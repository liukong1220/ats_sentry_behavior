#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

#include "ats_sentry_behavior/planning_snapshot_grid.hpp"

namespace
{

using ats_navigation_interfaces::msg::PlanningMapSnapshot;

PlanningMapSnapshot makeSnapshot(bool ready)
{
  PlanningMapSnapshot snapshot;
  snapshot.header.frame_id = "map";
  snapshot.header.stamp.sec = 20;
  snapshot.source_stamp.sec = 17;
  snapshot.source_stamp.nanosec = 5;
  snapshot.ready = ready;
  snapshot.info.width = 3;
  snapshot.info.height = 2;
  snapshot.info.resolution = 0.1F;
  snapshot.info.origin.position.x = -1.5;
  snapshot.occupancy = {0, 100, -1, 30, 0, 0};
  return snapshot;
}

TEST(PlanningSnapshotGrid, ReadySnapshotKeepsOccupancyGeometryAndSourceStamp)
{
  const auto grid = ats_sentry_behavior::planningSnapshotToGrid(makeSnapshot(true));
  EXPECT_EQ(grid.header.frame_id, "map");
  EXPECT_EQ(grid.header.stamp.sec, 17);
  EXPECT_EQ(grid.header.stamp.nanosec, 5U);
  EXPECT_EQ(grid.info.width, 3U);
  EXPECT_DOUBLE_EQ(grid.info.origin.position.x, -1.5);
  EXPECT_EQ(grid.data, (std::vector<std::int8_t>{0, 100, -1, 30, 0, 0}));
}

TEST(PlanningSnapshotGrid, NotReadySnapshotBecomesAllUnknown)
{
  // A blocked-unavailable snapshot still carries the last occupancy for audit;
  // the behavior layer must not treat it as traversable.
  const auto grid = ats_sentry_behavior::planningSnapshotToGrid(makeSnapshot(false));
  EXPECT_EQ(grid.data, std::vector<std::int8_t>(6, -1));
}

TEST(PlanningSnapshotGrid, MismatchedOccupancyBecomesAllUnknown)
{
  auto snapshot = makeSnapshot(true);
  snapshot.occupancy.pop_back();
  const auto grid = ats_sentry_behavior::planningSnapshotToGrid(snapshot);
  EXPECT_EQ(grid.data, std::vector<std::int8_t>(6, -1));
}

}  // namespace
