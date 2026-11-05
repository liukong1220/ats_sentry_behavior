#ifndef ATS_SENTRY_BEHAVIOR__PLANNING_SNAPSHOT_GRID_HPP_
#define ATS_SENTRY_BEHAVIOR__PLANNING_SNAPSHOT_GRID_HPP_

#include <cstddef>

#include "ats_navigation_interfaces/msg/planning_map_snapshot.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace ats_sentry_behavior
{

/// Blackboard view of the planning-map owner's numeric snapshot.
///
/// The grid keeps the occupancy encoding of the former `/rc_esdf/planning_grid`
/// (-1 unknown, 0..99 terrain risk, >= threshold occupied) and its stamp is the
/// snapshot's source stamp, so stale-sample checks keep their meaning. A
/// snapshot that is not ready, or whose occupancy does not match its geometry,
/// becomes an all-unknown grid of the same geometry: the behavior layer must
/// never see an old traversable map once the owner reports not-ready.
inline nav_msgs::msg::OccupancyGrid planningSnapshotToGrid(
  const ats_navigation_interfaces::msg::PlanningMapSnapshot & snapshot)
{
  nav_msgs::msg::OccupancyGrid grid;
  grid.header.frame_id = snapshot.header.frame_id;
  grid.header.stamp = snapshot.source_stamp;
  grid.info = snapshot.info;
  const std::size_t cells =
    static_cast<std::size_t>(snapshot.info.width) * static_cast<std::size_t>(snapshot.info.height);
  if (snapshot.ready && snapshot.occupancy.size() == cells) {
    grid.data = snapshot.occupancy;
  } else {
    grid.data.assign(cells, -1);
  }
  return grid;
}

}  // namespace ats_sentry_behavior

#endif  // ATS_SENTRY_BEHAVIOR__PLANNING_SNAPSHOT_GRID_HPP_
