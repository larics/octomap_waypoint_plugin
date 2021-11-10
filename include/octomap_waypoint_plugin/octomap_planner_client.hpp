#ifndef OCTOMAP_PLANNER_HPP
#define OCTOMAP_PLANNER_HPP

#include <uav_ros_tracker/planner_interface.hpp>

namespace uav_ros_tracker {

/**
 * @brief Environment aware planner plugin. Uses
 * https://github.com/larics/larics_motion_planning capabailities to plan a collision-free
 * path around an octomap.
 *
 */
class OctomapPlannerClient : public planner_interface
{
public:
  void addWaypoint(const uav_ros_msgs::Waypoint& waypoint) override;
  void addWaypoints(const uav_ros_msgs::Waypoints& waypoints) override;
  void clearWaypoints() override;

  geometry_msgs::PoseArray     getWaypointArray() override;
  uav_ros_msgs::WaypointStatus getWaypointStatus(const nav_msgs::Odometry& odom) override;

  bool initialize(ros::NodeHandle& nh, ros::NodeHandle& nh_private) override;
  std::tuple<bool, std::string, uav_ros_msgs::WaypointPtr> publishWaypoint(
    const nav_msgs::Odometry& current_odometry,
    bool                      tracking_enabled = true,
    bool                      control_enabled  = true) override;
};
}// namespace uav_ros_tracker

#endif /* OCTOMAP_PLANNER_HPP */