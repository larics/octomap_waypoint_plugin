#ifndef OCTOMAP_PLANNER_HPP
#define OCTOMAP_PLANNER_HPP

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TransformStamped.h"
#include "ros/forwards.h"
#include "ros/service_client.h"
#include "ros/subscriber.h"
#include <uav_ros_tracker/planner_interface.hpp>
#include <deque>
#include <mutex>

namespace uav_ros_tracker {

/**
 * @brief Environment aware planner plugin. Uses
 * https://github.com/larics/larixcs_motion_planning capabailities to plan a
 * collision-free path around an octomap.
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

  bool initialize(
    ros::NodeHandle&                                                 nh,
    ros::NodeHandle&                                                 nh_private,
    std::unordered_map<std::string, geometry_msgs::TransformStamped> transform_map,
    std::string tracking_frame) override;

  std::tuple<bool, std::string, uav_ros_msgs::WaypointPtr> publishWaypoint(
    const nav_msgs::Odometry& current_odometry,
    bool                      tracking_enabled = true,
    bool                      control_enabled  = true) override;

private:
  std::optional<uav_ros_msgs::WaypointPtr> getCurrentWaypoint();
  double                distanceToCurrentWp(const nav_msgs::Odometry& odom);
  static constexpr auto NAME         = "OctomapPlannerClient";
  static constexpr auto DISTANCE_TOL = 0.5;

  ros::Timer m_waiting_timer;
  void       waiting_callback(const ros::TimerEvent& e);

  ros::Timer m_planning_timer;
  void       plannning_callback(const ros::TimerEvent& e);

  geometry_msgs::PoseStamped m_carrot_pose;
  std::mutex                 m_carrot_pose_mutex;
  ros::Subscriber            m_carrot_pose_sub;
  void                       carrot_pose_cb(const geometry_msgs::PoseStamped& pose);

  bool                                                             m_is_flying  = false;
  bool                                                             m_is_waiting = false;
  std::unordered_map<std::string, geometry_msgs::TransformStamped> m_transform_map;
  std::string                                                      m_tracking_frame;

  std::mutex                            m_waypoint_buffer_mutex;
  std::deque<uav_ros_msgs::WaypointPtr> m_waypoint_buffer;

  ros::ServiceClient m_planner_client;
  ros::Publisher     m_tracker_trajectory_pub;
};
}// namespace uav_ros_tracker

#endif /* OCTOMAP_PLANNER_HPP */
