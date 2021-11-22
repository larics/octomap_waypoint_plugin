#include "geometry_msgs/PoseStamped.h"
#include "larics_motion_planning/MultiDofTrajectoryRequest.h"
#include "ros/duration.h"
#include "ros/forwards.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/convert.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "uav_ros_msgs/Waypoint.h"
#include <ios>
#include <mutex>
#include <octomap_waypoint_plugin/octomap_planner_client.hpp>
#include <larics_motion_planning/MultiDofTrajectory.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

void uav_ros_tracker::OctomapPlannerClient::addWaypoint(
  const uav_ros_msgs::Waypoint& waypoint)
{
  std::lock_guard<std::mutex> lock(m_waypoint_buffer_mutex);
  m_waypoint_buffer.emplace_back(boost::make_shared<uav_ros_msgs::Waypoint>(waypoint));

  ROS_INFO("[%s] Waypoint Added [%.2f, %.2f, %.2f]",
           NAME,
           waypoint.pose.pose.position.x,
           waypoint.pose.pose.position.y,
           waypoint.pose.pose.position.z);
}

void uav_ros_tracker::OctomapPlannerClient::addWaypoints(
  const uav_ros_msgs::Waypoints& waypoints)
{
  for (const auto& wp : waypoints.waypoints) { addWaypoint(wp); }
}

void uav_ros_tracker::OctomapPlannerClient::clearWaypoints()
{
  std::lock_guard<std::mutex> lock(m_waypoint_buffer_mutex);
  m_waypoint_buffer.clear();
}

geometry_msgs::PoseArray uav_ros_tracker::OctomapPlannerClient::getWaypointArray()
{
  geometry_msgs::PoseArray poseArray;

  {
    std::lock_guard<std::mutex> lock(m_waypoint_buffer_mutex);
    for (const auto& waypoint : m_waypoint_buffer) {
      auto transformed_wp =
        transform_waypoint(*waypoint, m_transform_map, m_tracking_frame);
      poseArray.poses.push_back(transformed_wp.pose.pose);
    }
  }

  return poseArray;
}

uav_ros_msgs::WaypointStatus uav_ros_tracker::OctomapPlannerClient::getWaypointStatus(
  const nav_msgs::Odometry& odom)
{
  uav_ros_msgs::WaypointStatus waypointStatus;
  waypointStatus.flying_to_wp  = m_is_flying;
  waypointStatus.waiting_at_wp = m_is_waiting;
  return waypointStatus;
}

bool uav_ros_tracker::OctomapPlannerClient::initialize(
  ros::NodeHandle&                                                 nh,
  ros::NodeHandle&                                                 nh_private,
  std::unordered_map<std::string, geometry_msgs::TransformStamped> transform_map,
  std::string                                                      tracking_frame)
{
  m_tracking_frame = std::move(tracking_frame);
  m_transform_map  = std::move(transform_map);
  m_planner_client =
    nh.serviceClient<larics_motion_planning::MultiDofTrajectory>("/uav/multi_dof_trajectory");
  m_tracker_trajectory_pub =
    nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>("tracker/input_trajectory", 1);

  m_waiting_timer = nh.createTimer(ros::Duration(1),
                                   &OctomapPlannerClient::waiting_callback,
                                   this,
                                   true /* oneshot */,
                                   false /*autostart */);

  m_planning_timer = nh.createTimer(ros::Duration(0.01),
                                    &OctomapPlannerClient::plannning_callback,
                                    this,
                                    true /* oneshot */,
                                    false /*autostart */);
  return true;
}

std::tuple<bool, std::string, uav_ros_msgs::WaypointPtr>
  uav_ros_tracker::OctomapPlannerClient::publishWaypoint(
    const nav_msgs::Odometry& current_odometry,
    bool                      tracking_enabled,
    bool                      control_enabled)
{

  if (!control_enabled) {
    // reset();
    return std::make_tuple<bool, std::string, uav_ros_msgs::WaypointPtr>(
      false, "UAV off-board control is disabled.", {});
  }

  // Get the latest waypoint
  uav_ros_msgs::WaypointPtr current_waypoint_ptr;
  {
    std::lock_guard<std::mutex> lock(m_waypoint_buffer_mutex);

    if (m_waypoint_buffer.empty()) {
      // reset();
      return std::make_tuple<bool, std::string, uav_ros_msgs::WaypointPtr>(
        false, "No waypoints available.", {});
    }

    current_waypoint_ptr = m_waypoint_buffer.front();
  }

  if (current_waypoint_ptr == nullptr) {
    return std::make_tuple<bool, std::string, uav_ros_msgs::WaypointPtr>(
      false, "No waypoints available.", {});
  }

  auto distance_to_wp = this->calc_distance(current_odometry, *current_waypoint_ptr);
  // If we are are flying and still haven't reached the distance
  if (m_is_flying && distance_to_wp >= DISTANCE_TOL) {
    return std::make_tuple(false, "Flying to current waypoint!", current_waypoint_ptr);
  }

  // We have reached the desired waypoint distance, start the waiting
  if (m_is_flying && !m_is_waiting && distance_to_wp < DISTANCE_TOL) {
    m_is_waiting = true;
    m_is_flying  = false;
    m_waiting_timer.setPeriod(ros::Duration(current_waypoint_ptr->waiting_time), true);
    m_waiting_timer.start();
    return std::make_tuple(
      false, "Started waiting at the current waypoint!", current_waypoint_ptr);
  }

  // If we are not flying but rather waiting than
  if (m_is_waiting) {
    return std::make_tuple(
      false, "Waiting at the current waypoint!", current_waypoint_ptr);
  }

  // Check if tracker is available
  if (!tracking_enabled) {
    // reset();
    return std::make_tuple<bool, std::string, uav_ros_msgs::WaypointPtr>(
      false, "Tracker is busy!", {});
  }

  if (!m_planning_timer.hasPending()) {
    m_planning_timer.setPeriod(ros::Duration(0.01), true);
  }

  m_planning_timer.start();
  return { false, "Not implemented!", nullptr };
}

void uav_ros_tracker::OctomapPlannerClient::waiting_callback(const ros::TimerEvent& e)
{
  ROS_INFO("[%s] Waiting at waypoint finished!", NAME);
  m_is_waiting = false;

  {
    std::lock_guard<std::mutex> lock(m_waypoint_buffer_mutex);
    m_waypoint_buffer.pop_front();
  }
}

void uav_ros_tracker::OctomapPlannerClient::plannning_callback(const ros::TimerEvent& e)
{
  ROS_INFO_THROTTLE(2.0, "[%s::planning_callback]", NAME);

  if (!m_planner_client.exists()) {
    ROS_WARN_THROTTLE(2.0, "[%s::planning_callback] Planning service is missing!", NAME);
    return;
  }

  // Get the current carrot pose
  geometry_msgs::PoseStamped current_carrot_pose;
  {
    std::lock_guard<std::mutex> lock(m_carrot_pose_mutex);
    current_carrot_pose = m_carrot_pose;
  }

  // Get the current waypoint
  uav_ros_msgs::Waypoint current_waypoint;
  {
    std::lock_guard<std::mutex> lock(m_waypoint_buffer_mutex);
    if (m_waypoint_buffer.empty()) {
      ROS_WARN("[%s::plannning_callback] No waypoints to plan!", NAME);
      return;
    }

    current_waypoint = *m_waypoint_buffer.front();
  }

  // Transform carrot pose
  auto waypoint_frame = current_waypoint.pose.header.frame_id;
  if (m_transform_map.count(waypoint_frame) == 0) {
    ROS_FATAL_THROTTLE(2.0,
                       "[%s::plannning_callback] waypoint frame %s unrecognized",
                       NAME,
                       waypoint_frame.c_str());
    return;
  }

  auto           tracking_to_waypoint = m_transform_map.at(waypoint_frame);
  tf2::Transform tracking_to_waypoint_transform, waypoint_to_tracking_transform;
  tf2::fromMsg(tracking_to_waypoint.transform, tracking_to_waypoint_transform);
  waypoint_to_tracking_transform = tracking_to_waypoint_transform.inverse();
  auto waypoint_to_tracking      = tf2::toMsg(waypoint_to_tracking_transform);

  geometry_msgs::PoseStamped transformed_carrot_pose;
  tf2::doTransform(
    current_carrot_pose, transformed_carrot_pose, m_transform_map.at(waypoint_frame));

  // Do the planning
  larics_motion_planning::MultiDofTrajectoryRequest planning_request;
  // planning_request.waypoints
}

void uav_ros_tracker::OctomapPlannerClient::carrot_pose_cb(
  const geometry_msgs::PoseStamped& msg)
{
  std::lock_guard<std::mutex> lock(m_carrot_pose_mutex);
  m_carrot_pose = msg;
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(uav_ros_tracker::OctomapPlannerClient,
                       uav_ros_tracker::planner_interface);
