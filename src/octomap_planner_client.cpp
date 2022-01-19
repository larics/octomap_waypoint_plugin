#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TransformStamped.h"
#include "larics_motion_planning/MultiDofTrajectoryRequest.h"
#include "ros/duration.h"
#include "ros/forwards.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/convert.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"
#include <nav_msgs/Path.h>
#include "uav_ros_msgs/Waypoint.h"
#include <ios>
#include <mutex>
#include <octomap_waypoint_plugin/octomap_planner_client.hpp>
#include <larics_motion_planning/MultiDofTrajectory.h>
#include <larics_motion_planning/CheckStateValidity.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <uav_ros_lib/ros_convert.hpp>
#include <uav_ros_lib/param_util.hpp>

void uav_ros_tracker::OctomapPlannerClient::addWaypoint(
  const uav_ros_msgs::Waypoint& waypoint)
{
  std::lock_guard<std::mutex> lock(m_waypoint_buffer_mutex);
  m_waypoint_buffer.emplace_back(
    WaypointInfo{ m_waypoint_id_counter++,
                  boost::make_shared<uav_ros_msgs::Waypoint>(waypoint),
                  trajectory_msgs::JointTrajectory{} });

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
  {
    std::lock_guard<std::mutex> lock(m_waypoint_buffer_mutex);
    m_waypoint_buffer.clear();
  }
}

geometry_msgs::PoseArray uav_ros_tracker::OctomapPlannerClient::getWaypointArray()
{
  std::unordered_map<std::string, geometry_msgs::TransformStamped> transform_map_copy;
  {
    std::lock_guard<std::mutex> lock(m_transform_map_mutex);
    transform_map_copy = m_transform_map;
  }

  geometry_msgs::PoseArray poseArray;

  {
    std::lock_guard<std::mutex> lock(m_waypoint_buffer_mutex);
    for (const auto& waypoint : m_waypoint_buffer) {
      auto transformed_wp =
        transform_waypoint(*(waypoint.waypoint), transform_map_copy, m_tracking_frame);
      poseArray.poses.push_back(transformed_wp.pose.pose);
    }
  }

  return poseArray;
}

std::optional<uav_ros_msgs::WaypointPtr>
  uav_ros_tracker::OctomapPlannerClient::getCurrentWaypoint()
{
  std::optional<uav_ros_msgs::WaypointPtr> current_waypoint = std::nullopt;

  std::unordered_map<std::string, geometry_msgs::TransformStamped> transform_map_copy;
  {
    std::lock_guard<std::mutex> lock(m_transform_map_mutex);
    transform_map_copy = m_transform_map;
  }

  {
    std::lock_guard<std::mutex> lock(m_waypoint_buffer_mutex);

    if (m_waypoint_buffer.empty()) { return current_waypoint; }

    auto waypoint = m_waypoint_buffer.front();
    auto transformed_wp =
      transform_waypoint(*(waypoint.waypoint), transform_map_copy, m_tracking_frame);

    current_waypoint = std::make_optional<uav_ros_msgs::WaypointPtr>(
      boost::make_shared<uav_ros_msgs::Waypoint>(transformed_wp));
  }

  return current_waypoint;
}

uav_ros_msgs::WaypointStatus uav_ros_tracker::OctomapPlannerClient::getWaypointStatus(
  const nav_msgs::Odometry& odom)
{
  uav_ros_msgs::WaypointStatus waypointStatus;
  auto                         current_wp = getCurrentWaypoint();
  waypointStatus.current_wp =
    current_wp.has_value() ? *current_wp.value() : uav_ros_msgs::Waypoint{};
  waypointStatus.distance_to_wp = distanceToCurrentWp(odom);
  waypointStatus.flying_to_wp   = m_is_flying;
  waypointStatus.waiting_at_wp  = m_is_waiting;
  return waypointStatus;
}

double uav_ros_tracker::OctomapPlannerClient::distanceToCurrentWp(
  const nav_msgs::Odometry& odom)
{
  auto optional_waypoint = getCurrentWaypoint();
  if (!optional_waypoint.has_value() || !optional_waypoint.value()) { return -1; }

  return calc_distance(odom, *optional_waypoint.value());
}

bool uav_ros_tracker::OctomapPlannerClient::initialize(
  ros::NodeHandle&                                                 nh,
  ros::NodeHandle&                                                 nh_private,
  std::unordered_map<std::string, geometry_msgs::TransformStamped> transform_map,
  std::string                                                      tracking_frame)
{
  m_carrot_pose.pose.orientation.x = 0;
  m_carrot_pose.pose.orientation.y = 0;
  m_carrot_pose.pose.orientation.z = 0;
  m_carrot_pose.pose.orientation.w = 1;
  param_util::getParamOrThrow(
    nh_private, "octomap_planner_client/plan_and_fly", m_plan_and_fly);
  param_util::getParamOrThrow(
    nh_private, "octomap_planner_client/refresh_transforms", m_refresh_transform_map);

  m_tracking_frame = std::move(tracking_frame);
  m_transform_map  = std::move(transform_map);
  m_carrot_pose_sub =
    nh.subscribe("carrot/pose", 1, &OctomapPlannerClient::carrot_pose_cb, this);
  m_planner_client =
    nh.serviceClient<larics_motion_planning::MultiDofTrajectory>("multi_dof_trajectory");
  m_trajectory_checker_client =
    nh.serviceClient<larics_motion_planning::CheckStateValidity>("validity_checker");
  m_tracker_trajectory_pub =
    nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>("tracker/input_trajectory", 1);
  m_planend_path_pub =
    nh.advertise<nav_msgs::Path>("octomap_planner_client/planned_path", 1);

  m_waiting_timer = nh.createTimer(ros::Duration(1),
                                   &OctomapPlannerClient::waiting_callback,
                                   this,
                                   true /* oneshot */,
                                   false /*autostart */);

  m_planning_timer =
    nh.createTimer(ros::Duration(0.1), &OctomapPlannerClient::plannning_callback, this);
  m_visualization_timer =
    nh.createTimer(ros::Duration(1), &OctomapPlannerClient::visualization_callback, this);
  m_trajectory_checker_timer = nh.createTimer(
    ros::Duration(0.1), &OctomapPlannerClient::trajectory_checker_callback, this);

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
  WaypointInfo current_waypoint_info;
  int          waypoint_count = 0;
  {
    std::lock_guard<std::mutex> lock(m_waypoint_buffer_mutex);

    if (m_waypoint_buffer.empty()) {
      // reset();
      return std::make_tuple<bool, std::string, uav_ros_msgs::WaypointPtr>(
        false, "No waypoints available.", {});
    }

    waypoint_count        = m_waypoint_buffer.size();
    current_waypoint_info = m_waypoint_buffer.front();
  }

  if (waypoint_count == 0) {
    return std::make_tuple<bool, std::string, uav_ros_msgs::WaypointPtr>(
      false, "No waypoints available.", {});
  }

  // Flying to a waypoint that was cleared
  if (tracking_enabled && m_is_flying
      && current_waypoint_info.waypoint_id != m_waiting_id) {
    ROS_FATAL("Current waypoint changed!");
    if (tracking_enabled) { m_is_flying = false; }
  }

  auto distance_to_wp = distanceToCurrentWp(current_odometry);
  // If we are are flying and still haven't reached the distance
  if (m_is_flying && distance_to_wp >= DISTANCE_TOL) {
    return std::make_tuple(
      false, "Flying to current waypoint!", current_waypoint_info.waypoint);
  }

  // We have reached the desired waypoint distance, start the waiting
  if (m_is_flying && !m_is_waiting && distance_to_wp < DISTANCE_TOL) {
    m_is_waiting = true;
    m_is_flying  = false;
    m_waiting_timer.setPeriod(ros::Duration(current_waypoint_info.waypoint->waiting_time),
                              true);
    m_waiting_timer.start();
    return std::make_tuple(
      false, "Started waiting at the current waypoint!", current_waypoint_info.waypoint);
  }

  // If we are not flying but rather waiting than
  if (m_is_waiting) {
    return std::make_tuple(
      false, "Waiting at the current waypoint!", current_waypoint_info.waypoint);
  }

  // Check if tracker is available
  if (!tracking_enabled) {
    // reset();
    return std::make_tuple<bool, std::string, uav_ros_msgs::WaypointPtr>(
      false, "Tracker is busy!", {});
  }

  // Check if trajectory to that waypoint is planned
  if (current_waypoint_info.planned_path.points.empty()) {
    return { false, "Trajectory not planned yet!", current_waypoint_info.waypoint };
  }

  // If we don't want to plan and fly wait until there is a trajectory to every waypoint
  if (!m_plan_and_fly) {

    int planned_path_count = plannedPathCount();
    if (planned_path_count < waypoint_count) {
      return { false,
               "Planning status " + std::to_string(planned_path_count) + " / "
                 + std::to_string(waypoint_count),
               current_waypoint_info.waypoint };
    }
  }

  auto        current_trajectory = current_waypoint_info.planned_path;
  std::string waypoint_frame     = current_waypoint_info.planned_path.header.frame_id;

  std::unordered_map<std::string, geometry_msgs::TransformStamped> transform_map_copy;
  {
    std::lock_guard<std::mutex> lock(m_transform_map_mutex);
    transform_map_copy = m_transform_map;
  }

  // If there are only 2 points in the planned path, skip the first one and only send the
  // last
  bool skip_first_point = current_trajectory.points.size() == 2;

  trajectory_msgs::MultiDOFJointTrajectory tracking_path;
  for (const auto& path_point : current_trajectory.points) {

    if (skip_first_point) {
      skip_first_point = false;
      continue;
    }

    // Pack the path point in the waypoint
    uav_ros_msgs::Waypoint map_waypoint;
    map_waypoint.pose.header.frame_id = waypoint_frame;
    map_waypoint.pose.pose.position.x = path_point.positions[0];
    map_waypoint.pose.pose.position.y = path_point.positions[1];
    map_waypoint.pose.pose.position.z = path_point.positions[2];
    map_waypoint.pose.pose.orientation =
      ros_convert::calculate_quaternion(path_point.positions[3]);

    auto tracking_waypoint =
      transform_waypoint(map_waypoint, transform_map_copy, m_tracking_frame);

    // construct the tracking trajectory point
    trajectory_msgs::MultiDOFJointTrajectoryPoint tracking_point;
    tracking_point.transforms = std::vector<geometry_msgs::Transform>(1);
    tracking_point.transforms.front().translation.x =
      tracking_waypoint.pose.pose.position.x;
    tracking_point.transforms.front().translation.y =
      tracking_waypoint.pose.pose.position.y;
    tracking_point.transforms.front().translation.z =
      tracking_waypoint.pose.pose.position.z;
    tracking_point.transforms.front().rotation.x =
      tracking_waypoint.pose.pose.orientation.x;
    tracking_point.transforms.front().rotation.y =
      tracking_waypoint.pose.pose.orientation.y;
    tracking_point.transforms.front().rotation.z =
      tracking_waypoint.pose.pose.orientation.z;
    tracking_point.transforms.front().rotation.w =
      tracking_waypoint.pose.pose.orientation.w;
    tracking_path.points.push_back(tracking_point);
  }

  m_waiting_id = current_waypoint_info.waypoint_id;
  m_tracker_trajectory_pub.publish(tracking_path);
  m_is_flying = true;
  return { true, "Planned trajectory published!", current_waypoint_info.waypoint };
}

void uav_ros_tracker::OctomapPlannerClient::updateTransformMap(
  std::unordered_map<std::string, geometry_msgs::TransformStamped> transform_map)
{
  if (!m_refresh_transform_map) return;
  std::lock_guard<std::mutex> lock(m_transform_map_mutex);
  m_transform_map = std::move(transform_map);
}

int uav_ros_tracker::OctomapPlannerClient::plannedPathCount()
{
  std::lock_guard<std::mutex> lock(m_waypoint_buffer_mutex);
  int                         planned_path_count = 0;

  for (const auto& waypoint_info : m_waypoint_buffer) {
    if (waypoint_info.planned_path.points.empty()) continue;
    planned_path_count++;
  }
  return planned_path_count;
}

void uav_ros_tracker::OctomapPlannerClient::visualization_callback(
  const ros::TimerEvent& e)
{
  std::deque<WaypointInfo> waypoint_buffer;
  {
    std::lock_guard<std::mutex> lock(m_waypoint_buffer_mutex);
    waypoint_buffer = m_waypoint_buffer;
  }

  nav_msgs::Path planned_path;
  planned_path.header.frame_id = m_last_waypoint_frame;
  planned_path.header.stamp    = ros::Time::now();

  if (!waypoint_buffer.empty()) {
    planned_path.header.frame_id = waypoint_buffer.front().planned_path.header.frame_id;
  }

  for (const auto& waypoint_info : waypoint_buffer) {
    for (const auto& joint_point : waypoint_info.planned_path.points) {
      geometry_msgs::PoseStamped pose_stamped;
      pose_stamped.header          = planned_path.header;
      pose_stamped.pose.position.x = joint_point.positions[0];
      pose_stamped.pose.position.y = joint_point.positions[1];
      pose_stamped.pose.position.z = joint_point.positions[2];
      pose_stamped.pose.orientation =
        ros_convert::calculate_quaternion(joint_point.positions[3]);
      planned_path.poses.push_back(pose_stamped);
    }
  }

  m_planend_path_pub.publish(planned_path);
}

void uav_ros_tracker::OctomapPlannerClient::trajectory_checker_callback(
  const ros::TimerEvent& e)
{
  if (!m_trajectory_checker_client.exists()) {
    ROS_WARN_THROTTLE(
      2.0, "[%s::trajectory_checker_callback] Checker service is missing!", NAME);
    return;
  }

  std::deque<WaypointInfo> waypoint_buffer_copy;
  {
    std::lock_guard<std::mutex> lock(m_waypoint_buffer_mutex);
    waypoint_buffer_copy = m_waypoint_buffer;
  }

  for (const auto& waypoint_info : waypoint_buffer_copy) {

    if (waypoint_info.planned_path.points.empty()) {
      continue;
    }

    larics_motion_planning::CheckStateValidity state_validity;
    state_validity.request.points = waypoint_info.planned_path;

    auto call_success = m_trajectory_checker_client.call(state_validity);
    if (!call_success) {
      ROS_FATAL("[%s::trajectory_checker_callback] call failed for trajectory id %d",
                NAME,
                waypoint_info.waypoint_id);
      continue;
    }

    if (state_validity.response.valid) { continue; }
    ROS_FATAL("[%s::trajectory_checker_callback] Collision detected at trajectory id %d",
              NAME,
              waypoint_info.waypoint_id);
    
    // TODO: Do something if colision is detected
  }
}

void uav_ros_tracker::OctomapPlannerClient::waiting_callback(const ros::TimerEvent& e)
{
  ROS_INFO("[%s] Waiting at waypoint finished!", NAME);
  m_is_waiting = false;

  {
    std::lock_guard<std::mutex> lock(m_waypoint_buffer_mutex);
    if (m_waypoint_buffer.empty()) {
      ROS_WARN(
        "[waiting_callback] Waiting at waypoint %d finished but waypoints were cleared "
        "in the meantime.",
        m_waiting_id);
      return;
    }

    if (m_waypoint_buffer.front().waypoint_id == m_waiting_id) {
      m_waypoint_buffer.pop_front();
    } else {
      ROS_WARN(
        "[waiting_callback] Not poping waypoint from buffer. It was changed while "
        "waiting.");
      return;
    }
  }
}

void uav_ros_tracker::OctomapPlannerClient::plannning_callback(const ros::TimerEvent& e)
{
  ROS_INFO_THROTTLE(2.0, "[%s::planning_callback]", NAME);

  if (!m_planner_client.exists()) {
    ROS_WARN_THROTTLE(2.0, "[%s::planning_callback] Planning service is missing!", NAME);
    return;
  }

  if (m_carrot_pose_sub.getNumPublishers() < 1) {
    ROS_WARN_THROTTLE(2.0, "[%s::planning_callback] Carrot publisher is missing!", NAME);
    return;
  }

  // Get the current carrot pose
  geometry_msgs::PoseStamped current_carrot_pose;
  {
    std::lock_guard<std::mutex> lock(m_carrot_pose_mutex);
    current_carrot_pose = m_carrot_pose;
  }

  // Get the current waypoint
  std::deque<WaypointInfo> waypoint_buffer_copy;
  {
    std::lock_guard<std::mutex> lock(m_waypoint_buffer_mutex);
    if (m_waypoint_buffer.empty()) {
      ROS_WARN_THROTTLE(2.0, "[%s::plannning_callback] No waypoints to plan!", NAME);
      return;
    }

    waypoint_buffer_copy = m_waypoint_buffer;
  }

  std::unordered_map<std::string, geometry_msgs::TransformStamped> transform_map_copy;
  {
    std::lock_guard<std::mutex> lock(m_transform_map_mutex);
    transform_map_copy = m_transform_map;
  }

  int planned_trajectory_count = plannedPathCount();
  if (m_is_flying && planned_trajectory_count == 0) {
    ROS_WARN_THROTTLE(2.0,
                      "[%s::planning_callback] Not planning while flying and no other "
                      "trajectories are available!",
                      NAME);
    return;
  }

  if (planned_trajectory_count == waypoint_buffer_copy.size()) {
    ROS_WARN_THROTTLE(2.0, "[%s::planning_callback] All waypoints are planned!", NAME);
    return;
  }

  // Transform carrot pose
  m_last_waypoint_frame = waypoint_buffer_copy.front().waypoint->pose.header.frame_id;
  if (transform_map_copy.count(m_last_waypoint_frame) == 0) {
    ROS_FATAL_THROTTLE(2.0,
                       "[%s::plannning_callback] waypoint frame %s unrecognized",
                       NAME,
                       m_last_waypoint_frame.c_str());
    return;
  }

  // Carrot pose is in the current tracking frame
  // Transform it to the waypoint frame to figure out where to start planning
  const auto     waypoint_to_tracking = transform_map_copy.at(m_last_waypoint_frame);
  tf2::Transform waypoint_to_tracking_transform;
  tf2::fromMsg(waypoint_to_tracking.transform, waypoint_to_tracking_transform);
  const auto tracking_to_waypoint_transform = waypoint_to_tracking_transform.inverse();
  const auto tracking_to_waypoint           = tf2::toMsg(tracking_to_waypoint_transform);

  // doTransfrom wants TransfromStamped and not Transfrom
  geometry_msgs::TransformStamped tracking_to_waypoint_stamped;
  tracking_to_waypoint_stamped.transform = tracking_to_waypoint;

  // Finally transform the carrot pose
  geometry_msgs::PoseStamped transformed_carrot_pose;
  tf2::doTransform(
    current_carrot_pose, transformed_carrot_pose, tracking_to_waypoint_stamped);

  // Check if we should start planning from carrot/pose
  bool empty_trajectory_buffer = planned_trajectory_count == 0;

  // Plan trajectories
  for (int i = 0; i < waypoint_buffer_copy.size(); i++) {

    // Check if waypoint is already planned
    if (!waypoint_buffer_copy[i].planned_path.points.empty()) {
      ROS_INFO("[planning_loop] Skipping waypoing at index %d", i);
      continue;
    }

    trajectory_msgs::JointTrajectoryPoint start_point;
    if (i == 0) {
      // If the first waypoint is not planned, plan it from the current carrot point
      start_point.positions =
        std::vector<double>{ transformed_carrot_pose.pose.position.x,
                             transformed_carrot_pose.pose.position.y,
                             transformed_carrot_pose.pose.position.z,
                             ros_convert::calculateYaw(
                               transformed_carrot_pose.pose.orientation.x,
                               transformed_carrot_pose.pose.orientation.y,
                               transformed_carrot_pose.pose.orientation.z,
                               transformed_carrot_pose.pose.orientation.w) };
    } else {
      const auto previous_waypoint = *waypoint_buffer_copy[i - 1].waypoint;
      start_point.positions =
        std::vector<double>{ previous_waypoint.pose.pose.position.x,
                             previous_waypoint.pose.pose.position.y,
                             previous_waypoint.pose.pose.position.z,
                             ros_convert::calculateYaw(
                               previous_waypoint.pose.pose.orientation.x,
                               previous_waypoint.pose.pose.orientation.y,
                               previous_waypoint.pose.pose.orientation.z,
                               previous_waypoint.pose.pose.orientation.w) };
    }

    const auto current_waypoint = *waypoint_buffer_copy[i].waypoint;
    trajectory_msgs::JointTrajectoryPoint end_point;
    end_point.positions =
      std::vector<double>{ current_waypoint.pose.pose.position.x,
                           current_waypoint.pose.pose.position.y,
                           current_waypoint.pose.pose.position.z,
                           ros_convert::calculateYaw(
                             current_waypoint.pose.pose.orientation.x,
                             current_waypoint.pose.pose.orientation.y,
                             current_waypoint.pose.pose.orientation.z,
                             current_waypoint.pose.pose.orientation.w) };

    auto planned_points =
      planTrajectoryBetween(start_point, end_point, m_last_waypoint_frame);
    planned_points.header.frame_id = m_last_waypoint_frame;
    planned_points.header.stamp    = ros::Time::now();
    if (planned_points.points.empty()) {
      ROS_FATAL("[planning_loop] Planning failed from %d to %d", i - 1, i);
      continue;
    }

    {
      // Copy the planned points to the trajectory
      std::lock_guard<std::mutex> lock(m_waypoint_buffer_mutex);

      if (m_waypoint_buffer.size() < i + 1) {
        ROS_FATAL(
          "[planning_loop] Waypoint buffer size changed while planning. Planning "
          "aborted!");
        return;
      }

      if (m_waypoint_buffer[i].waypoint_id != waypoint_buffer_copy[i].waypoint_id) {
        ROS_FATAL(
          "[planning_loop] Waypoint ID changed while path planning. Planning aborted!");
        return;
      }

      // Save the planned path if all is well
      m_waypoint_buffer[i].planned_path = planned_points;
    }
  }
}

trajectory_msgs::JointTrajectory
  uav_ros_tracker::OctomapPlannerClient::planTrajectoryBetween(
    const trajectory_msgs::JointTrajectoryPoint& start_point,
    const trajectory_msgs::JointTrajectoryPoint& end_point,
    const std::string&                           waypoint_frame)
{
  ROS_INFO_STREAM_THROTTLE(2.0, "[planTrajectoryBetween] Start point " << start_point);
  ROS_INFO_STREAM_THROTTLE(2.0, "[planTrajectoryBetween] End point " << end_point);

  // Pack the points in the request
  larics_motion_planning::MultiDofTrajectory planning_service;
  planning_service.request.waypoints.points =
    std::vector<trajectory_msgs::JointTrajectoryPoint>{ start_point, end_point };
  planning_service.request.waypoints.joint_names =
    std::vector<std::string>{ "x", "y", "z", "yaw" };
  planning_service.request.plan_trajectory    = true;
  planning_service.request.plan_path          = true;
  planning_service.request.publish_path       = true;
  planning_service.request.publish_trajectory = true;

  auto call_success = m_planner_client.call(planning_service);
  if (!call_success) {
    ROS_FATAL_THROTTLE(
      2.0, "[%s::planTrajectoryBetween] call to plannning service failed.", NAME);
    return trajectory_msgs::JointTrajectory{};
  }

  auto planning_response = planning_service.response;
  if (!planning_response.success) {
    ROS_WARN_THROTTLE(2.0, "[%s::planTrajectoryBetween] Planning failed!", NAME);
    // TODO: Do something if planning fails...
    return trajectory_msgs::JointTrajectory{};
  }

  ROS_INFO_THROTTLE(1.0,
                    "Path length: %ld, trajectory length: %ld",
                    planning_response.path.points.size(),
                    planning_response.trajectory.points.size());


  return planning_response.path;
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
