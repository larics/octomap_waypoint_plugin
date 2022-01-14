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
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <uav_ros_lib/ros_convert.hpp>
#include <uav_ros_lib/param_util.hpp>

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
  {
    std::lock_guard<std::mutex> lock(m_waypoint_buffer_mutex);
    m_waypoint_buffer.clear();
  }

  {
    std::lock_guard<std::mutex> lock(m_waypoint_trajectory_mutex);
    m_waypoint_trajectory_buffer.clear();
  }

  m_is_flying  = false;
  m_is_waiting = false;
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

std::optional<uav_ros_msgs::WaypointPtr>
  uav_ros_tracker::OctomapPlannerClient::getCurrentWaypoint()
{
  std::optional<uav_ros_msgs::WaypointPtr> current_waypoint = std::nullopt;

  {
    std::lock_guard<std::mutex> lock(m_waypoint_buffer_mutex);

    if (m_waypoint_buffer.empty()) { return current_waypoint; }

    auto waypoint = m_waypoint_buffer.front();
    auto transformed_wp =
      transform_waypoint(*waypoint, m_transform_map, m_tracking_frame);

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

  m_tracking_frame = std::move(tracking_frame);
  m_transform_map  = std::move(transform_map);
  m_carrot_pose_sub =
    nh.subscribe("carrot/pose", 1, &OctomapPlannerClient::carrot_pose_cb, this);
  m_planner_client =
    nh.serviceClient<larics_motion_planning::MultiDofTrajectory>("multi_dof_trajectory");
  m_tracker_trajectory_pub =
    nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>("tracker/input_trajectory", 1);

  m_waiting_timer = nh.createTimer(ros::Duration(1),
                                   &OctomapPlannerClient::waiting_callback,
                                   this,
                                   true /* oneshot */,
                                   false /*autostart */);

  m_planning_timer =
    nh.createTimer(ros::Duration(0.1), &OctomapPlannerClient::plannning_callback, this);
  m_visualization_timer =
    nh.createTimer(ros::Duration(1), &OctomapPlannerClient::visualization_callback, this);

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
  int                       waypoint_count = 0;
  {
    std::lock_guard<std::mutex> lock(m_waypoint_buffer_mutex);

    if (m_waypoint_buffer.empty()) {
      // reset();
      return std::make_tuple<bool, std::string, uav_ros_msgs::WaypointPtr>(
        false, "No waypoints available.", {});
    }

    waypoint_count       = m_waypoint_buffer.size();
    current_waypoint_ptr = m_waypoint_buffer.front();
  }

  {
    std::lock_guard<std::mutex> lock(m_waypoint_trajectory_mutex);
    if (waypoint_count < m_waypoint_trajectory_buffer.size()) {
      ROS_WARN(
        "Waypoint count: %d is less than planned trajectory count: %d. Something has "
        "gone wrong! Clearing planned trajectory.");
    }
  }

  if (current_waypoint_ptr == nullptr) {
    return std::make_tuple<bool, std::string, uav_ros_msgs::WaypointPtr>(
      false, "No waypoints available.", {});
  }

  auto distance_to_wp = distanceToCurrentWp(current_odometry);
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

  trajectory_msgs::JointTrajectory joint_trajectory;
  {
    std::lock_guard<std::mutex> lock(m_waypoint_trajectory_mutex);
    if (m_waypoint_trajectory_buffer.empty()) {
      return { false, "Trajectory not planned yet!", nullptr };
    }

    if (!m_plan_and_fly && waypoint_count != m_waypoint_trajectory_buffer.size()) {
      return { false,
               "Planning status " + std::to_string(m_waypoint_trajectory_buffer.size())
                 + " / " + std::to_string(waypoint_count),
               nullptr };
    }
    joint_trajectory = m_waypoint_trajectory_buffer.front();
  }

  std::string waypoint_frame;
  {
    std::lock_guard<std::mutex> lock(m_waypoint_buffer_mutex);
    waypoint_frame = m_waypoint_buffer.front()->pose.header.frame_id;
  }

  trajectory_msgs::MultiDOFJointTrajectory tracking_path;
  for (const auto& path_point : joint_trajectory.points) {

    // Pack the path point in the waypoint
    uav_ros_msgs::Waypoint map_waypoint;
    map_waypoint.pose.header.frame_id = waypoint_frame;
    map_waypoint.pose.pose.position.x = path_point.positions[0];
    map_waypoint.pose.pose.position.y = path_point.positions[1];
    map_waypoint.pose.pose.position.z = path_point.positions[2];
    map_waypoint.pose.pose.orientation =
      ros_convert::calculate_quaternion(path_point.positions[3]);

    auto tracking_waypoint =
      transform_waypoint(map_waypoint, m_transform_map, m_tracking_frame);

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

  m_tracker_trajectory_pub.publish(tracking_path);
  m_is_flying = true;
  return { true, "Planned trajectory published!", current_waypoint_ptr };
}

void uav_ros_tracker::OctomapPlannerClient::visualization_callback(
  const ros::TimerEvent& e)
{
  std::deque<trajectory_msgs::JointTrajectory> joint_trajectory;
  {
    std::lock_guard<std::mutex> lock(m_waypoint_trajectory_mutex);
    joint_trajectory = m_waypoint_trajectory_buffer;
  }
}

void uav_ros_tracker::OctomapPlannerClient::waiting_callback(const ros::TimerEvent& e)
{
  ROS_INFO("[%s] Waiting at waypoint finished!", NAME);
  m_is_waiting = false;

  {
    std::lock_guard<std::mutex> lock(m_waypoint_trajectory_mutex);
    if (!m_waypoint_trajectory_buffer.empty()) m_waypoint_trajectory_buffer.pop_front();
  }

  {
    std::lock_guard<std::mutex> lock(m_waypoint_buffer_mutex);
    if (!m_waypoint_buffer.empty()) m_waypoint_buffer.pop_front();
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
  std::deque<uav_ros_msgs::WaypointPtr> waypoint_buffer_copy;
  {
    std::lock_guard<std::mutex> lock(m_waypoint_buffer_mutex);
    if (m_waypoint_buffer.empty()) {
      ROS_WARN_THROTTLE(2.0, "[%s::plannning_callback] No waypoints to plan!", NAME);
      return;
    }

    waypoint_buffer_copy = m_waypoint_buffer;
  }

  // Transform carrot pose
  auto waypoint_frame = waypoint_buffer_copy.front()->pose.header.frame_id;
  if (m_transform_map.count(waypoint_frame) == 0) {
    ROS_FATAL_THROTTLE(2.0,
                       "[%s::plannning_callback] waypoint frame %s unrecognized",
                       NAME,
                       waypoint_frame.c_str());
    return;
  }

  // Carrot pose is in the current tracking frame
  // Transform it to the waypoint frame to figure out where to start planning
  const auto     waypoint_to_tracking = m_transform_map.at(waypoint_frame);
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
  bool empty_trajectory_buffer  = false;
  int  planned_trajectory_count = 0;
  {
    std::lock_guard<std::mutex> lock(m_waypoint_trajectory_mutex);
    if (m_waypoint_trajectory_buffer.empty()) { empty_trajectory_buffer = true; }
    planned_trajectory_count = m_waypoint_trajectory_buffer.size();
  }

  std::vector<trajectory_msgs::JointTrajectoryPoint> unplanned_points;
  trajectory_msgs::JointTrajectoryPoint              start_point;

  // If trajectory buffer is empty start point is the carrot point
  if (empty_trajectory_buffer) {
    // Pack the carrot point to JointTrajectoryPoint
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
    // Otherwise start points is the last planned waypoint
    start_point.positions = std::vector<double>{
      waypoint_buffer_copy.at(planned_trajectory_count - 1)->pose.pose.position.x,
      waypoint_buffer_copy.at(planned_trajectory_count - 1)->pose.pose.position.y,
      waypoint_buffer_copy.at(planned_trajectory_count - 1)->pose.pose.position.z,
      ros_convert::calculateYaw(
        waypoint_buffer_copy.at(planned_trajectory_count - 1)->pose.pose.orientation.x,
        waypoint_buffer_copy.at(planned_trajectory_count - 1)->pose.pose.orientation.y,
        waypoint_buffer_copy.at(planned_trajectory_count - 1)->pose.pose.orientation.z,
        waypoint_buffer_copy.at(planned_trajectory_count - 1)->pose.pose.orientation.w)
    };
  }

  for (int i = planned_trajectory_count; i < waypoint_buffer_copy.size(); i++) {
    trajectory_msgs::JointTrajectoryPoint point;
    point.positions =
      std::vector<double>{ waypoint_buffer_copy[i]->pose.pose.position.x,
                           waypoint_buffer_copy[i]->pose.pose.position.y,
                           waypoint_buffer_copy[i]->pose.pose.position.z,
                           ros_convert::calculateYaw(
                             waypoint_buffer_copy[i]->pose.pose.orientation.x,
                             waypoint_buffer_copy[i]->pose.pose.orientation.y,
                             waypoint_buffer_copy[i]->pose.pose.orientation.z,
                             waypoint_buffer_copy[i]->pose.pose.orientation.w) };
    ROS_INFO_STREAM("Add point: " << point << "\n to unplanned_points!");
    unplanned_points.push_back(point);
  }

  // Plan trajectories
  for (int i = 0; i < unplanned_points.size(); i++) {
    trajectory_msgs::JointTrajectoryPoint first_point;
    trajectory_msgs::JointTrajectoryPoint end_point = unplanned_points[i];

    // For first point either take previous unplanned or start_point
    if (i == 0) {
      first_point = start_point;
    } else {
      first_point = unplanned_points[i - 1];
    }

    auto planned_points = planTrajectoryBetween(first_point, end_point, waypoint_frame);
    planned_points.header.frame_id = waypoint_frame;
    {
      std::lock_guard<std::mutex> lock(m_waypoint_trajectory_mutex);
      m_waypoint_trajectory_buffer.push_back(planned_points);
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
