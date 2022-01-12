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
#include "trajectory_msgs/MultiDOFJointTrajectoryPoint.h"
#include "uav_ros_msgs/Waypoint.h"
#include <ios>
#include <mutex>
#include <octomap_waypoint_plugin/octomap_planner_client.hpp>
#include <larics_motion_planning/MultiDofTrajectory.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <uav_ros_lib/ros_convert.hpp>

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

  // Pack the carrot point to JointTrajectoryPoint
  trajectory_msgs::JointTrajectoryPoint carrot_point;
  carrot_point.positions =
    std::vector<double>{ transformed_carrot_pose.pose.position.x,
                         transformed_carrot_pose.pose.position.y,
                         transformed_carrot_pose.pose.position.z,
                         ros_convert::calculateYaw(
                           transformed_carrot_pose.pose.orientation.x,
                           transformed_carrot_pose.pose.orientation.y,
                           transformed_carrot_pose.pose.orientation.z,
                           transformed_carrot_pose.pose.orientation.w) };

  // Pack the waypoint point to JointTrajectoryPoint
  trajectory_msgs::JointTrajectoryPoint waypoint_point;
  waypoint_point.positions =
    std::vector<double>{ current_waypoint.pose.pose.position.x,
                         current_waypoint.pose.pose.position.y,
                         current_waypoint.pose.pose.position.z,
                         ros_convert::calculateYaw(
                           current_waypoint.pose.pose.orientation.x,
                           current_waypoint.pose.pose.orientation.y,
                           current_waypoint.pose.pose.orientation.z,
                           current_waypoint.pose.pose.orientation.w) };

  ROS_INFO_STREAM_THROTTLE(
    2.0, "[plannning_callback] transformed carrot pose " << transformed_carrot_pose);

  ROS_INFO_STREAM_THROTTLE(
    2.0, "[plannning_callback] waypoint_pose pose " << current_waypoint.pose);

  // Pack the points in the request
  larics_motion_planning::MultiDofTrajectory planning_service;
  planning_service.request.waypoints.points =
    std::vector<trajectory_msgs::JointTrajectoryPoint>{ carrot_point, waypoint_point };
  planning_service.request.waypoints.joint_names =
    std::vector<std::string>{ "x", "y", "z", "yaw" };
  planning_service.request.plan_trajectory    = true;
  planning_service.request.plan_path          = true;
  planning_service.request.publish_path       = true;
  planning_service.request.publish_trajectory = true;

  auto call_success = m_planner_client.call(planning_service);
  if (!call_success) {
    ROS_FATAL_THROTTLE(
      2.0, "[%s::plannning_callback] call to plannning service failed.", NAME);
    return;
  }

  auto planning_response = planning_service.response;
  if (!planning_response.success) {
    ROS_WARN_THROTTLE(2.0, "[%s::plannning_callback] Planning failed!", NAME);
    // TODO: Do something if planning fails...
    return;
  }

  ROS_INFO_THROTTLE(1.0,
                    "Path length: %ld, trajectory length: %ld",
                    planning_response.path.points.size(),
                    planning_response.trajectory.points.size());

  trajectory_msgs::MultiDOFJointTrajectory tracking_path;
  for (const auto& path_point : planning_response.path.points) {

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
