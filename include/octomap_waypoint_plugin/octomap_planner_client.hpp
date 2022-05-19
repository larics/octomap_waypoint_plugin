#ifndef OCTOMAP_PLANNER_HPP
#define OCTOMAP_PLANNER_HPP

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <trajectory_msgs/JointTrajectory.h>

#include <ros/forwards.h>
#include <ros/service_client.h>
#include <ros/subscriber.h>
#include <uav_ros_tracker/planner_interface.hpp>
#include <deque>
#include <mutex>

namespace uav_ros_tracker {

enum State { 
  IDLE,  // There is no trajectory, path was not planned yet
  READY_TO_FLY, // The trajectory has been requested but not yet started
  FLYING,  // UAV is flying, trajectory is active
  WAITING  // UAV is waiting
};
static constexpr auto NO_ID = -1;

struct WaypointInfo
{
  int                              waypoint_id;
  uav_ros_msgs::WaypointPtr        waypoint;
  trajectory_msgs::JointTrajectory planned_path;
};

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
  void                         updateTransformMap(
                            std::unordered_map<std::string, geometry_msgs::TransformStamped> transform_map);

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
  double                                   distanceToCurrentWp();
  trajectory_msgs::JointTrajectory         planTrajectoryBetween(
            const trajectory_msgs::JointTrajectoryPoint& start_point,
            const trajectory_msgs::JointTrajectoryPoint& end_point,
            const std::string&                           waypoint_frame);
  int plannedPathCount();

  static constexpr auto NAME         = "OctomapPlannerClient";
  static constexpr auto DISTANCE_TOL = 0.8;

  ros::Timer m_waiting_timer;
  void       waiting_callback(const ros::TimerEvent& e);

  ros::Timer m_planning_timer;
  void       plannning_callback(const ros::TimerEvent& e);

  ros::Timer m_visualization_timer;
  void       visualization_callback(const ros::TimerEvent& e);

  ros::Timer m_trajectory_checker_timer;
  void       trajectory_checker_callback(const ros::TimerEvent& e);

  geometry_msgs::PoseStamped m_carrot_pose;
  std::mutex                 m_carrot_pose_mutex;
  ros::Subscriber            m_carrot_pose_sub;
  void                       carrot_pose_cb(const geometry_msgs::PoseStamped& pose);

  int m_flying_id = NO_ID;

  std::mutex                                                       m_transform_map_mutex;
  std::unordered_map<std::string, geometry_msgs::TransformStamped> m_transform_map;
  std::string                                                      m_tracking_frame;
  bool m_refresh_transform_map = false;

  std::mutex                  m_waypoint_buffer_mutex;
  std::map<int, WaypointInfo> m_waypoint_buffer;

  ros::ServiceClient m_trajectory_checker_client;
  ros::ServiceClient m_planner_client;
  std::mutex         m_tracker_mutex;
  ros::ServiceClient m_tracker_reset_client;
  ros::Publisher     m_tracker_trajectory_pub;
  ros::Publisher     m_planend_path_pub;
  bool               m_plan_and_fly;
  bool               m_enable_replanning;
  std::string        m_last_waypoint_frame;
  int                m_waypoint_id_counter = 1;

  State get_state()
  {
    State copy;
    {
      std::lock_guard<std::mutex> lock(m_state_mutex);
      copy = m_current_state;
    }
    return copy;
  }

  void set_state(State newState, int new_id = -1)
  {
    {
      std::lock_guard<std::mutex> lock(m_state_mutex);
      m_current_state = newState;

      if (m_current_state == IDLE) { m_flying_id = -1; }

      if (m_current_state == READY_TO_FLY) { m_flying_id = new_id; }
    }
  }

  int get_flying_id()
  {
    int id = -1;
    {
      std::lock_guard<std::mutex> lock(m_state_mutex);
      id = m_flying_id;
    }
    return id;
  }

  bool is_flying() { return get_state() == FLYING; }
  bool is_waiting() { return get_state() == WAITING; }
  bool is_idle() { return get_state() == IDLE; }
  bool is_ready_to_fly() { return get_state() == READY_TO_FLY; }

  std::mutex m_state_mutex;
  State      m_current_state = IDLE;
};


}// namespace uav_ros_tracker

#endif /* OCTOMAP_PLANNER_HPP */
