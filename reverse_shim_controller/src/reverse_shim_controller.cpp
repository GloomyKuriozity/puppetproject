#include <algorithm>
#include <string>
#include <memory>
#include <vector>
#include <utility>

#include "angles/angles.h"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/robot_utils.hpp"

#include "reverse_shim_controller/reverse_shim_controller.hpp"
#include "reverse_shim_controller/tools/utils.hpp"

using rcl_interfaces::msg::ParameterType;

namespace reverse_shim_controller
{

ReverseShimController::ReverseShimController()
: lp_loader_("nav2_core", "nav2_core::Controller"),
  primary_controller_(nullptr),
  path_updated_(false),
  in_rotation_(false)
{
}

void ReverseShimController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  plugin_name_ = name;
  node_ = parent;
  auto node = parent.lock();

  tf_ = tf;
  costmap_ros_ = costmap_ros;
  logger_ = node->get_logger();
  clock_ = node->get_clock();

  std::string primary_controller;
  double control_frequency;
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".angular_dist_threshold", rclcpp::ParameterValue(0.785));  // 45 deg
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".angular_disengage_threshold", rclcpp::ParameterValue(0.785 / 2.0));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".forward_sampling_distance", rclcpp::ParameterValue(0.5));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".rotate_to_heading_angular_vel", rclcpp::ParameterValue(1.8));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_angular_accel", rclcpp::ParameterValue(3.2));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".simulate_ahead_time", rclcpp::ParameterValue(1.0));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".primary_controller", rclcpp::PARAMETER_STRING);
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".rotate_to_goal_heading", rclcpp::ParameterValue(false));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".rotate_to_heading_once", rclcpp::ParameterValue(false));

  node->get_parameter(plugin_name_ + ".angular_dist_threshold", angular_dist_threshold_);
  node->get_parameter(plugin_name_ + ".angular_disengage_threshold", angular_disengage_threshold_);
  node->get_parameter(plugin_name_ + ".forward_sampling_distance", forward_sampling_distance_);
  node->get_parameter(
    plugin_name_ + ".rotate_to_heading_angular_vel",
    rotate_to_heading_angular_vel_);
  node->get_parameter(plugin_name_ + ".max_angular_accel", max_angular_accel_);
  node->get_parameter(plugin_name_ + ".simulate_ahead_time", simulate_ahead_time_);

  primary_controller = node->get_parameter(plugin_name_ + ".primary_controller").as_string();
  node->get_parameter("controller_frequency", control_frequency);
  control_duration_ = 1.0 / control_frequency;

  node->get_parameter(plugin_name_ + ".rotate_to_goal_heading", rotate_to_goal_heading_);
  node->get_parameter(plugin_name_ + ".rotate_to_heading_once", rotate_to_heading_once_);

  try {
    primary_controller_ = lp_loader_.createUniqueInstance(primary_controller);
    RCLCPP_INFO(
      logger_, "Created internal controller for rotation shimming: %s of type %s",
      plugin_name_.c_str(), primary_controller.c_str());
  } catch (const pluginlib::PluginlibException & ex) {
    RCLCPP_FATAL(
      logger_,
      "Failed to create internal controller for rotation shimming. Exception: %s", ex.what());
    return;
  }

  primary_controller_->configure(parent, name, tf, costmap_ros);

  // initialize collision checker and set costmap
  collision_checker_ = std::make_unique<nav2_costmap_2d::
      FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *>>(costmap_ros->getCostmap());
}

void ReverseShimController::activate()
{
  RCLCPP_INFO(
    logger_,
    "Activating controller: %s of type "
    "reverse_shim_controller::ReverseShimController",
    plugin_name_.c_str());

  primary_controller_->activate();
  in_rotation_ = false;

  auto node = node_.lock();
  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(
      &ReverseShimController::dynamicParametersCallback,
      this, std::placeholders::_1));
}

void ReverseShimController::deactivate()
{
  RCLCPP_INFO(
    logger_,
    "Deactivating controller: %s of type "
    "reverse_shim_controller::ReverseShimController",
    plugin_name_.c_str());

  primary_controller_->deactivate();

  if (auto node = node_.lock()) {
    node->remove_on_set_parameters_callback(dyn_params_handler_.get());
  }
  dyn_params_handler_.reset();
}

void ReverseShimController::cleanup()
{
  RCLCPP_INFO(
    logger_,
    "Cleaning up controller: %s of type "
    "reverse_shim_controller::ReverseShimController",
    plugin_name_.c_str());

  primary_controller_->cleanup();
  primary_controller_.reset();
}

geometry_msgs::msg::TwistStamped ReverseShimController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * goal_checker)
{
  // Rotate to goal heading when in goal xy tolerance
  if (rotate_to_goal_heading_) {
    std::lock_guard<std::mutex> lock_reinit(mutex_);

    try {
      geometry_msgs::msg::PoseStamped sampled_pt_goal = getSampledPathGoal();

      if (!nav2_util::transformPoseInTargetFrame(
          sampled_pt_goal, sampled_pt_goal, *tf_,
          pose.header.frame_id))
      {
        throw nav2_core::ControllerTFError("Failed to transform pose to base frame!");
      }

      if (utils::withinPositionGoalTolerance(
          goal_checker,
          pose.pose,
          sampled_pt_goal.pose))
      {
        double pose_yaw = tf2::getYaw(pose.pose.orientation);
        double goal_yaw = tf2::getYaw(sampled_pt_goal.pose.orientation);

        double angular_distance_to_heading = angles::shortest_angular_distance(pose_yaw, goal_yaw);

        return computeRotateToHeadingCommand(angular_distance_to_heading, pose, velocity);
      }
    } catch (const std::runtime_error & e) {
      RCLCPP_INFO(
        logger_,
        "Rotation Shim Controller was unable to find a goal point,"
        " a rotational collision was detected, or TF failed to transform"
        " into base frame! what(): %s", e.what());
    }
  }

  if (path_updated_) {
    nav2_costmap_2d::Costmap2D * costmap = costmap_ros_->getCostmap();
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap->getMutex()));

    std::lock_guard<std::mutex> lock_reinit(mutex_);
    try {
      geometry_msgs::msg::Pose sampled_pt_base = transformPoseToBaseFrame(getSampledPathPt());

      double angular_distance_to_heading =
        std::atan2(-sampled_pt_base.position.y, -sampled_pt_base.position.x);

      double angular_thresh =
        in_rotation_ ? angular_disengage_threshold_ : angular_dist_threshold_;
      if (abs(angular_distance_to_heading) > angular_thresh) {
        RCLCPP_INFO(
          logger_,
          "Robot is not within the new path's rough *backward* heading, rotating to *backward* heading... %f", angular_distance_to_heading);
        in_rotation_ = true;
        return computeRotateToHeadingCommand(angular_distance_to_heading, pose, velocity);
      } else {
        RCLCPP_DEBUG(
          logger_,
          "Robot is at the new path's rough *backward* heading, passing to controller");
        path_updated_ = false;
      }
    } catch (const std::runtime_error & e) {
      RCLCPP_DEBUG(
        logger_,
        "Rotation Shim Controller was unable to find a sampling point,"
        " a rotational collision was detected, or TF failed to transform"
        " into base frame! what(): %s", e.what());
      path_updated_ = false;
    }
  }

  // If at this point, use the primary controller to path track
  in_rotation_ = false;
  return primary_controller_->computeVelocityCommands(pose, velocity, goal_checker);
}

geometry_msgs::msg::PoseStamped ReverseShimController::getSampledPathPt()
{
  if (current_path_.poses.size() < 2) {
    throw nav2_core::ControllerException(
            "Path is too short to find a valid sampled path point for rotation.");
  }

  geometry_msgs::msg::Pose start = current_path_.poses.front().pose;
  double dx, dy;

  // Find the first point at least sampling distance away
  for (unsigned int i = 1; i != current_path_.poses.size(); i++) {
    dx = current_path_.poses[i].pose.position.x - start.position.x;
    dy = current_path_.poses[i].pose.position.y - start.position.y;
    if (hypot(dx, dy) >= forward_sampling_distance_) {
      current_path_.poses[i].header.frame_id = current_path_.header.frame_id;
      current_path_.poses[i].header.stamp = clock_->now();  // Get current time transformation
      return current_path_.poses[i];
    }
  }

  auto goal = current_path_.poses.back();
  goal.header.frame_id = current_path_.header.frame_id;
  goal.header.stamp = clock_->now();
  return goal;
}

geometry_msgs::msg::PoseStamped ReverseShimController::getSampledPathGoal()
{
  if (current_path_.poses.empty()) {
    throw nav2_core::InvalidPath("Path is empty - cannot find a goal point");
  }

  auto goal = current_path_.poses.back();
  goal.header.frame_id = current_path_.header.frame_id;
  goal.header.stamp = clock_->now();
  return goal;
}

geometry_msgs::msg::Pose
ReverseShimController::transformPoseToBaseFrame(const geometry_msgs::msg::PoseStamped & pt)
{
  geometry_msgs::msg::PoseStamped pt_base;
  if (!nav2_util::transformPoseInTargetFrame(pt, pt_base, *tf_, costmap_ros_->getBaseFrameID())) {
    throw nav2_core::ControllerTFError("Failed to transform pose to base frame!");
  }
  return pt_base.pose;
}

geometry_msgs::msg::TwistStamped
ReverseShimController::computeRotateToHeadingCommand(
  const double & angular_distance_to_heading,
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity)
{
  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header = pose.header;
  const double sign = angular_distance_to_heading > 0.0 ? 1.0 : -1.0;
  const double angular_vel = sign * rotate_to_heading_angular_vel_;
  const double & dt = control_duration_;
  const double min_feasible_angular_speed = velocity.angular.z - max_angular_accel_ * dt;
  const double max_feasible_angular_speed = velocity.angular.z + max_angular_accel_ * dt;
  cmd_vel.twist.angular.z =
    std::clamp(angular_vel, min_feasible_angular_speed, max_feasible_angular_speed);

  isCollisionFree(cmd_vel, angular_distance_to_heading, pose);
  return cmd_vel;
}

void ReverseShimController::isCollisionFree(
  const geometry_msgs::msg::TwistStamped & cmd_vel,
  const double & angular_distance_to_heading,
  const geometry_msgs::msg::PoseStamped & pose)
{
  // Simulate rotation ahead by time in control frequency increments
  double simulated_time = 0.0;
  double initial_yaw = tf2::getYaw(pose.pose.orientation);
  double yaw = 0.0;
  double footprint_cost = 0.0;
  double remaining_rotation_before_thresh =
    fabs(angular_distance_to_heading) - angular_dist_threshold_;

  while (simulated_time < simulate_ahead_time_) {
    simulated_time += control_duration_;
    yaw = initial_yaw + cmd_vel.twist.angular.z * simulated_time;

    // Stop simulating past the point it would be passed onto the primary controller
    if (angles::shortest_angular_distance(yaw, initial_yaw) >= remaining_rotation_before_thresh) {
      break;
    }

    using namespace nav2_costmap_2d;  // NOLINT
    footprint_cost = collision_checker_->footprintCostAtPose(
      pose.pose.position.x, pose.pose.position.y,
      yaw, costmap_ros_->getRobotFootprint());

    if (footprint_cost == static_cast<double>(NO_INFORMATION) &&
      costmap_ros_->getLayeredCostmap()->isTrackingUnknown())
    {
      throw nav2_core::NoValidControl(
              "ReverseShimController detected a potential collision ahead!");
    }

    if (footprint_cost >= static_cast<double>(LETHAL_OBSTACLE)) {
      throw nav2_core::NoValidControl("ReverseShimController detected collision ahead!");
    }
  }
}

bool ReverseShimController::isGoalChanged(const nav_msgs::msg::Path & path)
{
  // Return true if rotating or if the current path is empty
  if (in_rotation_ || current_path_.poses.empty()) {
    return true;
  }

  // Check if the last pose of the current and new paths differ
  return current_path_.poses.back().pose != path.poses.back().pose;
}

void ReverseShimController::setPlan(const nav_msgs::msg::Path & path)
{
  path_updated_ = rotate_to_heading_once_ ? isGoalChanged(path) : true;
  current_path_ = path;
  primary_controller_->setPlan(path);
}

void ReverseShimController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  primary_controller_->setSpeedLimit(speed_limit, percentage);
}

rcl_interfaces::msg::SetParametersResult
ReverseShimController::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  std::lock_guard<std::mutex> lock_reinit(mutex_);

  for (auto parameter : parameters) {
    const auto & type = parameter.get_type();
    const auto & name = parameter.get_name();

    if (type == ParameterType::PARAMETER_DOUBLE) {
      if (name == plugin_name_ + ".angular_dist_threshold") {
        angular_dist_threshold_ = parameter.as_double();
      } else if (name == plugin_name_ + ".forward_sampling_distance") {
        forward_sampling_distance_ = parameter.as_double();
      } else if (name == plugin_name_ + ".rotate_to_heading_angular_vel") {
        rotate_to_heading_angular_vel_ = parameter.as_double();
      } else if (name == plugin_name_ + ".max_angular_accel") {
        max_angular_accel_ = parameter.as_double();
      } else if (name == plugin_name_ + ".simulate_ahead_time") {
        simulate_ahead_time_ = parameter.as_double();
      }
    } else if (type == ParameterType::PARAMETER_BOOL) {
      if (name == plugin_name_ + ".rotate_to_goal_heading") {
        rotate_to_goal_heading_ = parameter.as_bool();
      } else if (name == plugin_name_ + ".rotate_to_heading_once") {
        rotate_to_heading_once_ = parameter.as_bool();
      }
    }
  }

  result.successful = true;
  return result;
}

}  // namespace reverse_shim_controller

// Register this controller as a nav2_core plugin
PLUGINLIB_EXPORT_CLASS(
  reverse_shim_controller::ReverseShimController,
  nav2_core::Controller)
