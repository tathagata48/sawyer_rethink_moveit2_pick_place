// Sawyer pick and place with MoveIt 2 (Humble) - final version
// table + can + obstacle, top-down grasp, straight-line approach/retreat,
// attach/detach, repeated cycles, everything configurable via ROS parameters.

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

using moveit::planning_interface::MoveGroupInterface;
using moveit::planning_interface::PlanningSceneInterface;
using moveit_msgs::msg::CollisionObject;

constexpr double kPi = 3.14159265358979323846;

// ============================================================================
// Configuration (ROS parameters)
// ============================================================================
struct Config
{
  std::string arm_group, tcp_link;
  double pick_x, pick_y, place_x, place_y;
  double table_top, table_x, table_size_x, table_size_y;
  double can_radius, can_height, approach;
  bool add_obstacle;
  double obstacle_height;
  int cycles;
  double velocity_scaling, line_speed;

  double can_z() const {return table_top + 0.001 + can_height / 2.0;}  // can centre
};

// read a parameter, declaring it with a default if the launch file did not set it
template<typename T>
T param(const rclcpp::Node::SharedPtr & node, const std::string & name, const T & def)
{
  if (!node->has_parameter(name)) {node->declare_parameter<T>(name, def);}
  return node->get_parameter(name).get_value<T>();
}

Config load_config(const rclcpp::Node::SharedPtr & node)
{
  Config c;
  c.arm_group = param<std::string>(node, "arm_group", "arm");
  c.tcp_link = param<std::string>(node, "tcp_link", "right_gripper_tip");
  c.pick_x = param<double>(node, "pick_x", 0.65);
  c.pick_y = param<double>(node, "pick_y", 0.20);
  c.place_x = param<double>(node, "place_x", 0.65);
  c.place_y = param<double>(node, "place_y", -0.20);
  c.table_top = param<double>(node, "table_top_z", -0.15);
  c.table_x = param<double>(node, "table_x", 0.75);
  c.table_size_x = param<double>(node, "table_size_x", 0.70);
  c.table_size_y = param<double>(node, "table_size_y", 1.20);
  c.can_radius = param<double>(node, "can_radius", 0.012);
  c.can_height = param<double>(node, "can_height", 0.08);
  c.approach = param<double>(node, "approach_distance", 0.12);
  c.add_obstacle = param<bool>(node, "add_obstacle", true);
  c.obstacle_height = param<double>(node, "obstacle_height", 0.20);
  c.cycles = static_cast<int>(param<int64_t>(node, "cycles", 2));
  c.velocity_scaling = param<double>(node, "velocity_scaling", 0.4);
  c.line_speed = param<double>(node, "line_speed", 0.2);

  if (c.can_radius <= 0.0 || c.can_height <= 0.0) {throw std::runtime_error("can size must be > 0");}
  if (c.cycles < 1) {throw std::runtime_error("cycles must be >= 1");}
  if (c.velocity_scaling <= 0.0 || c.velocity_scaling > 1.0) {
    throw std::runtime_error("velocity_scaling must be in (0, 1]");
  }

  auto log = node->get_logger();
  RCLCPP_INFO(log, "Pick (%.3f, %.3f) -> place (%.3f, %.3f), %d cycle(s)",
    c.pick_x, c.pick_y, c.place_x, c.place_y, c.cycles);
  RCLCPP_INFO(log, "Table top z=%.3f, can r=%.3f h=%.3f, obstacle %s, speed %.2f",
    c.table_top, c.can_radius, c.can_height, c.add_obstacle ? "on" : "off", c.velocity_scaling);
  return c;
}

// ============================================================================
// Poses
// ============================================================================
// TCP z axis pointing straight down, rotated by yaw about world z: q = Rz(yaw) * Rx(pi)
geometry_msgs::msg::Pose top_down(double x, double y, double z, double yaw)
{
  geometry_msgs::msg::Pose p;
  p.position.x = x;
  p.position.y = y;
  p.position.z = z;
  p.orientation.x = std::cos(yaw / 2.0);
  p.orientation.y = std::sin(yaw / 2.0);
  p.orientation.z = 0.0;
  p.orientation.w = 0.0;
  return p;
}

geometry_msgs::msg::Pose at(double x, double y, double z)
{
  geometry_msgs::msg::Pose p;
  p.position.x = x;
  p.position.y = y;
  p.position.z = z;
  p.orientation.w = 1.0;
  return p;
}

// ============================================================================
// Planning scene
// ============================================================================
CollisionObject make_box(const std::string & frame, const std::string & id,
  const std::array<double, 3> & size, const geometry_msgs::msg::Pose & centre)
{
  CollisionObject o;
  o.header.frame_id = frame;
  o.id = id;
  shape_msgs::msg::SolidPrimitive shape;
  shape.type = shape.BOX;
  shape.dimensions = {size[0], size[1], size[2]};
  o.primitives.push_back(shape);
  o.primitive_poses.push_back(centre);
  o.operation = o.ADD;
  return o;
}

CollisionObject make_cylinder(const std::string & frame, const std::string & id,
  double radius, double height, const geometry_msgs::msg::Pose & centre)
{
  CollisionObject o;
  o.header.frame_id = frame;
  o.id = id;
  shape_msgs::msg::SolidPrimitive shape;
  shape.type = shape.CYLINDER;
  shape.dimensions.resize(2);
  shape.dimensions[shape.CYLINDER_HEIGHT] = height;
  shape.dimensions[shape.CYLINDER_RADIUS] = radius;
  o.primitives.push_back(shape);
  o.primitive_poses.push_back(centre);
  o.operation = o.ADD;
  return o;
}

bool build_scene(PlanningSceneInterface & psi, const std::string & frame, const Config & c,
  const rclcpp::Logger & log)
{
  std::vector<CollisionObject> objs;
  objs.push_back(make_box(frame, "table", {c.table_size_x, c.table_size_y, 0.04},
    at(c.table_x, 0.0, c.table_top - 0.02)));
  objs.push_back(make_cylinder(frame, "can", c.can_radius, c.can_height,
    at(c.pick_x, c.pick_y, c.can_z())));

  if (c.add_obstacle) {
    const double h = c.obstacle_height;
    objs.push_back(make_box(frame, "obstacle", {0.05, 0.05, h},
      at((c.pick_x + c.place_x) / 2.0 + 0.05, (c.pick_y + c.place_y) / 2.0, c.table_top + h / 2.0)));
  } else {
    CollisionObject remove;                 // drop an obstacle left over from an earlier run
    remove.header.frame_id = frame;
    remove.id = "obstacle";
    remove.operation = remove.REMOVE;
    objs.push_back(remove);
  }

  if (!psi.applyCollisionObjects(objs)) {
    RCLCPP_ERROR(log, "Could not update the planning scene");
    return false;
  }
  RCLCPP_INFO(log, "Scene ready: table, can%s", c.add_obstacle ? ", obstacle" : "");
  return true;
}

// ============================================================================
// Gripper detection
// ============================================================================
struct Gripper
{
  std::string group;
  std::vector<std::string> joints;   // active joints only (mimic joints follow)
  std::vector<double> open, closed;
};

Gripper detect_gripper(const moveit::core::RobotModelConstPtr & model,
  const std::string & arm_group, const rclcpp::Logger & log)
{
  Gripper g;
  const auto * arm = model->getJointModelGroup(arm_group);
  if (!arm) {throw std::runtime_error("Arm group '" + arm_group + "' not in the SRDF");}

  std::string fallback;
  for (const auto * jmg : model->getJointModelGroups()) {
    if (jmg->getName() == arm_group || jmg->getActiveJointModelNames().empty()) {continue;}
    bool shares = false;
    for (const auto & j : jmg->getActiveJointModelNames()) {
      if (arm->hasJointModel(j)) {shares = true;}
    }
    if (shares) {continue;}
    const std::string n = jmg->getName();
    if (n.find("grip") != std::string::npos || n.find("hand") != std::string::npos) {
      g.group = n;
      break;
    }
    if (fallback.empty()) {fallback = n;}
  }
  if (g.group.empty()) {g.group = fallback;}
  if (g.group.empty()) {throw std::runtime_error("No gripper group found in the SRDF");}

  const auto * jmg = model->getJointModelGroup(g.group);
  g.joints = jmg->getActiveJointModelNames();

  auto from_named = [&](const std::vector<std::string> & names, std::vector<double> & out) {
      for (const auto & name : names) {
        std::map<std::string, double> vals;
        if (jmg->getVariableDefaultPositions(name, vals)) {
          out.clear();
          for (const auto & j : g.joints) {out.push_back(vals[j]);}
          RCLCPP_INFO(log, "  using SRDF state '%s'", name.c_str());
          return true;
        }
      }
      return false;
    };
  const bool has_open = from_named({"open", "opened"}, g.open);
  const bool has_closed = from_named({"close", "closed"}, g.closed);

  for (const auto & j : g.joints) {
    const auto & b = model->getVariableBounds(j);
    const bool upper_is_open = std::abs(b.max_position_) >= std::abs(b.min_position_);
    if (!has_open) {g.open.push_back(upper_is_open ? b.max_position_ : b.min_position_);}
    if (!has_closed) {g.closed.push_back(upper_is_open ? b.min_position_ : b.max_position_);}
  }

  RCLCPP_INFO(log, "Gripper group '%s' with %zu active joint(s)", g.group.c_str(), g.joints.size());
  for (size_t i = 0; i < g.joints.size(); ++i) {
    RCLCPP_INFO(log, "  %s: open %.4f  closed %.4f", g.joints[i].c_str(), g.open[i], g.closed[i]);
  }
  return g;
}

// ============================================================================
// Motion
// ============================================================================
// OMPL is randomised: a failed or invalid plan often succeeds on the next attempt
bool plan_and_execute(MoveGroupInterface & g, const rclcpp::Logger & log, const std::string & what,
  int attempts = 3)
{
  for (int i = 1; i <= attempts; ++i) {
    g.setStartStateToCurrentState();
    MoveGroupInterface::Plan plan;
    if (static_cast<bool>(g.plan(plan))) {
      if (!static_cast<bool>(g.execute(plan))) {
        RCLCPP_ERROR(log, "Executing '%s' failed", what.c_str());
        return false;
      }
      RCLCPP_INFO(log, "Done: %s", what.c_str());
      return true;
    }
    RCLCPP_WARN(log, "Planning '%s' failed (attempt %d/%d)", what.c_str(), i, attempts);
  }
  RCLCPP_ERROR(log, "Planning '%s' failed after %d attempts", what.c_str(), attempts);
  return false;
}

bool set_gripper(MoveGroupInterface & hand, const Gripper & g, const std::vector<double> & values,
  const rclcpp::Logger & log, const std::string & what)
{
  std::map<std::string, double> target;
  for (size_t i = 0; i < g.joints.size(); ++i) {target[g.joints[i]] = values[i];}
  hand.setJointValueTarget(target);
  return plan_and_execute(hand, log, what);
}

// pre-grasp above (x, y): the can is round, so any yaw works; try several
bool move_above(MoveGroupInterface & arm, const rclcpp::Logger & log, const Config & c,
  double x, double y)
{
  const double z = c.can_z() + c.approach;
  for (double yaw : {0.0, kPi / 2.0, -kPi / 2.0, kPi}) {
    arm.setStartStateToCurrentState();
    arm.setPoseTarget(top_down(x, y, z, yaw));
    MoveGroupInterface::Plan plan;
    if (static_cast<bool>(arm.plan(plan))) {
      arm.clearPoseTargets();
      RCLCPP_INFO(log, "Above (%.2f, %.2f) with yaw %.2f", x, y, yaw);
      return static_cast<bool>(arm.execute(plan));
    }
    RCLCPP_INFO(log, "  yaw %.2f not reachable, trying next", yaw);
  }
  arm.clearPoseTargets();
  RCLCPP_ERROR(log, "No reachable top-down pose above (%.2f, %.2f, %.2f)", x, y, z);
  return false;
}

// straight vertical TCP motion to height z: keeps x, y and orientation
bool vertical_line(MoveGroupInterface & arm, const rclcpp::Logger & log, double z, double speed)
{
  auto target = arm.getCurrentPose().pose;
  target.position.z = z;

  moveit_msgs::msg::RobotTrajectory traj;
  arm.setStartStateToCurrentState();
  const double fraction = arm.computeCartesianPath({target}, 0.005, 0.0, traj, true);
  RCLCPP_INFO(log, "Cartesian path to z=%.3f: %.1f%% achieved", z, fraction * 100.0);
  if (fraction < 0.99) {
    RCLCPP_ERROR(log, "Straight line blocked (collision or unreachable)");
    return false;
  }

  robot_trajectory::RobotTrajectory rt(arm.getRobotModel(), arm.getName());
  rt.setRobotTrajectoryMsg(*arm.getCurrentState(), traj);
  trajectory_processing::TimeOptimalTrajectoryGeneration totg;
  if (!totg.computeTimeStamps(rt, speed, speed)) {
    RCLCPP_ERROR(log, "Time parameterisation failed");
    return false;
  }
  rt.getRobotTrajectoryMsg(traj);
  return static_cast<bool>(arm.execute(traj));
}

// ============================================================================
// Grasping
// ============================================================================
struct Robot
{
  MoveGroupInterface & arm;
  MoveGroupInterface & hand;
  const Gripper & gripper;
  PlanningSceneInterface & psi;
  const Config & cfg;
  std::vector<std::string> touch_links;
  rclcpp::Logger log;
};

std::vector<std::string> gripper_touch_links(const moveit::core::RobotModelConstPtr & model,
  const std::string & gripper_group, const std::string & tcp_link)
{
  std::vector<std::string> links = model->getJointModelGroup(gripper_group)->getLinkModelNames();
  auto add = [&links](const std::string & l) {
      if (std::find(links.begin(), links.end(), l) == links.end()) {links.push_back(l);}
    };
  for (const auto & l : model->getLinkModelNames()) {
    if (l.find("gripper") != std::string::npos) {add(l);}
  }
  add(tcp_link);
  return links;
}

bool attach_can(Robot & r)
{
  moveit_msgs::msg::AttachedCollisionObject a;
  a.link_name = r.arm.getEndEffectorLink();
  a.object.header.frame_id = r.arm.getPlanningFrame();
  a.object.id = "can";
  a.object.operation = a.object.ADD;
  a.touch_links = r.touch_links;
  const bool ok = r.psi.applyAttachedCollisionObject(a);
  RCLCPP_INFO(r.log, "%s", ok ? "Can attached to the gripper" : "Attach failed");
  return ok;
}

bool detach_can(Robot & r)
{
  moveit_msgs::msg::AttachedCollisionObject a;
  a.link_name = r.arm.getEndEffectorLink();
  a.object.id = "can";
  a.object.operation = a.object.REMOVE;
  const bool ok = r.psi.applyAttachedCollisionObject(a);
  RCLCPP_INFO(r.log, "%s", ok ? "Can released into the world" : "Detach failed");
  return ok;
}

bool pick(Robot & r, double x, double y)
{
  const Config & c = r.cfg;
  RCLCPP_INFO(r.log, "--- pick at (%.2f, %.2f)", x, y);
  return set_gripper(r.hand, r.gripper, r.gripper.open, r.log, "open gripper")
         && move_above(r.arm, r.log, c, x, y)
         && vertical_line(r.arm, r.log, c.can_z(), c.line_speed)
         && attach_can(r)
         && set_gripper(r.hand, r.gripper, r.gripper.closed, r.log, "close gripper")
         && vertical_line(r.arm, r.log, c.can_z() + c.approach, c.line_speed);
}

bool place(Robot & r, double x, double y)
{
  const Config & c = r.cfg;
  RCLCPP_INFO(r.log, "--- place at (%.2f, %.2f)", x, y);
  return move_above(r.arm, r.log, c, x, y)
         && vertical_line(r.arm, r.log, c.can_z(), c.line_speed)
         && set_gripper(r.hand, r.gripper, r.gripper.open, r.log, "open gripper")
         && detach_can(r)
         && vertical_line(r.arm, r.log, c.can_z() + c.approach, c.line_speed);
}

// ============================================================================
// Main
// ============================================================================
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions opts;
  opts.automatically_declare_parameters_from_overrides(true);
  auto node = rclcpp::Node::make_shared("rethink_pick_place", opts);
  auto log = node->get_logger();

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  std::thread spinner([&exec]() {exec.spin();});

  int rc = 1;
  try {
    const Config cfg = load_config(node);

    RCLCPP_INFO(log, "Connecting to move_group ...");
    MoveGroupInterface arm(node, MoveGroupInterface::Options(cfg.arm_group),
      std::shared_ptr<tf2_ros::Buffer>(), rclcpp::Duration::from_seconds(10.0));
    if (!arm.getRobotModel()->hasLinkModel(cfg.tcp_link)) {
      throw std::runtime_error("TCP link '" + cfg.tcp_link + "' not in the robot model");
    }
    arm.setEndEffectorLink(cfg.tcp_link);
    arm.setMaxVelocityScalingFactor(cfg.velocity_scaling);
    arm.setMaxAccelerationScalingFactor(cfg.velocity_scaling);
    arm.setPlanningTime(5.0);
    arm.setNumPlanningAttempts(5);

    const Gripper gripper = detect_gripper(arm.getRobotModel(), cfg.arm_group, log);
    MoveGroupInterface hand(node, gripper.group);
    hand.setMaxVelocityScalingFactor(std::min(1.0, cfg.velocity_scaling * 1.25));
    hand.setMaxAccelerationScalingFactor(std::min(1.0, cfg.velocity_scaling * 1.25));

    PlanningSceneInterface psi;
    Robot robot{arm, hand, gripper, psi, cfg,
      gripper_touch_links(arm.getRobotModel(), gripper.group, cfg.tcp_link), log};

    if (!psi.getAttachedObjects({"can"}).empty()) {detach_can(robot);}

    auto start = arm.getCurrentState(10.0);
    if (!start) {throw std::runtime_error("No joint states. Is demo.launch.py running?");}
    std::vector<double> home;
    start->copyJointGroupPositions(cfg.arm_group, home);

    bool ok = build_scene(psi, arm.getPlanningFrame(), cfg, log);

    // the can travels back and forth: after each place, swap the two spots
    double ax = cfg.pick_x, ay = cfg.pick_y, bx = cfg.place_x, by = cfg.place_y;
    for (int i = 1; ok && i <= cfg.cycles; ++i) {
      RCLCPP_INFO(log, "=========== cycle %d/%d ===========", i, cfg.cycles);
      ok = pick(robot, ax, ay) && place(robot, bx, by);
      std::swap(ax, bx);
      std::swap(ay, by);
    }

    arm.setJointValueTarget(home);
    const bool home_ok = plan_and_execute(arm, log, "return home");
    rc = (ok && home_ok) ? 0 : 1;
    RCLCPP_INFO(log, "%s", rc == 0 ? "Pick and place finished." : "Pick and place aborted.");
  } catch (const std::exception & e) {
    RCLCPP_FATAL(log, "%s", e.what());
  }

  rclcpp::shutdown();
  spinner.join();
  return rc;
}