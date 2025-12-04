#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <gazebo_ros_link_attacher/Attach.h>
#include <gazebo_msgs/GetModelState.h>
#include <geometry_msgs/Pose.h>
#include <tf2/LinearMath/Quaternion.h>
#include <trajectory_msgs/JointTrajectory.h>

// ========================================
// Get Box Position from Gazebo
// ========================================
geometry_msgs::Pose getBoxPose(ros::NodeHandle& nh)
{
  ros::ServiceClient client = 
      nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
  
  ros::service::waitForService("/gazebo/get_model_state", ros::Duration(5.0));

  gazebo_msgs::GetModelState srv;
  srv.request.model_name = "box";
  srv.request.relative_entity_name = "world";

  geometry_msgs::Pose pose;
  if (client.call(srv) && srv.response.success)
  {
    pose = srv.response.pose;
    ROS_INFO("Box position: x=%.3f, y=%.3f, z=%.3f",
             pose.position.x, pose.position.y, pose.position.z);
  }
  else
  {
    ROS_ERROR("Failed to get box position from Gazebo!");
  }
  return pose;
}

// ========================================
// Control Gripper
// ========================================
void controlGripper(ros::Publisher& pub, double position)
{
  trajectory_msgs::JointTrajectory traj;
  traj.joint_names.push_back("finger_joint");

  trajectory_msgs::JointTrajectoryPoint point;
  point.positions.push_back(position);
  point.time_from_start = ros::Duration(1.0);
  traj.points.push_back(point);

  pub.publish(traj);
  ros::Duration(1.5).sleep();
}

// ========================================
// Move to Pose using MoveIt
// ========================================
bool moveToPose(moveit::planning_interface::MoveGroupInterface& move_group,
                geometry_msgs::Pose& target_pose)
{
  move_group.setPoseTarget(target_pose);
  
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  
  if (success)
  {
    ROS_INFO("Plan found! Executing...");
    move_group.execute(plan);
    return true;
  }
  else
  {
    ROS_WARN("Planning failed!");
    return false;
  }
}

// ========================================
// MAIN
// ========================================
int main(int argc, char** argv)
{
  ros::init(argc, argv, "pick_place_moveit");
  ros::NodeHandle nh;
  
  // Start async spinner (required for MoveIt)
  ros::AsyncSpinner spinner(2);
  spinner.start();

  // Wait for everything to start
  ROS_INFO("Waiting for system to initialize...");
  ros::Duration(2.0).sleep();

  // ========================================
  // Setup MoveIt
  // ========================================
  // CHANGE "arm" to your planning group name!
  // Check your MoveIt config or run: rosrun moveit_commander moveit_commander_cmdline.py
  // Then type: group_names
  
  std::string planning_group = "arm";  // <-- CHANGE THIS to your group name
  
  moveit::planning_interface::MoveGroupInterface move_group(planning_group);
  moveit::planning_interface::PlanningSceneInterface planning_scene;

  // Print some info
  ROS_INFO("Planning frame: %s", move_group.getPlanningFrame().c_str());
  ROS_INFO("End effector link: %s", move_group.getEndEffectorLink().c_str());
  ROS_INFO("Available Planning Groups:");
  std::vector<std::string> group_names = move_group.getJointModelGroupNames();
  for (const auto& name : group_names)
  {
    ROS_INFO("  - %s", name.c_str());
  }

  // Set planning parameters
  move_group.setPlanningTime(10.0);
  move_group.setNumPlanningAttempts(10);
  move_group.setGoalPositionTolerance(0.01);
  move_group.setGoalOrientationTolerance(0.05);
  move_group.setMaxVelocityScalingFactor(0.5);
  move_group.setMaxAccelerationScalingFactor(0.5);

  // ========================================
  // Setup Gripper Publisher
  // ========================================
  ros::Publisher gripper_pub = 
      nh.advertise<trajectory_msgs::JointTrajectory>("/arm_controller/command", 1);
  ros::Duration(0.5).sleep();

  // ========================================
  // Setup Link Attacher Services
  // ========================================
  ros::ServiceClient attach_client =
      nh.serviceClient<gazebo_ros_link_attacher::Attach>("/link_attacher_node/attach");
  ros::ServiceClient detach_client =
      nh.serviceClient<gazebo_ros_link_attacher::Attach>("/link_attacher_node/detach");

  ROS_INFO("Waiting for link_attacher services...");
  if (!ros::service::waitForService("/link_attacher_node/attach", ros::Duration(10.0)))
  {
    ROS_ERROR("Link attacher service not available! Make sure world file has the plugin.");
    return 1;
  }
  ROS_INFO("Link attacher services ready!");

  // ========================================
  // Get Box Position from Gazebo
  // ========================================
  geometry_msgs::Pose box_pose = getBoxPose(nh);
  
  // Box parameters (adjust to match your box model)
  double box_height = 0.05;  // Half-height of box
  double approach_height = 0.15;  // Height above box for approach

  // Place location (adjust as needed)
  double place_x = 0.3;
  double place_y = -0.3;

  ROS_INFO("========================================");
  ROS_INFO("PICK AND PLACE STARTING");
  ROS_INFO("Box at: (%.3f, %.3f, %.3f)", 
           box_pose.position.x, box_pose.position.y, box_pose.position.z);
  ROS_INFO("Place at: (%.3f, %.3f)", place_x, place_y);
  ROS_INFO("========================================");

  // Create target pose with gripper pointing down
  geometry_msgs::Pose target_pose;
  tf2::Quaternion q;
  q.setRPY(M_PI, 0, 0);  // Gripper pointing down
  target_pose.orientation.x = q.x();
  target_pose.orientation.y = q.y();
  target_pose.orientation.z = q.z();
  target_pose.orientation.w = q.w();

  // ========================================
  // Step 1: Go to Home Position
  // ========================================
  ROS_INFO("Step 1: Going to home position...");
  move_group.setNamedTarget("home");  // or "ready", "zero" - check your SRDF
  move_group.move();
  ros::Duration(1.0).sleep();

  // ========================================
  // Step 2: Open Gripper
  // ========================================
  ROS_INFO("Step 2: Opening gripper...");
  controlGripper(gripper_pub, 0.7);  // Open position

  // ========================================
  // Step 3: Move Above Box (Pre-grasp)
  // ========================================
  ROS_INFO("Step 3: Moving above box...");
  target_pose.position.x = box_pose.position.x;
  target_pose.position.y = box_pose.position.y;
  target_pose.position.z = box_pose.position.z + approach_height;
  
  if (!moveToPose(move_group, target_pose))
  {
    ROS_ERROR("Failed to reach pre-grasp position!");
    return 1;
  }
  ros::Duration(0.5).sleep();

  // ========================================
  // Step 4: Move Down to Grasp Position
  // ========================================
  ROS_INFO("Step 4: Moving down to grasp...");
  target_pose.position.z = box_pose.position.z + box_height + 0.02;
  
  // Try Cartesian path for linear move
  std::vector<geometry_msgs::Pose> waypoints;
  waypoints.push_back(target_pose);
  
  moveit_msgs::RobotTrajectory trajectory;
  double fraction = move_group.computeCartesianPath(waypoints, 0.01, 0.0, trajectory);
  
  if (fraction > 0.9)
  {
    move_group.execute(trajectory);
  }
  else
  {
    // Fallback to regular planning
    moveToPose(move_group, target_pose);
  }
  ros::Duration(0.5).sleep();

  // ========================================
  // Step 5: Close Gripper
  // ========================================
  ROS_INFO("Step 5: Closing gripper...");
  controlGripper(gripper_pub, 0.35);  // Closed position

  // ========================================
  // Step 6: Attach Object in Gazebo
  // ========================================
  ROS_INFO("Step 6: Attaching object...");
  gazebo_ros_link_attacher::Attach attach_srv;
  attach_srv.request.model_name_1 = "medrob";
  attach_srv.request.link_name_1  = "end_effector";
  attach_srv.request.model_name_2 = "box";
  attach_srv.request.link_name_2  = "link";

  if (attach_client.call(attach_srv))
  {
    ROS_INFO("Attach service called: %s", attach_srv.response.ok ? "SUCCESS" : "FAILED");
  }
  ros::Duration(0.5).sleep();

  // ========================================
  // Step 7: Lift Object
  // ========================================
  ROS_INFO("Step 7: Lifting object...");
  target_pose.position.z = box_pose.position.z + approach_height;
  moveToPose(move_group, target_pose);
  ros::Duration(0.5).sleep();

  // ========================================
  // Step 8: Move to Place Location
  // ========================================
  ROS_INFO("Step 8: Moving to place location...");
  target_pose.position.x = place_x;
  target_pose.position.y = place_y;
  target_pose.position.z = box_pose.position.z + approach_height;
  moveToPose(move_group, target_pose);
  ros::Duration(0.5).sleep();

  // ========================================
  // Step 9: Lower Object
  // ========================================
  ROS_INFO("Step 9: Lowering object...");
  target_pose.position.z = box_pose.position.z + box_height + 0.02;
  
  waypoints.clear();
  waypoints.push_back(target_pose);
  fraction = move_group.computeCartesianPath(waypoints, 0.01, 0.0, trajectory);
  
  if (fraction > 0.9)
  {
    move_group.execute(trajectory);
  }
  else
  {
    moveToPose(move_group, target_pose);
  }
  ros::Duration(0.5).sleep();

  // ========================================
  // Step 10: Detach Object
  // ========================================
  ROS_INFO("Step 10: Detaching object...");
  gazebo_ros_link_attacher::Attach detach_srv;
  detach_srv.request = attach_srv.request;
  
  if (detach_client.call(detach_srv))
  {
    ROS_INFO("Detach service called: %s", detach_srv.response.ok ? "SUCCESS" : "FAILED");
  }
  ros::Duration(0.3).sleep();

  // ========================================
  // Step 11: Open Gripper
  // ========================================
  ROS_INFO("Step 11: Opening gripper...");
  controlGripper(gripper_pub, 0.7);

  // ========================================
  // Step 12: Retreat
  // ========================================
  ROS_INFO("Step 12: Retreating...");
  target_pose.position.z = box_pose.position.z + approach_height;
  moveToPose(move_group, target_pose);

  // ========================================
  // Step 13: Return Home
  // ========================================
  ROS_INFO("Step 13: Returning home...");
  move_group.setNamedTarget("home");
  move_group.move();

  ROS_INFO("========================================");
  ROS_INFO("PICK AND PLACE COMPLETED!");
  ROS_INFO("========================================");

  ros::shutdown();
  return 0;
}
