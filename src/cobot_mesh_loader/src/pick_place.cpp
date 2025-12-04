#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>
#include <geometric_shapes/shape_operations.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pick_place_demo");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // ---------------------------------------------------------
  //  YOUR MOVEIT CONFIGURATION NAMES
  // ---------------------------------------------------------
  static const std::string PLANNING_GROUP = "arm_1";       
  static const std::string EE_LINK = "eff_1";      

  // Initialize MoveIt interfaces
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  moveit::planning_interface::MoveGroupInterface::Plan plan;

  ROS_INFO("Pick & Place Demo Started");

  // ---------------------------------------------------------
  // 1. ADD OBJECT TO THE SCENE
  // ---------------------------------------------------------
  moveit_msgs::CollisionObject object;
  object.header.frame_id = move_group.getPlanningFrame();
  object.id = "cylinder";

  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions = {0.1, 0.10, 0.200};   // 20cm box

  geometry_msgs::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x = 0.5;
  box_pose.position.y = 0.0;
  box_pose.position.z = 0.10;                  // sits on table

  object.primitives.push_back(primitive);
  object.primitive_poses.push_back(box_pose);
  object.operation = moveit_msgs::CollisionObject::ADD;

  planning_scene_interface.applyCollisionObject(object);
  ROS_INFO("Object Added to Scene");
  ros::Duration(1.0).sleep();

  // ---------------------------------------------------------
  // 2. MOVE ABOVE THE OBJECT
  // ---------------------------------------------------------
  geometry_msgs::Pose approach_pose;
  approach_pose.orientation.w = 1.0;
  approach_pose.position.x = 0.5;
  approach_pose.position.y = 0.0;
  approach_pose.position.z = 0.30;    // 30 cm above object

  move_group.setPoseTarget(approach_pose);
  move_group.setPlanningTime(5.0);

  if (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS)
  {
    move_group.move();
    ROS_INFO("Moved above object");
  }

  // ---------------------------------------------------------
  // 3. MOVE DOWN TO PICK HEIGHT
  // ---------------------------------------------------------
  geometry_msgs::Pose pick_pose = approach_pose;
  pick_pose.position.z = 0.18;    // just above box

  move_group.setPoseTarget(pick_pose);

  if (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS)
  {
    move_group.move();
    ROS_INFO("Reached pick position");
  }

  // ---------------------------------------------------------
  // 4. ATTACH OBJECT TO GRIPPER
  // ---------------------------------------------------------
  ROS_INFO("Attaching object to gripper...");
  move_group.attachObject("cylinder", EE_LINK);
  ros::Duration(1.0).sleep();

  // ---------------------------------------------------------
  // 5. LIFT THE OBJECT
  // ---------------------------------------------------------
  geometry_msgs::Pose lift_pose = pick_pose;
  lift_pose.position.z += 0.20;  // lift 20 cm

  move_group.setPoseTarget(lift_pose);

  if (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS)
  {
    move_group.move();
    ROS_INFO("Object lifted");
  }

  // ---------------------------------------------------------
  // 6. MOVE TO PLACE LOCATION
  // ---------------------------------------------------------
  geometry_msgs::Pose place_pose = lift_pose;
  place_pose.position.x = 0.0;
  place_pose.position.y = 0.5;   // Move left side

  move_group.setPoseTarget(place_pose);

  if (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS)
  {
    move_group.move();
    ROS_INFO("Moved to place location");
  }

  // ---------------------------------------------------------
  // 7. LOWER THE OBJECT
  // ---------------------------------------------------------
  geometry_msgs::Pose lower_pose = place_pose;
  lower_pose.position.z = 0.12;   // lower near table

  move_group.setPoseTarget(lower_pose);
  move_group.move();
  ROS_INFO("Lowered object");

  // ---------------------------------------------------------
  // 8. DETACH OBJECT
  // ---------------------------------------------------------
  ROS_INFO("Detaching object...");
  move_group.detachObject("box");
  ros::Duration(1.0).sleep();

  // ---------------------------------------------------------
  // 9. REMOVE OBJECT FROM SCENE
  // ---------------------------------------------------------
  ROS_INFO("Removing object...");
  planning_scene_interface.removeCollisionObjects({"box"});
  ros::Duration(1.0).sleep();

  ROS_INFO("Pick & Place Demo Finished Successfully!");
  ros::shutdown();
  return 0;
}
