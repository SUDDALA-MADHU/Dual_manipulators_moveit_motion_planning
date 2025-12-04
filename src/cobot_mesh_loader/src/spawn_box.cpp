#include <ros/ros.h>
#include <ros/package.h>
#include <gazebo_msgs/SpawnModel.h>
#include <fstream>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "spawn_box");
  ros::NodeHandle nh;

  ros::ServiceClient client =
      nh.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_sdf_model");

  ros::service::waitForService("/gazebo/spawn_sdf_model");

  gazebo_msgs::SpawnModel srv;
  srv.request.model_name = "box";

  // IMPORTANT: spawn relative to robot base
  srv.request.reference_frame = "base_link";

  std::string path =
      ros::package::getPath("cobot_mesh_loader") + "/models/box/model.sdf";
  std::ifstream file(path);

  srv.request.model_xml.assign(
    std::istreambuf_iterator<char>(file),
    std::istreambuf_iterator<char>());

  srv.request.initial_pose.position.x = 0.6;
  srv.request.initial_pose.position.y = 0.0;
  srv.request.initial_pose.position.z = 0.1;

  client.call(srv);
  ROS_INFO("Box spawned");
  

  return 0;
}
