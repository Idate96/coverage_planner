#include <ros/ros.h>

#include <grid_map_ros/grid_map_ros.hpp>
#include <string>

int main(int argc, char** argv) {
  ros::init(argc, argv, "load_map_bag");
  ros::NodeHandle nh;
  ros::Publisher map_pub = nh.advertise<grid_map_msgs::GridMap>("grid_map", 1);

  grid_map::GridMap map;
  std::string map_path = nh.param<std::string>("map_path", "map.bag");

  ROS_INFO("Loading map from %s", map_path.c_str());
  grid_map::GridMapRosConverter::loadFromBag(map_path, "grid_map", map);
  // print map layers
  ROS_INFO("Map layers:");
  for (auto& layer : map.getLayers()) {
    ROS_INFO("  %s", layer.c_str());
  }
  // extract eigen matrix from map layer elevation
  grid_map::Matrix elevation = map.get("occupancy");
  double max_elevation = elevation.array().maxCoeff();
  ROS_INFO("Max elevation: %f", max_elevation);

  // convert the map into a message
  grid_map_msgs::GridMap message;
  grid_map::GridMapRosConverter::toMessage(map, message);

  while (ros::ok()) {
    // publish the map message
    map_pub.publish(message);
    ros::spinOnce();
  }
}