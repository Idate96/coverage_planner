#include <ros/package.h>
#include <ros/ros.h>

#include <grid_map_ros/grid_map_ros.hpp>
#include <string>

int main(int argc, char** argv) {
  // Initialize ROS.
  ros::init(argc, argv, "publish_map_bag");
  ros::NodeHandle node;

  // load grid map bag from file
  grid_map::GridMap map;
  std::string map_path;
  node.param("map_path", map_path, ros::package::getPath("coverage_planner") + "/data/map.bag");

  grid_map::GridMapRosConverter::loadFromBag(map_path, "/desired_elevation", map);

  std::string map_topic;
  std::string std_topic = "map";
  node.param("map_topic", map_topic, std_topic);
  // create publisher
  ros::Publisher map_pub = node.advertise<grid_map_msgs::GridMap>(map_topic, 1, true);

  // publish map every 2 seconds
  ros::Rate rate(2.0);
  while (node.ok()) {
    grid_map_msgs::GridMap map_msg;
    grid_map::GridMapRosConverter::toMessage(map, map_msg);
    map_pub.publish(map_msg);
    rate.sleep();
  }
}