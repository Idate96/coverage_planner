#include <ros/package.h>
#include <ros/ros.h>

#include <grid_map_core/GridMap.hpp>

#include "coverage_planner/ImageToGridmap.hpp"

int main(int argc, char** argv) {
  // Initialize node and publisher.
  ros::init(argc, argv, "image_to_gridmap");
  ros::NodeHandle nh("~");
  std::string layer_name = "elevation";
  ImageToGridmap imageToGridmap(nh, layer_name);

  ros::spin();
  return 0;
}
