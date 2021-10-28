/*!
 * This files contains a function to create a simple desired elevation map from an occupancy grid
 * and current elevation It loads the gridmap from a bag file
 */
#pragma once

// ROS
#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include <grid_map_ros/grid_map_ros.hpp>

int main(int argc, char** argv) {
  ros::init(argc, argv, "desired_map");
  ros::NodeHandle nh;

  std::string map_path;
  nh.param<std::string>("map_path", map_path,
                        ros::package::getPath("coverage_planner") + "/data/map.bag");

  std::string desired_map_path;
  nh.param<std::string>("desired_map_path", desired_map_path,
                        ros::package::getPath("coverage_planner") + "/data/map_desired.bag");

  double desired_elevation_change;
  nh.param<double>("desired_elevation_change", desired_elevation_change, 0.0);

  // Load map
  grid_map::GridMap map;
  grid_map::GridMapRosConverter::loadFromBag(map_path, "grid_map", map);
  // add layer desired elevation to map
  map.add("desired_elevation", 0.0);

  // sanity check
  auto layers = map.getLayers();
  assert(std::find(layers.begin(), layers.end(), "elevation") != layers.end());
  assert(std::find(layers.begin(), layers.end(), "occupancy") != layers.end());

  // iterate over the map and where the occupancy is 0 subtract to the elevation the desired
  // elevation change
  int count = 0;
  int total_count = 0;
  for (grid_map::GridMapIterator iterator(map); !iterator.isPastEnd(); ++iterator) {
    if (std::abs(map.at("occupancy", *iterator)) < 0.9) {
      map.at("desired_elevation", *iterator) =
          map.at("elevation", *iterator) - desired_elevation_change;
      count++;
    }
    total_count++;
  }

  ROS_INFO_STREAM("Created desired elevation map with " << count << " points"
                                                        << " out of " << total_count);
  // Create desired map
  grid_map::GridMapRosConverter::saveToBag(map, desired_map_path, "/desired_elevation");
}