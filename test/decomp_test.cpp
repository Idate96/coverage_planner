#include <gtest/gtest.h>
#include <ros/ros.h>

#include <Eigen/Dense>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include "coverage_planner/CellDecompBsd.hpp"
#include "grid_map_ros/grid_map_ros.hpp"

using namespace grid_map;

void set_obstacle(grid_map::GridMap& map);
// make the args accessible to the tests

auto main(int argc, char* argv[]) -> int {
  ros::init(argc, argv, "decomp_test");
  ros::NodeHandle nh("~");
  ros::Publisher publisher = nh.advertise<grid_map_msgs::GridMap>("grid_map", 1, true);

  GridMap map({"elevation", "normal_x", "normal_y", "normal_z"});
  set_obstacle(map);

  CellDecompBsd bsd(map);
  bsd.decompose("occupancy");
  // print all names of the map layers

  ros::Rate rate(30.0);
  while (nh.ok()) {
    ros::Time time = ros::Time::now();
    map.setTimestamp(time.toNSec());
    grid_map_msgs::GridMap message;
    GridMapRosConverter::toMessage(bsd.map_, message);
    publisher.publish(message);
    ROS_INFO_THROTTLE(1.0, "Grid map (timestamp %f) published.", message.info.header.stamp.toSec());
    rate.sleep();
  }
  // testing::InitGoogleTest(&argc, argv);
  // return RUN_ALL_TESTS();
}

void set_obstacle(grid_map::GridMap& map) {
  // Create a grid map
  // Initialize node and publisher.
  // Create grid map.
  map.setFrameId("map");
  map.setGeometry(Length(50, 50), 0.3, Position(0.0, -0.1));
  ROS_INFO("Created map with size %f x %f m (%i x %i cells).\n The center of the map is located at (%f, %f) in the %s frame.",
           map.getLength().x(), map.getLength().y(),
           map.getSize()(0), map.getSize()(1),
           map.getPosition().x(), map.getPosition().y(), map.getFrameId().c_str());

  // Work with grid map in a loop.
  for (GridMapIterator it(map); !it.isPastEnd(); ++it) {
    // Get position of the cell.
    Position position;
    map.getPosition(*it, position);
    // Set map value at the current position.
    map.at("elevation", *it) = 0;
    // Set map normal at the current position.
    map.at("normal_x", *it) = 0;
    map.at("normal_y", *it) = 0;
    map.at("normal_z", *it) = 1;
  }

  // Filter values for submap (iterators).
  map.add("occupancy", map.get("elevation"));
  Position topLeftCorner(10, 4);
  boundPositionToRange(topLeftCorner, map.getLength(), map.getPosition());
  Index startIndex;
  map.getIndex(topLeftCorner, startIndex);
  ROS_INFO_ONCE("Top left corner was limited from (1.0, 0.2) to (%f, %f) and corresponds to index (%i, %i).",
                topLeftCorner.x(), topLeftCorner.y(), startIndex(0), startIndex(1));

  Size size = (Length(12, 8) / map.getResolution()).cast<int>();
  ROS_INFO("Size of submap is %i x %i cells.", size(0), size(1));
  SubmapIterator it(map, startIndex, size);
  for (; !it.isPastEnd(); ++it) {
    Position currentPosition;
    map.getPosition(*it, currentPosition);
    map.at("occupancy", *it) = -1;
  }
}
