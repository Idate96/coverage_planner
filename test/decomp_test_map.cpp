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

class CellDecompTest : public ::testing::Test {
 protected:
  ros::NodeHandle nh_;
  ros::Subscriber gridMapSubscriber_;
  ros::Publisher decompGridMapPublisher_;
  // return RUN_ALL_TESTS();

  void SetUp() override {
    gridMapSubscriber_ = nh_.subscribe("image_to_gridmap/grid_map", 10, &CellDecompTest::mapCallBack, this);
    decompGridMapPublisher_ = nh_.advertise<grid_map_msgs::GridMap>("grid_map_decomp", 1, true);
  };

  void mapCallBack(const grid_map_msgs::GridMap& msg);
};

void CellDecompTest::mapCallBack(const grid_map_msgs::GridMap& msg) {
  GridMap map;
  grid_map::GridMapRosConverter::fromMessage(msg, map, {"elevation"});
  CellDecompBsd bsd(map);
  bsd.decompose("elevation");
  grid_map_msgs::GridMap mapMessage;
  grid_map::GridMapRosConverter::toMessage(bsd.map_, mapMessage);
  decompGridMapPublisher_.publish(mapMessage);
}

TEST_F(CellDecompTest, testDecompose) {
  auto topic = gridMapSubscriber_.getTopic();
  EXPECT_EQ(topic, "/image_to_gridmap/grid_map");
}

// auto main(int argc, char* argv[]) -> int {
//   ros::init(argc, argv, "decomp_test_map");
//   testing::InitGoogleTest(&argc, argv);
//   return RUN_ALL_TESTS();
// }
