#include <gtest/gtest.h>

#include <Eigen/Dense>

#include "coverage_planner/CellDecompBsd.hpp"
#include "grid_map_ros/grid_map_ros.hpp"

using namespace Eigen;

TEST(test_bsd, test_slice_decomp_ones) {
  // test slice decomposition
  grid_map::GridMap map;
  grid_map::Matrix layer = Eigen::MatrixXf::Ones(10, 1);
  map.setGeometry(grid_map::Length(10, 1), 1.0);
  map.add("occupancy", layer);

  CellDecompBsd bsd(map);
  auto segments = bsd.getSegments(0);
  std::vector<std::pair<int, int>> expected;
  expected.push_back(std::make_pair(0, 10));

  EXPECT_EQ(segments, expected);
}

TEST(test_bsd, test_slice_decomp_one_slice) {
  // test slice decomposition
  grid_map::GridMap map;
  grid_map::Matrix layer = Eigen::MatrixXf::Zero(10, 1);
  layer.col(0) = VectorXf::Ones(10);
  map.setGeometry(grid_map::Length(10, 1), 1.0);
  map.add("occupancy", layer);

  CellDecompBsd bsd(map);
  auto segments = bsd.getSegments(0);
  // print segments for debugging
  // for (auto segment : segments) {
  //   std::cout << segment.first << " " << segment.second << std::endl;
  // }

  std::vector<std::pair<int, int>> expected;
  expected.push_back(std::make_pair(0, 10));

  EXPECT_EQ(segments, expected);
}

TEST(test_bsd, test_slice_decomp_two_slices) {
  // test slice decomposition
  grid_map::GridMap map;

  Eigen::MatrixXf layer = Eigen::MatrixXf::Zero(15, 2);

  layer.col(0).segment(1, 5).setConstant(1);
  layer.col(0).segment(10, 14).setConstant(1);
  map.setGeometry(grid_map::Length(15, 1), 1.0);
  map.add("occupancy", layer);

  CellDecompBsd bsd(map);

  auto segments = bsd.getSegments(0);

  std::vector<std::pair<int, int>> expected;
  expected.push_back(std::make_pair(0, 5));
  expected.push_back(std::make_pair(11, 15));
}

TEST(test_bsd, test_get_slice_adj_ones) {
  grid_map::GridMap map;
  // this boiler plate must be removed or include assertion
  grid_map::Matrix layer = Eigen::MatrixXf::Zero(15, 1);
  map.setGeometry(grid_map::Length(15, 1), 1.0);
  map.add("occupancy", layer);
  CellDecompBsd bsd(map);

  std::vector<std::pair<int, int>> segments_0 = {std::make_pair(0, 1)};
  std::vector<std::pair<int, int>> segments_1 = {std::make_pair(0, 1)};
  Eigen::MatrixXi adj_matrix = bsd.getSliceAdj(segments_0, segments_1);

  // print adj_matrix
  Eigen::MatrixXi expected = Eigen::MatrixXi::Ones(1, 1);
  // EXPECT_EQ(adj_matrix, expected);
  // ASSERT_TRUE(adj_matrix.isApprox(expected));
  ASSERT_TRUE(expected.isApprox(adj_matrix));
  ASSERT_TRUE(adj_matrix.isApprox(expected));
}

/*!
 * @brief Test the getSliceAdj function
 * with a case with no overlap
 */
TEST(test_bsd, test_get_slice_adj_one_zeros) {
  grid_map::GridMap map;
  // this boiler plate must be removed or include assertion
  grid_map::Matrix layer = Eigen::MatrixXf::Zero(15, 1);
  map.setGeometry(grid_map::Length(15, 1), 1.0);
  map.add("occupancy", layer);
  CellDecompBsd bsd(map);

  std::vector<std::pair<int, int>> segments_0 = {std::make_pair(0, 1)};
  std::vector<std::pair<int, int>> segments_1 = {std::make_pair(2, 3)};
  Eigen::MatrixXi adj_matrix = bsd.getSliceAdj(segments_0, segments_1);
  Eigen::MatrixXi expected = Eigen::MatrixXi::Zero(1, 1);

  ASSERT_TRUE(adj_matrix.isApprox(expected));
}

TEST(test_bsd, test_get_slice_adj_two) {
  grid_map::GridMap map;
  // this boiler plate must be removed or include assertion
  grid_map::Matrix layer = Eigen::MatrixXf::Zero(15, 1);
  map.setGeometry(grid_map::Length(15, 1), 1.0);
  map.add("occupancy", layer);
  CellDecompBsd bsd(map);

  std::vector<std::pair<int, int>> segments_0 = {std::make_pair(0, 2), std::make_pair(7, 10)};
  std::vector<std::pair<int, int>> segments_1 = {std::make_pair(1, 3), std::make_pair(5, 7)};
  Eigen::MatrixXi adj_matrix = bsd.getSliceAdj(segments_0, segments_1);
  Eigen::MatrixXi expected = Eigen::MatrixXi::Zero(2, 2);
  expected(0, 0) = 1;
  ASSERT_TRUE(adj_matrix.isApprox(expected));
}

TEST(test_bsd, test_decomp_exist) {
  grid_map::GridMap map;
  grid_map::Matrix layer = Eigen::MatrixXf::Zero(15, 1);
  map.setGeometry(grid_map::Length(15, 1), 1.0);
  map.add("occupancy", layer);
  CellDecompBsd bsd(map);
  bsd.decompose("occupancy");
  std::vector<std::string> layers = bsd.map_.getLayers();
  // check if layer "decomposed" is in layers
  EXPECT_TRUE(std::find(layers.begin(), layers.end(), "decomposed") != layers.end());
}
