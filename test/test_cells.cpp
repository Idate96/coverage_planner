#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <map>

#include "coverage_planner/Cell.hpp"

/*!
 * Converts Eigen vectors into std::vectors
 */
template <typename T>
std::vector<T> eigen2std(const Eigen::Matrix<T, Eigen::Dynamic, 1>& vec) {
  std::vector<T> std_vec(vec.data(), vec.data() + vec.size());
  return std_vec;
}

TEST(test_cell, test_unique_cell) {
  Eigen::MatrixXi simple_cells = Eigen::MatrixXi::Zero(10, 10);
  simple_cells(15, 0) = 1;
  // print simple_cells
  std::map<int, Cell> cells = Cell::from_layer(simple_cells);

  // expected cell
  Cell expected_cell = Cell(0, 10, 0);
  // ad as left vertices range of numbers from 0 to 10
  Eigen::VectorXi left_vertices = Eigen::VectorXi::LinSpaced(10, 0, 10);
  expected_cell.left_vertices_ = eigen2std(left_vertices);
  // ad as right vertices range of numbers from 0 to 10
  Eigen::VectorXi right_vertices = Eigen::VectorXi::LinSpaced(10, 0, 10);
  expected_cell.right_vertices_ = eigen2std(right_vertices);

  expected_cell.x_left_ = 0;
  expected_cell.x_right_ = 9;

  // fill top and bottom vertices
  for (int i = 0; i < 10; i++) {
    expected_cell.top_edges_.insert({i, 9});
    expected_cell.bottom_edges_.insert({i, 0});
  }
}
