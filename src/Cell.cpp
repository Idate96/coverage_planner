#include "coverage_planner/Cell.hpp"

#include <set>

Cell::Cell(int x_left, int x_right, int cell_id) : x_left_(x_left), x_right_(x_right), cell_id_(cell_id) {}

Cell::Cell() : x_left_(0), x_right_(0), cell_id_(0) {}

Cell::~Cell() {}

std::map<int, Cell> Cell::from_layer(const Eigen::MatrixXi& layer) {
  std::vector<int> layer_cells_idx(layer.data(), layer.data() + layer.size());
  // create a set with unique indices
  std::set<int> layer_cells_idx_set(layer_cells_idx.begin(), layer_cells_idx.end());
  // dictionary of cells indexed by the cell id
  std::map<int, Cell> cells;
  // construct the boudaries of the cells by iterating over the layer coordinates
  for (int i = 0; i < layer.rows(); i++) {
    for (int j = 0; j < layer.cols(); j++) {
      int cell_id = layer(i, j);
      Cell cell = cells[cell_id];
      if (i < cell.x_left_) {
        cell.x_left_ = i;
        cell.left_vertices_.push_back(j);
      } else if (i == cell.x_left_) {
        cell.left_vertices_.push_back(j);
      }
      if (i > cell.x_right_) {
        cell.x_right_ = i;
        cell.right_vertices_.push_back(j);
      } else if (i == cell.x_right_) {
        cell.right_vertices_.push_back(j);
      }

      // check if i is part of the cell bottom edge, if not add it
      if (cell.bottom_edges_.find(i) == cell.bottom_edges_.end()) {
        cell.bottom_edges_.insert({i, j});
      } else if (cell.bottom_edges_[i] > j) {
        cell.bottom_edges_[i] = j;
      }

      // check if i is part of the cell top edge, if not add it
      if (cell.top_edges_.find(i) == cell.top_edges_.end()) {
        cell.top_edges_.insert({i, j});
      } else if (cell.top_edges_[i] < j) {
        cell.top_edges_[i] = j;
      }
    }
  }
  return cells;
}

std::pair<int, int> Cell::get_corner(int corner_id) {
  switch (corner_id) {
    case 0:
      return std::make_pair(x_left_, bottom_edges_[x_left_]);
    case 1:
      return std::make_pair(x_left_, top_edges_[x_left_]);
    case 2:
      return std::make_pair(x_right_, top_edges_[x_right_]);
    case 3:
      return std::make_pair(x_right_, bottom_edges_[x_right_]);
    default:
      throw std::runtime_error("Invalid corner id");
  }
}

bool Cell::is_in_cell(int x, int y) {
  return x >= x_left_ && x <= x_right_ && y >= bottom_edges_.at(x) && y <= top_edges_.at(x);
}

std::pair<int, int> Cell::get_center() const {
  return std::make_pair((x_left_ + x_right_) / 2, (bottom_edges_.at(x_left_) + top_edges_.at(x_right_)) / 2);
}
