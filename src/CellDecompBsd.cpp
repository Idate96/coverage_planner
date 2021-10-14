#include "coverage_planner/CellDecompBsd.hpp"

#include "math.h"

CellDecompBsd::CellDecompBsd(grid_map::GridMap& map) : map_(map) {
  occupancy_map_ = map_["occupancy"];
}

CellDecompBsd::~CellDecompBsd() {}

std::vector<std::pair<int, int>> CellDecompBsd::getSegments(int col) {
  std::vector<std::pair<int, int>> segments;
  int row = 0;
  bool occupied_cell = false;

  Eigen::VectorXf slice = occupancy_map_.col(col);

  // check when the cell is occupied and split the slice into segments
  for (int i = 0; i < slice.size(); i++) {
    if (slice(i) == 0 and !occupied_cell) {
      segments.push_back(std::make_pair(row, i - 1));
      occupied_cell = true;
    }
    if (slice(i) == 1 and occupied_cell) {
      row = i;
      occupied_cell = false;
    }
  }
  // add last segment if not already added
  if (!occupied_cell) {
    segments.push_back(std::make_pair(row, slice.size()));
  }
  return segments;
}

Eigen::MatrixXi CellDecompBsd::getSliceAdj(std::vector<std::pair<int, int>>& slice_1,
                                           std::vector<std::pair<int, int>>& slice_2) {
  Eigen::MatrixXi adj_matrix(slice_1.size(), slice_2.size());
  adj_matrix.setZero();
  for (int i = 0; i < slice_1.size(); i++) {
    for (int j = 0; j < slice_2.size(); j++) {
      int len_overlapping =
          std::min(slice_1[i].second, slice_2[j].second) - std::max(slice_1[i].first, slice_2[j].first);
      if (len_overlapping > 0) {
        adj_matrix(i, j) = 1;
      }
    }
  }
  return adj_matrix;
}

void CellDecompBsd::decompose(const std::string& occupancy_layer) {
  // get occupancy map
  occupancy_map_ = map_[occupancy_layer];
  // make empty layer of the same size os the coverage map_
  auto size = map_.getSize();

  std::vector<std::pair<int, int>> previous_segments;
  int num_previous_segments = 0;

  // we store the cell id associated to each segment
  std::vector<int> previous_cells;
  std::vector<int> current_cells;
  int num_cells = 1;

  for (int col_id = 0; col_id < size(1); col_id++) {
    Eigen::VectorXf slice = occupancy_map_.col(col_id);
    std::vector<std::pair<int, int>> segments = getSegments(col_id);
    int num_segments = segments.size();

    // at the beginning
    if (num_previous_segments == 0) {
      for (int i = 0; i < num_segments; i++) {
        current_cells.push_back(num_cells);
        num_cells++;
      }
    } else {
      std::fill(current_cells.begin(), current_cells.end(), 0);
      Eigen::MatrixXi adj_matrix = getSliceAdj(previous_segments, segments);

      for (int i = 0; i < num_previous_segments; i++) {
        if (adj_matrix.row(i).sum() == 1) {
          for (int j = 0; j < num_segments; j++) {
            if (adj_matrix(i, j) == 1) {
              current_cells[j] = previous_cells[i];
            }
          }
        }
      }

      for (int i = 0; i < num_segments; i++) {
        if (adj_matrix.col(i).sum() > 1 || adj_matrix.col(i).sum() == 0) {
          current_cells[i] = num_cells;
          num_cells++;
        }
      }
    }

    Eigen::MatrixXf decomp_map = Eigen::MatrixXf::Zero(size(0), size(1));
    // assign to the map the value of the current cells
    for (int i = 0; i < current_cells.size(); i++) {
      for (int j = segments[i].first; j <= segments[i].second; j++) {
        decomp_map(j, col_id) = current_cells[i];
      }
    }
    map_.add("decomposed", decomp_map);

    previous_segments = segments;
    previous_cells = current_cells;
    num_previous_segments = num_segments;
  }
}

Eigen::MatrixXi CellDecompBsd::getCellsAdj() {
  int num_cells = map_["decomposed"].maxCoeff();
  Eigen::MatrixXi adj_matrix(num_cells, num_cells);
  adj_matrix.setZero();
  std::vector<std::pair<int, int>> previous_segments;

  for (int col_id = 0; col_id < map_.getSize()(1); col_id++) {
    Eigen::VectorXf slice = map_["decomposed"].col(col_id);
    std::vector<std::pair<int, int>> segments = getSegments(col_id);

    Eigen::MatrixXi adj_matrix_slice = getSliceAdj(previous_segments, segments);

    // iterate through the previous segments
    for (int i = 0; i < previous_segments.size(); i++) {
      // iterate through the current segments
      for (int j = 0; j < segments.size(); j++) {
        // if the two segments are adjacent
        if (adj_matrix_slice(i, j) == 1) {
          // check if the cell in map_["composed] id is the same for both segments
          int idx_i = map_["decomposed"](previous_segments[i].first, col_id);
          int idx_j = map_["decomposed"](segments[j].first, col_id);
          // if they don't belong to the same cell set the adjacency
          if (idx_i != idx_j) {
            adj_matrix(idx_i, idx_j) = 1;
            adj_matrix(idx_j, idx_i) = 1;
          }
        }
      }
    }
    previous_segments = segments;
  }
  return adj_matrix;
}
