#pragma once 
#include <Eigen/Dense>
#include <map>

class Cell {
 public:
  Cell();
  Cell(int x_left, int x_right, int cell_id);
  // constructor from a decomposed map
  static std::map<int, Cell> from_layer(const Eigen::MatrixXi& decomposed_layer);
  ~Cell();

  // get corner coodinates
  std::pair<int, int> get_corner(int corner_id);
  // check if point is in the cell
  bool is_in_cell(int x, int y);
  // get center coordinates
  std::pair<int, int> get_center() const;

  // dictionary containing the cell's top edges
  std::map<int, int> top_edges_;
  // dictionary containing the cell's bottom edges
  std::map<int, int> bottom_edges_;
  // list containing the indexes of vertices on the left edge of the cell
  std::vector<int> left_vertices_;
  // list containing the indexes of vertices on the right edge of the cell
  std::vector<int> right_vertices_;

  int x_right_;
  int x_left_;
  int cell_id_;
};
