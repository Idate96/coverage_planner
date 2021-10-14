#pragma once
#include <Eigen/Dense>
#include <grid_map_ros/grid_map_ros.hpp>
/*!
 * The Bsd class implements the boustrophedon decomposition 
 * of the coverage area for path planning. 
*/
class CellDecompBsd {
 public:
  /*!
    * Constructor.
    @param map the map to be used for the coverage area. 
    The occupancy layer is used to decompose the global workspace. 
  */
  CellDecompBsd(grid_map::GridMap& map);
  /*!
         * Empty Constructor;
        */
  CellDecompBsd();

  /*!
    * Default copy assign and copy construtors.
  */
  CellDecompBsd(const CellDecompBsd& other) = default;
  CellDecompBsd(CellDecompBsd&& other) = default;
  CellDecompBsd& operator=(const CellDecompBsd& other) = default;
  CellDecompBsd& operator=(CellDecompBsd&& other) = default;

  /*!
         * Destructor
        */
  ~CellDecompBsd();

  /*!
    * Decomposes the coverage area into cells.
    * @param occupancy_layer the map to be used for the coverage area.
  */
  void decompose(const std::string& occupancy_layer);

  /*!
   * Returns an adjency matrix for two adjancent slices of the map, 
     each slice corresponds to a columns of the occupancy layer. 
  */
  static Eigen::MatrixXi getSliceAdj(std::vector<std::pair<int, int>>& slice_1, std::vector<std::pair<int, int>>& slice_2);

  /*!
   @brief Returns the number of disconnected segments in a column. 
   @param column the column of the occupancy layer.
   @return the number of disconnected segments in a column.

   * Given a column of the occupancy layer, 
   * returns a vector of disconnected segments. Each segment is identified by 
   * the row index at which it starts and it ends. 
  */
  std::vector<std::pair<int, int>> getSegments(int col);

  /*!
   @brief computes the global undirected adjency matrix of the cells in the coverage area.
   @returns adj matrix ofthe cells in the coverage area.
  */
  Eigen::MatrixXi getCellsAdj();
  
  /*!
   @brief computes the global directed adjency matrix of the cells in the coverage area.
   @returns adj matrix ofthe cells in the coverage area.
  */
  Eigen::MatrixXi getCellsDirAdj();
  
  grid_map::GridMap map_;
  grid_map::Matrix occupancy_map_;
};