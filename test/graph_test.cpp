#include "coverage_planner/Graph.hpp"

#include <gtest/gtest.h>
#include <ros/ros.h>

#include <Eigen/Dense>

using namespace Eigen;

TEST(GraphTest, test_add_edge) {
  Graph graph;
  graph.addEdge(0, 1);
  graph.addEdge(1, 2);

  EXPECT_EQ(graph.adj_list_[0][0], 1);
  EXPECT_EQ(graph.adj_list_[1][0], 2);
}

TEST(GraphTest, test_add_edges) {
  Graph graph;
  std::pair edge_01 = std::make_pair(0, 1);
  std::pair edge_12 = std::make_pair(1, 2);
  std::pair edge_03 = std::make_pair(0, 3);
  std::pair edge_34 = std::make_pair(3, 4);
  std::vector<std::pair<int, int>> edges = {edge_01, edge_12, edge_03, edge_34};
  bool directed = true;
  graph.addEdges(edges, directed);

  EXPECT_EQ(graph.adj_list_[0][0], 1);
  EXPECT_EQ(graph.adj_list_[1][0], 2);
  EXPECT_EQ(graph.adj_list_[0][1], 3);
  EXPECT_EQ(graph.adj_list_[3][0], 4);
}

TEST(GraphTest, bsf_test) {
  Graph graph;
  std::pair edge_01 = std::make_pair(0, 1);
  std::pair edge_12 = std::make_pair(1, 2);
  std::pair edge_03 = std::make_pair(0, 3);
  std::pair edge_34 = std::make_pair(3, 4);
  std::vector<std::pair<int, int>> edges = {edge_01, edge_12, edge_03, edge_34};
  bool directed = true;
  graph.addEdges(edges, directed);

  std::vector<int> bsf_order = graph.bfs(0);
  EXPECT_EQ(bsf_order[0], 0);
  EXPECT_EQ(bsf_order[1], 1);
  EXPECT_EQ(bsf_order[2], 3);
  EXPECT_EQ(bsf_order[3], 2);
  EXPECT_EQ(bsf_order[4], 4);
}

TEST(GraphTest, dfs_test) {
  Graph graph;
  std::pair edge_01 = std::make_pair(0, 1);
  std::pair edge_12 = std::make_pair(1, 2);
  std::pair edge_03 = std::make_pair(0, 3);
  std::pair edge_34 = std::make_pair(3, 4);
  std::vector<std::pair<int, int>> edges = {edge_01, edge_12, edge_03, edge_34};
  bool directed = true;
  graph.addEdges(edges, directed);

  std::vector<int> dfs_order = graph.dfs(0);
  EXPECT_EQ(dfs_order[0], 0);
  EXPECT_EQ(dfs_order[1], 1);
  EXPECT_EQ(dfs_order[2], 2);
  EXPECT_EQ(dfs_order[3], 3);
  EXPECT_EQ(dfs_order[4], 4);
}

TEST(GraphTest, test_find_heigth) {
  Graph graph;
  std::pair edge_01 = std::make_pair(0, 1);
  std::pair edge_12 = std::make_pair(1, 2);
  std::pair edge_03 = std::make_pair(0, 3);
  std::pair edge_34 = std::make_pair(3, 4);
  std::vector<std::pair<int, int>> edges = {edge_01, edge_12, edge_03, edge_34};
  bool directed = true;
  graph.addEdges(edges, directed);

  EXPECT_EQ(graph.findHeight(0).first, 2);
  EXPECT_EQ(graph.findHeight(1).first, 1);
  EXPECT_EQ(graph.findHeight(2).first, 0);
  EXPECT_EQ(graph.findHeight(3).first, 1);
  EXPECT_EQ(graph.findHeight(4).first, 0);
}

TEST(GraphTest, test_diameter) {
  Graph graph;
  std::pair edge_01 = std::make_pair(0, 1);
  std::pair edge_12 = std::make_pair(1, 2);
  std::pair edge_03 = std::make_pair(0, 3);
  std::pair edge_34 = std::make_pair(3, 4);
  std::vector<std::pair<int, int>> edges = {edge_01, edge_12, edge_03, edge_34};
  bool directed = false;
  graph.addEdges(edges, directed);

  std::tuple<int, int, int> diameter = graph.findDiameter(0);
  EXPECT_EQ(std::get<0>(diameter), 4);
  EXPECT_EQ(std::get<1>(diameter), 2);
  EXPECT_EQ(std::get<2>(diameter), 4);
}

TEST(GraphTest, test_postorder) {
  Graph graph;
  std::pair edge_01 = std::make_pair(0, 1);
  std::pair edge_12 = std::make_pair(1, 2);
  std::pair edge_03 = std::make_pair(0, 3);
  std::pair edge_34 = std::make_pair(3, 4);
  std::vector<std::pair<int, int>> edges = {edge_01, edge_12, edge_03, edge_34};
  bool directed = true;
  graph.addEdges(edges, directed);

  std::vector<int> postorder = graph.postorder(0);
  EXPECT_EQ(postorder[0], 2);
  EXPECT_EQ(postorder[1], 1);
  EXPECT_EQ(postorder[2], 4);
  EXPECT_EQ(postorder[3], 3);
  EXPECT_EQ(postorder[4], 0);
}

auto main(int argc, char** argv) -> int {
  // init ros
  ros::init(argc, argv, "graph_test");
  // print start tessting
  std::cout << "Start testing graph" << std::endl;
  testing::InitGoogleTest(&argc, argv);
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}
