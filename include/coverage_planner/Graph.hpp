#pragma once
#include <Eigen/Dense>
#include <vector>
#include <map>
#include <queue>
#include <iostream>

class Graph { 
  public:
    Graph() = default;
    Graph(std::map<int, std::vector<int>>& adjacency_list);
    ~Graph() = default;
    void addEdge(int v, int w);
    void addEdge(int v, int w, bool directed);

    void addEdges(std::vector<std::pair<int, int>>& edges, bool directed);
        /*!
      @brief Breadth-first search of the tree.
      @param root: random initial node from which to run the search
      @return:
        bfs_nodes (std::vector<int>): breadth-first traversal of the tree
    */
    std::vector<int> bfs(int root);

    /*!
      @brief Depth-first search of the tree.
      @param root: random initial node from which to run the search
      @return:
        dfs_nodes (std::vector<int>): depth-first traversal of the tree
    */    
    std::vector<int> dfs(int root);
    /*! 
      @brief Use DFS to find the height of the tree.

      @param root: random initial node from which to run DFS

      @return: 
        furthest_node (int): furthest away node from the root
        max_tree_height(int): diameter of the tree (longest path length)
    */

    std::pair<int, int> findHeight(int root);

    /*!
      @brief Find the diameter of the tree.
      @param root: random initial node 
      @return: 
        root (int): root of the tree
        furthest_node (int): furthest away node from the root
        max_tree_height(int): diameter of the tree (longest path length)
    */
    std::tuple<int, int, int> findDiameter(int root);


    /*! 
      @brief Postorder search of the tree.
      @param root: random initial node from which to run DFS
      @return: 
        postorder_nodes (std::vector<int>): postorder traversal of the tree
    */
    std::vector<int> postorder(int root);
    /*!
      @brief Find the shortest path between two nodes.
      @param root: random initial node from which to run the search
      @param v: node to find the shortest path to
      @return:
        path (std::vector<int>): shortest path from root to v
    */
    std::vector<int> shortestPath(int root, int v);
    /*!
      @brief preorder search of the tree.
      @param root: random initial node from which to run the search
      @return:
        preorder_nodes (std::vector<int>): preorder traversal of the tree
    */
    std::vector<int> preorder(int root);

    /*!
      Adjacency list of the graph.
    */
    std::map<int, std::vector<int>> adj_list_;

    private:

    void dfsHelper(int root, std::vector<int>& visited);
    void findHeightHelper(int root, int& furthest_node, int& max_tree_height, int tree_height, std::vector<int>& visited);
    void postorderHelper(int root, std::vector<int>& visited);
};
