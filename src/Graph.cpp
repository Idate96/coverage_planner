#include "coverage_planner/Graph.hpp"

// Graph::Graph() : adj_list_(){};

Graph::Graph(std::map<int, std::vector<int>>& graph) : adj_list_(graph){};

void Graph::addEdge(int u, int v) { adj_list_[u].push_back(v); }

void Graph::addEdge(int u, int v, bool directed) {
  if (directed) {
    addEdge(u, v);
  } else {
    addEdge(u, v);
    addEdge(v, u);
  }
}

void Graph::addEdges(std::vector<std::pair<int, int>>& edges, bool directed) {
  for (auto& edge : edges) {
    addEdge(edge.first, edge.second, directed);
  }
}


std::vector<int> Graph::bfs(int start) {
  std::vector<int> bfs_order;
  std::vector<bool> visited(adj_list_.size(), false);
  std::queue<int> q;
  q.push(start);
  visited[start] = true;
  while (!q.empty()) {
    int u = q.front();
    q.pop();
    bfs_order.push_back(u);
    for (auto& v : adj_list_[u]) {
      if (!visited[v]) {
        q.push(v);
        visited[v] = true;
      }
    }
  }
  return bfs_order;
}


void Graph::dfsHelper(int node, std::vector<int>& visited) {
  for (auto neighbor : adj_list_[node]) {
    // if neighbor is not visited
    if (find(visited.begin(), visited.end(), neighbor) == visited.end()) {
      visited.push_back(neighbor);
      dfsHelper(neighbor, visited);
    }
  }
}

std::vector<int> Graph::dfs(int root) {
  std::vector<int> visited;
  visited.push_back(root);
  dfsHelper(root, visited);
  return visited;
}

void Graph::findHeightHelper(int node, int& furthest_node, int& max_tree_height, int tree_height,
                             std::vector<int>& visited) {
  for (auto neighbor : adj_list_[node]) {
    // if neighbor is not visited
    if (find(visited.begin(), visited.end(), neighbor) == visited.end()) {
      visited.push_back(neighbor);
      if (tree_height + 1 > max_tree_height) {
        max_tree_height = tree_height + 1;
        furthest_node = neighbor;
      }
      findHeightHelper(neighbor, furthest_node, max_tree_height, tree_height + 1, visited);
    }
  }
}

std::pair<int, int> Graph::findHeight(int node) {
  std::vector<int> visited;
  visited.push_back(node);

  int max_height = 0;
  int furthest_node = node;

  findHeightHelper(node, furthest_node, max_height, 0, visited);
  return std::make_pair(max_height, furthest_node);
}

std::tuple<int, int, int> Graph::findDiameter(int node) {
  std::pair<int, int> height_and_furthest_node = findHeight(node);
  int new_root = std::get<1>(height_and_furthest_node);

  std::pair<int, int> height_and_furthest_node_new_root = findHeight(new_root);
  return std::make_tuple(std::get<0>(height_and_furthest_node_new_root), new_root,
                         std::get<1>(height_and_furthest_node_new_root));
}

std::vector<int> Graph::postorder(int node) {
  std::vector<int> visited;
  postorderHelper(node, visited);
  return visited;
}

void Graph::postorderHelper(int node, std::vector<int>& visited) {
  for (int neighbor : adj_list_[node]) {
    postorderHelper(neighbor, visited);
  }
  visited.push_back(node);
}
