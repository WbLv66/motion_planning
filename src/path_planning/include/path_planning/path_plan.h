#ifndef PATH_PLAN_H
#define PATH_PLAN_H

#include <array>
#include <nav_msgs/OccupancyGrid.h>
#include <queue>
#include <stack>
#include <unordered_set>
#include <vector>

class Bfs {
public:
  Bfs(float grid_resolution, std::vector<std::vector<int>> &grid_map);
  std::vector<std::array<double, 2>> Plan(std::array<int, 2> start,
                                          std::array<int, 2> end);

private:
  std::vector<std::vector<int>> grid_map_{};
  std::vector<std::vector<std::array<int, 2>>> parent_list_{};
  std::queue<Node> open_list_{};
  // TODO: 未来可以换成std::unordered_set，当地图较大的时候更加节省空间
  std::unordered_set std::vector<std::vector<int>> close_list_{};
  std::vector<std::array<int, 2>> grid_path_{};
  std::vector<std::array<double, 2>> path_{};
  float grid_resolution_{};

  std::array<std::array<int, 2>, 4> directions_{
      {{0, 1}, {1, 0}, {0, -1}, {-1, 0}}};
};

class DFS {
public:
  DFS(float grid_resolution, std::vector<std::vector<int>> &grid_map);
  std::vector<std::array<double, 2>> Plan(std::array<int, 2> start,
                                          std::array<int, 2> end);

private:
  std::vector<std::vector<int>> grid_map_{};
  std::vector<std::vector<std::array<int, 2>>> parent_list_{};
  std::stack<std::array<int, 2>> open_list_{};
  // TODO: 未来可以换成std::unordered_set，当地图较大的时候更加节省空间
  std::vector<std::vector<int>> close_list_{};
  std::vector<std::array<int, 2>> grid_path_{};
  std::vector<std::array<double, 2>> path_{};
  float grid_resolution_{};

  std::array<std::array<int, 2>, 4> directions_{
      {{0, 1}, {1, 0}, {0, -1}, {-1, 0}}};
};

class Dijkstra {
public:
  Dijkstra(float grid_resolution, std::vector<std::vector<int>> &grid_map);
  std::vector<std::array<double, 2>> Plan(std::array<int, 2> start,
                                          std::array<int, 2> end);

private:
  std::vector<std::vector<int>> grid_map_{};
  std::vector<std::vector<std::array<int, 2>>> parent_list_{};
  struct compare {
    bool operator()(int a, int b) {
      return a > b; // 这里定义了最小堆
    }
  };
  std::priority_queue<std::map<std::array<int, 2>, double>,
                      std::vector<std::map<std::array<int, 2>, double>>,
                      compare>
      open_list_{};
  // TODO: 未来可以换成std::unordered_set，当地图较大的时候更加节省空间
  std::vector<std::vector<int>> close_list_{};
  std::vector<std::array<int, 2>> grid_path_{};
  std::vector<std::array<double, 2>> path_{};
  float grid_resolution_{};

  std::array<std::array<int, 2>, 4> directions_{
      {{0, 1}, {1, 0}, {0, -1}, {-1, 0}}};
};

#endif