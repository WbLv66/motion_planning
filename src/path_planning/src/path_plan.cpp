#include "path_plan.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <queue>
#include <ros/ros.h>

float kGridResolution{};
std::vector<std::vector<int>> kGridMap{};

Bfs::Bfs(float grid_resolution, std::vector<std::vector<int>> &grid_map)
    : grid_resolution_{grid_resolution}, grid_map_{grid_map} {}
std::vector<std::array<double, 2>> Bfs::Plan(std::array<int, 2> start,
                                             std::array<int, 2> end) {
  close_list_ = grid_map_;
  for (auto &row : close_list_) {
    row.assign(row.size(), 0); // 将每一行的元素全部赋值为0
  }

  parent_list_.resize(grid_map_.size());
  for (auto &row : parent_list_) {
    row.resize(grid_map_[0].size());
  }

  std::array<int, 2> current_node{};
  std::array<int, 2> next_node{};

  open_list_.push(start);

  while (!open_list_.empty()) {
    current_node = open_list_.front();
    open_list_.pop();
    close_list_[current_node[0]][current_node[1]] = 1;
    if (current_node == end) {
      grid_path_.emplace_back(end);
      while (grid_path_.back() != start) {
        grid_path_.emplace_back(
            parent_list_[grid_path_.back()[0]][grid_path_.back()[1]]);
      }
      break;
    }

    for (const auto &i : directions_) {
      next_node[0] = current_node[0] + i[0];
      next_node[1] = current_node[1] + i[1];
      if (next_node[0] < 0 || next_node[0] >= grid_map_[0].size() ||
          next_node[1] < 0 || next_node[1] >= grid_map_.size()) {
        continue;
      }
      if (close_list_[next_node[0]][next_node[1]] == 1 ||
          grid_map_[next_node[0]][next_node[1]] == 100) {
        continue;
      }
      open_list_.push(next_node);
      parent_list_[next_node[0]][next_node[1]] = current_node;
    }
  }
  for (auto it = grid_path_.rbegin(); it != grid_path_.rend(); ++it) {
    path_.push_back(
        {(*it)[1] * grid_resolution_ + 0.5, (*it)[0] * grid_resolution_ + 0.5});
  }
  return path_;
}

DFS::DFS(float grid_resolution, std::vector<std::vector<int>> &grid_map)
    : grid_resolution_{grid_resolution}, grid_map_{grid_map} {}
std::vector<std::array<double, 2>> DFS::Plan(std::array<int, 2> start,
                                             std::array<int, 2> end) {
  close_list_ = grid_map_;
  for (auto &row : close_list_) {
    row.assign(row.size(), 0); // 将每一行的元素全部赋值为0
  }

  parent_list_.resize(grid_map_.size());
  for (auto &row : parent_list_) {
    row.resize(grid_map_[0].size());
  }

  std::array<int, 2> current_node{};
  std::array<int, 2> next_node{};

  open_list_.push(start);

  while (!open_list_.empty()) {
    current_node = open_list_.top();
    open_list_.pop();
    close_list_[current_node[0]][current_node[1]] = 1;
    if (current_node == end) {
      grid_path_.emplace_back(end);
      while (grid_path_.back() != start) {
        grid_path_.emplace_back(
            parent_list_[grid_path_.back()[0]][grid_path_.back()[1]]);
      }
      break;
    }

    for (const auto &i : directions_) {
      next_node[0] = current_node[0] + i[0];
      next_node[1] = current_node[1] + i[1];
      if (next_node[0] < 0 || next_node[0] >= grid_map_[0].size() ||
          next_node[1] < 0 || next_node[1] >= grid_map_.size()) {
        continue;
      }
      if (close_list_[next_node[0]][next_node[1]] == 1 ||
          grid_map_[next_node[0]][next_node[1]] == 100) {
        continue;
      }
      open_list_.push(next_node);
      parent_list_[next_node[0]][next_node[1]] = current_node;
    }
  }
  for (auto it = grid_path_.rbegin(); it != grid_path_.rend(); ++it) {
    path_.push_back(
        {(*it)[1] * grid_resolution_ + 0.5, (*it)[0] * grid_resolution_ + 0.5});
  }
  return path_;
}

void MapCallback(const nav_msgs::OccupancyGrid &grid_map) {

  const int col = grid_map.info.width;
  const int row = grid_map.info.height;
  kGridResolution = grid_map.info.resolution;

  kGridMap.reserve(row);

  auto vector_begin = grid_map.data.begin();

  for (int i = 0; i < row; ++i) {
    kGridMap.emplace_back(vector_begin + col * i, vector_begin + col * (i + 1));
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "path_plan");
  ros::NodeHandle node_handle;
  ros::Subscriber sub_map = node_handle.subscribe("/map", 1, MapCallback);
  ros::Publisher path_pub = node_handle.advertise<nav_msgs::Path>("path", 1);
  ros::Rate rate(50);
  while (kGridMap.empty()) {
    ros::spinOnce();
    rate.sleep();
  }
  Bfs bfs{kGridResolution, kGridMap};
  DFS dfs{kGridResolution, kGridMap};
  // auto path = bfs.Plan({0, 0}, {5, 5});
  auto path = dfs.Plan({0, 0}, {5, 5});
  nav_msgs::Path path_msg;
  path_msg.header.frame_id = "map";
  geometry_msgs::PoseStamped pose;
  for (auto &p : path) {
    pose.pose.position.x = p[0];
    pose.pose.position.y = p[1];
    path_msg.poses.push_back(pose);
  }
  while (ros::ok()) {
    path_pub.publish(path_msg);
    rate.sleep();
  }

  //
}