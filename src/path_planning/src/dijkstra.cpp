#include "dijkstra.h"

#include <iostream>
Dijkstra::Dijkstra(const std::vector<std::vector<int>> &grid_map)
    : Planer<std::priority_queue<Node, std::vector<Node>, std::greater<>>>{
          grid_map} {
    cost_map_.reserve(BasePlaner::grid_row_());
    for (int i = 0; i < BasePlaner::grid_row_(); ++i) {
        cost_map_.emplace_back(BasePlaner::grid_col_(),
                               std::numeric_limits<double>::max());
    }
}
void Dijkstra::Initialization(const Node &start) { open_list_.push(start); }
void Dijkstra::VisitNode() {
    current_node_ = open_list_.top();
    open_list_.pop();
    std::cout << open_list_.size() << std::endl;
    close_list_.insert(current_node_);
}
void Dijkstra::ExpandNode() {
    for (const auto &difference : eight_directions_()) {
        next_node_ = current_node_ + difference;
        auto iterator = close_list_.find(current_node_);
        next_node_.parent_ = &(*iterator);
        if (close_list_.find(next_node_) != close_list_.end()) {
            continue;
        }
        if (!next_node_.InMap()) {
            continue;
        }
        if (next_node_.IsObstacle()) {
            continue;
        }
        if (next_node_.cost_from_start_ >=
            cost_map_[next_node_.row_][next_node_.col_]) {
            continue;
        }
        open_list_.push(next_node_);
        cost_map_[next_node_.row_][next_node_.col_] =
            next_node_.cost_from_start_;
    }
}