#include "dfs.h"
Dfs::Dfs(const std::vector<std::vector<int>> &grid_map)
    : Planer<std::stack<Node>>{grid_map} {}
void Dfs::Initialization() { open_list_.push(start_node_); }
void Dfs::VisitNode() {
    current_node_ = open_list_.top();
    open_list_.pop();
    close_list_.insert(current_node_);
}
void Dfs::ExpandNode() {
    for (const auto &difference : four_directions_()) {
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
        open_list_.push(next_node_);
    }
}