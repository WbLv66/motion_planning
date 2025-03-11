#include "bfs.h"
Bfs::Bfs(const std::vector<std::vector<int>> &grid_map)
    : Planer<std::queue<Node>>{grid_map} {}
void Bfs::Initialization() { open_list_.push(start_node_); }
void Bfs::VisitNode() {
    current_node_ = open_list_.front();
    open_list_.pop();
    close_list_.insert(current_node_);
}
void Bfs::ExpandNode() {
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