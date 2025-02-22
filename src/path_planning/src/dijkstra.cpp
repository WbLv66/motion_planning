#include "dijkstra.h"
Dijkstra::Dijkstra(const std::vector<std::vector<int>> &grid_map)
    : Planer<std::priority_queue<Node, std::vector<Node>, std::greater<>>>{
          grid_map} {}
void Dijkstra::Initialization(const Node &start) { open_list_.push(start); }
void Dijkstra::VisitNode() {
    current_node_ = open_list_.top();
    open_list_.pop();
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
        open_list_.push(next_node_);
    }
}