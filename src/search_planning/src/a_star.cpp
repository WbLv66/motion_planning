#include "a_star.h"
AStar::AStar(const std::vector<std::vector<int>> &grid_map)
    : Planer<std::priority_queue<Node, std::vector<Node>, std::greater<>>>{
          grid_map} {
    cost_map_.reserve(BasePlaner::grid_row_());
    for (int i = 0; i < BasePlaner::grid_row_(); ++i) {
        cost_map_.emplace_back(BasePlaner::grid_col_(),
                               std::numeric_limits<double>::max());
    }
}
void AStar::Initialization() {
    start_node_.cost_to_goal_ = CalculateEuclideanDistanceToGoal(start_node_);
    open_list_.push(start_node_);
}
void AStar::VisitNode() {
    current_node_ = open_list_.top();
    open_list_.pop();
    close_list_.insert(current_node_);
}
void AStar::ExpandNode() {
    for (const auto &difference : eight_directions_()) {
        next_node_ = current_node_ + difference;
        next_node_.cost_to_goal_ = CalculateEuclideanDistanceToGoal(next_node_);
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
double AStar::CalculateEuclideanDistanceToGoal(Node node) {
    return sqrt(pow(goal_node_.col_ - node.col_, 2) +
                pow(goal_node_.row_ - node.row_, 2));
}