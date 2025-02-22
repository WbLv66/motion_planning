#include "planer.h"

#include <queue>

Node::Node(int row, int col, double cost_from_start, const Node* parent)
    : row_{row},
      col_{col},
      cost_from_start_{cost_from_start},
      parent_{parent} {}
bool Node::operator==(const Node& other) const {
    return (row_ == other.row_ && col_ == other.col_);
}
bool Node::operator>(const Node& other) const {
    return (cost_from_start_ + cost_to_goal_ >
            other.cost_from_start_ + other.cost_to_goal_);
}
Node Node::operator+(const Node& other) const {
    return {row_ + other.row_, col_ + other.col_,
            cost_from_start_ + other.cost_from_start_};
}
bool Node::InMap() const {
    return (row_ >= 0 && row_ < BasePlaner::grid_row_() && col_ >= 0 &&
            col_ < BasePlaner::grid_col_());
}

bool Node::IsObstacle() const {
    return (BasePlaner::grid_map_()[row_][col_] > 0);
}

std::vector<std::vector<int>>& BasePlaner::grid_map_() {
    static std::vector<std::vector<int>> grid_map_;
    return grid_map_;
}

int& BasePlaner::grid_row_() {
    static int grid_row_;
    return grid_row_;
}

int& BasePlaner::grid_col_() {
    static int grid_col_;
    return grid_col_;
}

const std::vector<Node>& BasePlaner::four_directions_() {
    static const std::vector<Node> four_directions{
        {0, 1, 1}, {1, 0, 1}, {0, -1, 1}, {-1, 0, 1}};
    return four_directions;
}

const std::vector<Node>& BasePlaner::eight_directions_() {
    static const std::vector<Node> eight_directions{
        {0, 1, 1},  {1, 1, std::sqrt(2)},   {1, 0, 1},  {1, -1, std::sqrt(2)},
        {0, -1, 1}, {-1, -1, std::sqrt(2)}, {-1, 0, 1}, {-1, 1, std::sqrt(2)}};
    return eight_directions;
}