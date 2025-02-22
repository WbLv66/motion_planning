#include "planer.h"

#include <queue>
Node::Node(int row, int col, double cost_from_start, const Node *parent)
    : row_{row},
      col_{col},
      cost_from_start_{cost_from_start},
      parent_{parent} {}
bool Node::operator==(const Node &other) const {
    return (row_ == other.row_ && col_ == other.col_);
}
Node Node::operator+(const Node &other) const {
    return {row_ + other.row_, col_ + other.col_,
            cost_from_start_ + other.cost_from_start_};
}
bool Node::InMap() const {
    return (row_ >= 0 && row_ < Planer<std::queue<Node>>::grid_row_() &&
            col_ >= 0 && col_ < Planer<std::queue<Node>>::grid_col_());
}

bool Node::IsObstacle() const {
    return (Planer<std::queue<Node>>::grid_map_()[row_][col_] > 0);
}