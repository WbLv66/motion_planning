/**
 * @file planer.h
 * @author lwb
 * @brief include Node class and base class Plan
 */
#ifndef PLANER_H
#define PLANER_H

#include <cmath>
#include <limits>
#include <stack>
#include <unordered_set>
#include <vector>

template <class T>
class Planer;
/**
 * @brief Node class for planning
 * @param row_: row index of the node
 * @param col_: column index of the node
 * @param cost_to_goal_: cost from this node to goal
 * @param cost_from_start_: cost from start to this node
 * @param parent_: parent node
 */
class Node {
public:
    Node() = default;
    ~Node() = default;
    Node(int row, int col,
         double cost_from_start = std::numeric_limits<double>::max(),
         const Node *parent = nullptr);
    Node(const Node &other) = default;
    Node &operator=(const Node &other) = default;
    Node &operator=(Node &&other) = default;
    bool operator==(const Node &other) const;
    Node operator+(const Node &other) const;

    bool InMap() const;
    bool IsObstacle() const;

    int row_{};
    int col_{};
    double cost_to_goal_;
    double cost_from_start_;
    const Node *parent_{nullptr};
};

namespace std {
template <>
struct hash<Node> {
    size_t operator()(const Node &node) const {
        return (hash<int>()(node.row_) ^ hash<int>()(node.col_));
    }
};
}  // namespace std

template <class T>
class Planer {
public:
    Planer() = delete;
    Planer(const std::vector<std::vector<int>> &grid_map);
    virtual ~Planer() = default;
    std::stack<Node> &Plan(const Node &start, const Node &goal);

    static std::vector<std::vector<int>> &grid_map_();
    static int &grid_row_();
    static int &grid_col_();

protected:
    virtual void Initialization(const Node &start) {};
    virtual void VisitNode() {};
    bool GetToGoal(const Node &goal);
    void PathBacktrack();
    virtual void ExpandNode() {};

    T open_list_;
    std::unordered_set<Node> close_list_;
    std::stack<Node> grid_path_{};
    Node current_node_{};
    Node next_node_{};
    static const std::vector<Node> &four_directions_();
    static const std::vector<Node> &eight_directions_();
};

template <class T>
Planer<T>::Planer(const std::vector<std::vector<int>> &grid_map) {
    grid_map_() = grid_map;
    grid_row_() = grid_map.size();
    grid_col_() = grid_map[0].size();
}
template <class T>
std::stack<Node> &Planer<T>::Plan(const Node &start, const Node &goal) {
    Initialization(start);
    while (!open_list_.empty()) {
        VisitNode();
        if (GetToGoal(goal)) {
            PathBacktrack();
            break;
        }
        ExpandNode();
    }
    return grid_path_;
}

template <class T>
std::vector<std::vector<int>> &Planer<T>::grid_map_() {
    static std::vector<std::vector<int>> grid_map_{};
    return grid_map_;
}

template <class T>
int &Planer<T>::grid_row_() {
    static int grid_row_{};
    return grid_row_;
}

template <class T>
int &Planer<T>::grid_col_() {
    static int grid_col_{};
    return grid_col_;
}

template <class T>
bool Planer<T>::GetToGoal(const Node &goal) {
    return current_node_ == goal;
}

template <class T>
void Planer<T>::PathBacktrack() {
    while (current_node_.parent_ != nullptr) {
        grid_path_.push(current_node_);
        current_node_ = *(current_node_.parent_);
    }
    grid_path_.push(current_node_);
}

template <class T>
const std::vector<Node> &Planer<T>::four_directions_() {
    static const std::vector<Node> four_directions_{
        {0, 1, 1}, {1, 0, 1}, {0, -1, 1}, {-1, 0, 1}};
    return four_directions_;
}

template <class T>
const std::vector<Node> &Planer<T>::eight_directions_() {
    static const std::vector<Node> eight_directions_{
        {0, 1, 1},  {1, 1, std::sqrt(2)},   {1, 0, 1},  {1, -1, std::sqrt(2)},
        {0, -1, 1}, {-1, -1, std::sqrt(2)}, {-1, 0, 1}, {-1, 1, std::sqrt(2)}};
    return eight_directions_;
}

#endif