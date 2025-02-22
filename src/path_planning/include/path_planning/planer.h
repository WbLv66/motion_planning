/**
 * @file planer.h
 * @author lwb
 * @brief Include Node class and base class Plan
 */
#ifndef PLANER_H
#define PLANER_H

#include <cmath>
#include <limits>
#include <stack>
#include <unordered_set>
#include <vector>

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
    /**
     * @brief Default constructor for Node class
     */
    Node() = default;
    /**
     * @brief Default destructor for Node class
     */
    ~Node() = default;
    /**
     * @brief Constructor for Node class
     * @param row: row index of the node
     * @param col: column index of the node
     * @param cost_from_start: cost from start to this node, the default value
     * is infinite
     * @param parent: parent node, the default value is nullptr
     */
    Node(int row, int col,
         double cost_from_start = std::numeric_limits<double>::max(),
         const Node *parent = nullptr);
    /**
     * @brief Copy constructor for Node class
     * @param other: the node to be copied
     */
    Node(const Node &other) = default;
    /**
     * @brief Copy assignment operator for Node class
     * @param other: the node to be copied
     */
    Node &operator=(const Node &other) = default;
    /**
     * @brief Move constructor for Node class
     * @param other: the node to be moved
     */
    Node(Node &&other) = default;
    /**
     * @brief Move assignment operator for Node class
     * @param other: the node to be moved
     */
    Node &operator=(Node &&other) = default;
    /**
     * @brief Overload operator == for Node class
     * @param other: the node to be compared
     * @return bool: whether the two nodes are equal
     */
    bool operator==(const Node &other) const;
    /**
     * @brief Overload operator > for Node class
     * @param other: the node to be added
     * @return bool: whether this node is greater than the other
     */
    bool operator>(const Node &other) const;
    /**
     * @brief Overload operator + for Node class
     * @param other: the node to be added
     * @return Node: the sum of two nodes
     */
    Node operator+(const Node &other) const;
    /**
     * @brief Adjust whether the node is in the map
     * @return bool: whether the node is in the map
     */
    bool InMap() const;
    /**
     * @brief Adjust whether the node is an obstacle
     * @return bool: whether the node is an obstacle
     */
    bool IsObstacle() const;

    int row_{};
    int col_{};
    double cost_to_goal_;
    double cost_from_start_;
    const Node *parent_{nullptr};
};

namespace std {
/**
 * @brief Hash function for Node class
 * @param node: the node to be hashed
 * @return size_t: the hash value of the node
 * @details  hash value is determined by row_ and col_
 */
template <>
struct hash<Node> {
    size_t operator()(const Node &node) const {
        return (hash<int>()(node.row_) ^ hash<int>()(node.col_));
    }
};
}  // namespace std
/**
 * @brief Base class BasePlaner
 * @param grid_map(): grid map, can be used as static variable
 * @param grid_row_(): rows number, can be used as static variable
 * @param grid_col_(): columns number, can be used as static variable
 * @param four_directions_(): expand to four directions, can be used as static
 * variable
 * @param eight_directions_(): expand to eight directions, can be used as static
 * variable
 */
class BasePlaner {
public:
    /**
     *@brief Grid map
     *@return std::vector<std::vector<int>>&: reference to static variable
     *@details static function which creats static variable
     */
    static std::vector<std::vector<int>> &grid_map_();
    /**
     *@brief Rows number
     *@return int&: reference to static variable
     *@details static function which creats static variable
     */
    static int &grid_row_();
    /**
     *@brief Columns number
     *@return int&: reference to static variable
     *@details static function which creats static variable
     */
    static int &grid_col_();
    /**
     *@brief Expand to four directions
     *@return const std::vector<Node> &: reference to static variable
     *@details static function which creats static variable
     */
    static const std::vector<Node> &four_directions_();
    /**
     *@brief Expand to eight directions
     *@return const std::vector<Node> &: reference to static variable
     *@details static function which creats static variable
     */
    static const std::vector<Node> &eight_directions_();
};

/**
 * @brief Template class Planer
 * @param T: the type of open_list_
 * @param open_list_: contains nodes which will be visited
 * @param close_list_: contains nodes which have been visited
 * @param grid_path_: the grid path obtained
 * @param current_node_: current node
 * @param next_node_: neighbor node
 */
template <class T>
class Planer : public BasePlaner {
public:
    Planer() = delete;
    /**
     * @brief Constructor for Planer class
     * @param grid_map: grid map
     */
    Planer(const std::vector<std::vector<int>> &grid_map);
    /**
     * @brief Virtual default destructor for Planer class
     */
    virtual ~Planer() = default;
    /**
     * @brief Plan the path from start to goal
     * @param start: start node
     * @param goal: goal node
     * @return std::stack<Node>: the grid path
     * @details designd using template method pattern
     */
    std::stack<Node> &Plan(const Node &start, const Node &goal);

protected:
    /**
     *@brief Virtual function that initialize the open_list_
     *@param start: start node
     */
    virtual void Initialization(const Node &start){};
    /**
     *@brief Virtual function that visit the node
     */
    virtual void VisitNode(){};
    /**
     *@brief Adjust whether get to goal
     *@param goal: goal node
     *@return bool: whether get to goal
     */
    bool GetToGoal(const Node &goal);
    /**
     *@brief Backtrack to obtain path
     */
    void PathBacktrack();
    /**
     *@brief Virtual function that expand the node
     */
    virtual void ExpandNode(){};

    T open_list_;
    std::unordered_set<Node> close_list_;
    std::stack<Node> grid_path_{};
    Node current_node_{};
    Node next_node_{};
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

#endif