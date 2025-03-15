#ifndef PRM_H
#define PRM_H
#include <array>
#include <random>
#include <vector>

class Node {
public:
    Node(double y, double x);
    Node operator+(const Node& other) const;
    int row_;
    int col_;
    std::vector<Node*> neighbor_node_{};
    double CalculateDistance(const Node other);
    double y_;
    double x_;
};

class Prm {
private:
    std::vector<std::vector<int>> grid_map_{};
    void GenerateNode();
    void GenerateEdge();
    void Learning() {
        void GenerateNode();
        void GenerateEdge();
    }
    void Query();

    bool HasObstacleOnEdge(Node& start, Node& end);

    std::vector<Node> node_list_{};
    const double x_min_{};
    const double x_max_{};
    const double y_min_{};
    const double y_max_{};

    const int N{};

    int grid_row_{};
    int grid_col_{};
    const std::array<std::array<int, 2>, 4> four_directions{
        {{0, 1}, {1, 0}, {0, -1}, {-1, 0}}};
    double distance_threshold_{};
    double grid_resolution_;
};

#endif