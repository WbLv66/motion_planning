#include "prm.h"

#include <Eigen/Core>
#include <random>

void Prm::GenerateNode() {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis_x(x_min_, x_max_);
    std::uniform_real_distribution<> dis_y(y_min_, y_max_);
    node_list_.reserve(N);
    while (node_list_.size() < N) {
        Node node{dis_y(gen), dis_x(gen)};
        if (grid_map_[node.row_][node.col_] == 0) {
            node_list_.push_back(node);
        }
    }
}
void Prm::GenerateEdge() {
    for (auto first_iteator = node_list_.begin();
         first_iteator < node_list_.end() - 1; ++first_iteator) {
        for (auto second_iteator = first_iteator + 1;
             second_iteator < node_list_.end(); ++second_iteator) {
            if (first_iteator->CalculateDistance(*second_iteator) >
                distance_threshold_) {
                continue;
            }
            if (HasObstacleOnEdge(*first_iteator, *second_iteator)) {
                continue;
            }
            first_iteator->neighbor_node_.push_back(&(*second_iteator));
            second_iteator->neighbor_node_.push_back(&(*first_iteator));
        }
    }
}

bool Prm::HasObstacleOnEdge(Node& start, Node& end) {
    // straight line equation
    double y_1 = start.y_;
    double x_1 = start.x_;
    double y_2 = end.y_;
    double x_2 = end.x_;
    double A = y_2 - y_1;
    double B = x_1 - x_2;
    double C = x_2 * y_1 - x_1 * y_2;
    if (B == 0) {
        int row_start{};
        int row_end{};
        if (A > 0) {
            row_start = start.row_;
            row_end = end.row_;
        } else {
            row_start = end.row_;
            row_end = start.row_;
        }

        for (int row = row_start; row <= row_end; ++row) {
            if (grid_map_[row][start.col_] == 100) {
                return true;
            }
        }
    } else {
        int col_start{};
        int col_end{};
        if (B < 0) {
            col_start = start.col_;
            col_end = end.col_;
        } else {
            col_start = end.col_;
            col_end = start.col_;
        }
        for (int col = col_start; col < col_end; ++col) {
            double grid_x_coordinate = (col + 1) * grid_resolution_;

            double y_coordinate = (-C - A * grid_x_coordinate) / B;
        }
    }
    return false;
}