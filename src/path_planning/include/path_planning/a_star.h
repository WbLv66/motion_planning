/**
 * @file a_star.h
 * @author lwb (wb.lv@qq.com)
 * @brief A* algorithm
 */
#ifndef A_STAR_H
#define A_STAR_H
#include <queue>

#include "planer.h"

class AStar
    : public Planer<
          std::priority_queue<Node, std::vector<Node>, std::greater<>>> {
public:
    AStar() = delete;
    explicit AStar(const std::vector<std::vector<int>> &grid_map);
    ~AStar() override = default;

private:
    void Initialization() override;
    void VisitNode() override;
    void ExpandNode() override;
    double CalculateEuclideanDistanceToGoal(Node node);

    std::vector<std::vector<double>> cost_map_;
};
#endif