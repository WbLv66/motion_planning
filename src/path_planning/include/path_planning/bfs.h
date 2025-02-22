/**
 * @file bfs.h
 * @author lwb
 * @brief BFS algorithm
 */
#ifndef BFS_H
#define BFS_H
#include <queue>

#include "planer.h"
class Bfs : public Planer<std::queue<Node>> {
public:
    Bfs() = delete;
    Bfs(const std::vector<std::vector<int>> &grid_map);
    ~Bfs() override = default;
    void Initialization(const Node &start) override;
    void VisitNode() override;
    void ExpandNode() override;
};
#endif
