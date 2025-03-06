/**
 *@file dijkstra.h
 *@author lwb
 *@brief Dijkstra algorithm
 */
#ifndef DIJKSTRA_H_
#define DIJKSTRA_H_
#include <queue>

#include "planer.h"

class Dijkstra
    : public Planer<
          std::priority_queue<Node, std::vector<Node>, std::greater<>>> {
public:
    Dijkstra() = delete;
    /**
     * @brief Constructor for Dijkstra class
     * @param grid_map: grid map
     */
    Dijkstra(const std::vector<std::vector<int>> &grid_map);
    /**
     * @brief Default destructor override for Dijkstra class
     */
    ~Dijkstra() override = default;
    /**
     *@brief Override function that initialize the open_list_
     *@param start: start node
     */
    void Initialization(const Node &start) override;
    /**
     *@brief Override function that visit the node
     */
    void VisitNode() override;
    /**
     *@brief Override function that expand the node
     */
    void ExpandNode() override;

private:
    std::vector<std::vector<double>> cost_map_;
};
#endif