/**
 * @file dfs.h
 * @author lwb
 * @brief DFS algorithm
 */
#ifndef DFS_H
#define DFS_H
#include <stack>

#include "planer.h"
/**
 * @brief Dfs class inherits Planer class
 */
class Dfs : public Planer<std::stack<Node>> {
public:
    Dfs() = delete;
    /**
     * @brief Constructor for Dfs class
     * @param grid_map: grid map
     */
    Dfs(const std::vector<std::vector<int>> &grid_map);
    /**
     * @brief Default destructor override for Dfs class
     */
    ~Dfs() override = default;
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
};
#endif
