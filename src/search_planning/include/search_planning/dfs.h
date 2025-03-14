/**
 * @file dfs.h
 * @author lwb (wb.lv@qq.com)
 * @brief DFS algorithm
 */
#ifndef DFS_H
#define DFS_H
#include <stack>

#include "planer.h"
/**
 * @brief Dfs class inherits Planer class, open_list_ is a stack
 */
class Dfs : public Planer<std::stack<Node>> {
public:
    Dfs() = delete;
    /**
     * @brief Constructor for Dfs class
     * @param grid_map: grid map
     */
    explicit Dfs(const std::vector<std::vector<int>> &grid_map);
    /**
     * @brief Default destructor override for Dfs class
     */
    ~Dfs() override = default;
    /**
     *@brief Override function that initialize the open_list_
     *@param start: start node
     */
    void Initialization() override;
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
