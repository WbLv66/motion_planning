/**
 * @file bfs.h
 * @author lwb (wb.lv@qq.com)
 * @brief BFS algorithm
 */
#ifndef BFS_H
#define BFS_H
#include <queue>

#include "planer.h"
/**
 * @brief Bfs class inherits Planer class, open_list_ is a queue
 */
class Bfs : public Planer<std::queue<Node>> {
public:
    Bfs() = delete;
    /**
     * @brief Constructor for Bfs class
     * @param grid_map: grid map
     */
    explicit Bfs(const std::vector<std::vector<int>> &grid_map);
    /**
     * @brief Default destructor override for Bfs class
     */
    ~Bfs() override = default;
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
