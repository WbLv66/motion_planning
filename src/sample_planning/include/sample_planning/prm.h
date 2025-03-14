#ifndef PRM_H
#define PRM_H
#include <random>
#include <vector>
class Prm {
private:
    std::vector<std::vector<int>> grid_map_{};
    void GetObstacleBoundaryNode();
};

#endif