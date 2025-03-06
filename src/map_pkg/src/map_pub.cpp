#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>
const int MAP_ROW = 30;
const int MAP_COL = 30;

const std::vector<std::array<int, 4>> OBS_RECTANGLE = {{8, 6, 14, 12},
                                                       {18, 22, 24, 29}};
// const std::array<std::array<int, MAP_COL>, MAP_ROW> GRID_MAP = {
//     {{0, 1, 0, 0, 0, 0},
//      {0, 1, 0, 1, 0, 0},
//      {0, 0, 0, 1, 0, 0},
//      {0, 1, 0, 0, 0, 0},
//      {0, 1, 0, 1, 1, 0},
//      {0, 1, 0, 0, 0, 0}}};
int main(int argc, char **argv) {
    ros::init(argc, argv, "map_pub");
    ros::NodeHandle node_handle;
    ros::Publisher map_pub =
        node_handle.advertise<nav_msgs::OccupancyGrid>("/map", 1);
    ros::Rate rate(1);
    nav_msgs::OccupancyGrid map;
    map.header.frame_id = "map";
    map.info.resolution = 1.0;
    map.info.width = MAP_COL;
    map.info.height = MAP_ROW;
    map.info.origin.position.x = 0;
    map.info.origin.position.y = 0;
    map.data.reserve(MAP_ROW * MAP_COL);  // 预分配空间（可选，提升效率）

    std::array<std::array<int, MAP_COL>, MAP_ROW> grid_map{};
    for (auto obs : OBS_RECTANGLE) {
        for (auto col = obs[0]; col <= obs[2]; ++col) {
            for (auto row = obs[1]; row <= obs[3]; ++row) {
                grid_map[row][col] = 1;
            }
        }
    }

    // 遍历二维数组，逐行展开到一维 vector
    for (const auto &row : grid_map) {
        map.data.insert(map.data.end(), row.begin(), row.end());
    }
    std::for_each(map.data.begin(), map.data.end(),
                  [](auto &elem) { elem *= 100; });
    while (ros::ok()) {
        map_pub.publish(map);
        rate.sleep();
    }
}
