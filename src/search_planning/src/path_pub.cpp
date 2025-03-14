
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>

#include <stack>

#include "a_star.h"
#include "bfs.h"
#include "dfs.h"
#include "dijkstra.h"
#include "planer.h"
std::vector<std::vector<int>> kGridMap;

double kGridResolution;
void MapCallback(const nav_msgs::OccupancyGrid &msg) {
    kGridResolution = msg.info.resolution;
    kGridMap.reserve(msg.info.height);
    auto begin = msg.data.begin();
    for (int i = 0; i < msg.info.height; ++i) {
        kGridMap.emplace_back(begin + msg.info.width * i,
                              begin + msg.info.width * (i + 1));
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "path_pub");
    ros::NodeHandle node_handle;
    ros::Subscriber map_sub = node_handle.subscribe("/map", 1, MapCallback);
    ros::Publisher path_pub = node_handle.advertise<nav_msgs::Path>("/path", 1);
    Node start{0, 0, 0};
    Node goal{29, 29};
    ros::Rate rate(50);
    while (kGridMap.empty()) {
        ros::spinOnce();
        rate.sleep();
    }

    // Bfs bfs{kGridMap};
    // std::stack<Node> path = bfs.Plan(start, goal);

    // Dfs dfs{kGridMap};
    // std::stack<Node> path = dfs.Plan(start, goal);

    // Dijkstra dijkstra{kGridMap};
    // std::stack<Node> path = dijkstra.Plan(start, goal);

    AStar a_star{kGridMap};
    std::stack<Node> path = a_star.Plan(start, goal);

    nav_msgs::Path path_msg;
    path_msg.header.frame_id = "map";
    geometry_msgs::PoseStamped pose;
    while (!path.empty()) {
        pose.pose.position.x = (path.top().col_ + 0.5) * kGridResolution;
        pose.pose.position.y = (path.top().row_ + 0.5) * kGridResolution;
        path_msg.poses.push_back(pose);
        path.pop();
    }

    while (ros::ok()) {
        path_pub.publish(path_msg);
        rate.sleep();
    }

    //
}