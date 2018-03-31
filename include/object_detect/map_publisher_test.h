//
// Created by ros on 3/25/18.
//

#ifndef OBJECT_DETECT_MAPPUBLISHERTEST_H
#define OBJECT_DETECT_MAPPUBLISHERTEST_H

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseArray.h>
#include <Eigen/Dense>

struct Node{
    Eigen::Vector3d pos;
    Eigen::Quaterniond rot;
    int id;
};
struct Edge {
    std::shared_ptr<Node> v1;
    std::shared_ptr<Node> v2;
};
struct Graph {
    std::vector<std::shared_ptr<Node>> nodes;
    std::vector<std::shared_ptr<Edge>> edges;
};

void odomCallback(const nav_msgs::OdometryConstPtr& odom);

#endif //OBJECT_DETECT_MAPPUBLISHERTEST_H
