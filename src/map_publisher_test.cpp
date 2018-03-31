//
// Created by ros on 3/25/18.
//

#include "../include/object_detect/map_publisher_test.h"

ros::Publisher ros_pub;
void odomCallback(const nav_msgs::OdometryConstPtr& odom){
    static Graph pose_graph;
    static int id_num = 0;
    // Add new node.
    ROS_INFO("Got Message!");
    std::shared_ptr<Node> node = std::make_shared<Node>();
    node->pos[0] = odom->pose.pose.position.x;
    node->pos[1] = odom->pose.pose.position.y;
    node->pos[2] = odom->pose.pose.position.z;

    node->rot.x() = odom->pose.pose.orientation.x;
    node->rot.y() = odom->pose.pose.orientation.y;
    node->rot.z() = odom->pose.pose.orientation.z;
    node->rot.w() = odom->pose.pose.orientation.w;

    node->id = id_num++;
    // Set node to have a link with the previous node.
    pose_graph.nodes.push_back(node);
    std::shared_ptr<Edge> e = std::make_shared<Edge>();
    if (pose_graph.nodes.empty()){
        ROS_INFO("First Node!");
    }else {
        ROS_INFO_STREAM("Adding Edge between Vertex: " << pose_graph.nodes.back()->id << " and: " << node->id);
        e->v1 = pose_graph.nodes.back();
        e->v2 = node;
        pose_graph.edges.push_back(e);
    }
    geometry_msgs::PoseArray array;
    for(const auto node_ptr : pose_graph.nodes){
        geometry_msgs::Pose pose;
        pose.position.x = node_ptr->pos[0];
        pose.position.y = node_ptr->pos[1];
        pose.position.z = node_ptr->pos[2];

        pose.orientation.x = node_ptr->rot.x();
        pose.orientation.y = node_ptr->rot.y();
        pose.orientation.z = node_ptr->rot.z();
        pose.orientation.w = node_ptr->rot.w();
        array.poses.push_back(pose);
    }
    array.header.frame_id = "odom";
    ros_pub.publish(array);
}

int main(int argc, char** argv){
    ros::init(argc,argv,"map_publisher_test");
    ros::NodeHandle n;

    ros::Rate loop(0.5);
// Create a subscriber to subscribe to robot odometry.
    ros::Subscriber s = n.subscribe("odom",10,&odomCallback);
    ros_pub = n.advertise<geometry_msgs::PoseArray>("pose_graph",10);
    while(ros::ok()) {
        ros::spinOnce();
        loop.sleep();
    }
}