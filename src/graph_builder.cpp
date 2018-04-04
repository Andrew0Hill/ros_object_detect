//
// Created by ros on 3/31/18.
//
#include "../include/object_detect/graph_builder.h"
tf::Transform transform;
pcl::PointCloud<pcl::PointXYZRGB> poses_cloud;
boost::shared_mutex odom_lock;
boost::shared_mutex cloud_lock;

int main(int argc, char** argv){
    // Initialize the graph_builder node.
    ros::init(argc,argv,"graph_builder");
    ros::NodeHandle n;

    // Subscribe to Odometry, and RGBXYZ Point Cloud
    ros::Subscriber odom_sub = n.subscribe("odom",1,odom_callback);
    ros::Subscriber pc_sub = n.subscribe("camera2/cloudXYZ",1,pointcloud_callback);
    boost::thread graph_thread(build_graph);
    ros::spin();
}
void build_graph(){
    // Read from odometry message.
    ros::Rate rate(1);
    while(ros::ok()) {
        odom_lock.lock_shared();
        cloud_lock.lock_shared();
        ROS_INFO_STREAM("Reading Odom");

        odom_lock.unlock_shared();
        cloud_lock.unlock_shared();
        rate.sleep();
    }
    ROS_INFO_STREAM("Thread exiting.");
}
void odom_callback(const nav_msgs::Odometry::ConstPtr& odom_ptr){
    odom_lock.lock();
    transform.setRotation(tf::Quaternion(odom_ptr->pose.pose.orientation.x,odom_ptr->pose.pose.orientation.y, odom_ptr->pose.pose.orientation.z, odom_ptr->pose.pose.orientation.w));
    transform.setOrigin(tf::Vector3(odom_ptr->pose.pose.position.x, odom_ptr->pose.pose.position.y,odom_ptr->pose.pose.position.z));
    odom_lock.unlock();
}

void pointcloud_callback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud){
    cloud_lock.lock();
    ROS_INFO_STREAM("Cloud size: " << cloud->size());
    cloud_lock.unlock();
}