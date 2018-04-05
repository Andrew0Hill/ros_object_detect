//
// Created by ros on 3/31/18.
//
#include "../include/object_detect/graph_builder.h"
nav_msgs::Odometry odom;
std::shared_ptr<Eigen::Affine3d> odom_trans;
std::shared_ptr<Eigen::Affine3d> camera_trans;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr poses_cloud;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr current_cloud;
boost::shared_mutex odom_lock;
boost::shared_mutex cloud_lock;
std::shared_ptr<tf2_ros::Buffer> buffer;
std::shared_ptr<tf2_ros::TransformListener> listener;
pcl::IterativeClosestPoint<pcl::PointXYZRGB,pcl::PointXYZRGB> icp_gen;
ros::Rate* rate;
ros::Publisher p;
int main(int argc, char** argv){
    icp_gen.setMaxCorrespondenceDistance(0.1);
    icp_gen.setMaximumIterations(50);
    icp_gen.setTransformationEpsilon(1e-6);

    poses_cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    current_cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    // Initialize the graph_builder node.
    odom_trans = std::make_shared<Eigen::Affine3d>();
    camera_trans = std::make_shared<Eigen::Affine3d>();
    ros::init(argc,argv,"graph_builder");
    ros::NodeHandle n;
    buffer = std::make_shared<tf2_ros::Buffer>();
    listener = std::make_shared<tf2_ros::TransformListener>(*buffer);
    rate = new ros::Rate(1);
    // Subscribe to Odometry, and RGBXYZ Point Cloud
    ros::Subscriber odom_sub = n.subscribe("odom",1,odom_callback);
    ros::Subscriber pc_sub = n.subscribe("camera2/cloudRGB",1,pointcloud_callback);
    // Publish the stitched cloud
    poses_cloud->header.frame_id = "odom";
    p = n.advertise<pcl::PointCloud<pcl::PointXYZRGB>>("full_cloud",1);
    boost::thread graph_thread(build_graph);
    ros::spin();
}
void build_graph(){
    // Read from odometry message.
    while(ros::ok()) {
        if(current_cloud->size() > 0) {
            odom_lock.lock_shared();
            cloud_lock.lock_shared();
            pcl::transformPointCloud(*current_cloud, *current_cloud, *odom_trans);
            std::cout << odom_trans->matrix() << std::endl;
            if(poses_cloud->size() == 0){
                *poses_cloud += *current_cloud;
            }else {
                icp_gen.setInputSource(current_cloud);
                icp_gen.setInputTarget(poses_cloud);
                icp_gen.align(*poses_cloud);
                //p.publish(*poses_cloud);
            }
            p.publish(*poses_cloud);
            odom_lock.unlock_shared();
            cloud_lock.unlock_shared();
        }
        rate->sleep();
    }
    ROS_INFO_STREAM("Thread exiting.");
}
void odom_callback(const nav_msgs::Odometry::ConstPtr& odom_ptr){
    odom_lock.lock();
    odom = *odom_ptr;
    odom_trans->setIdentity();
    odom_trans->rotate(Eigen::Quaterniond(odom.pose.pose.orientation.w,
                                          odom.pose.pose.orientation.x,
                                          odom.pose.pose.orientation.y,
                                          odom.pose.pose.orientation.z));
    odom_trans->translation() << odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z;

    camera_trans->setIdentity();
    geometry_msgs::TransformStamped st = buffer->lookupTransform("base_link","camera2_depth_optical_frame",ros::Time());
    camera_trans->rotate(Eigen::Quaterniond(st.transform.rotation.w,
                                            st.transform.rotation.x,
                                            st.transform.rotation.y,
                                            st.transform.rotation.z));
    camera_trans->translation() << st.transform.translation.x,st.transform.translation.y,st.transform.translation.z;

    *odom_trans = (*odom_trans) * (*camera_trans);

    odom_lock.unlock();
}

void pointcloud_callback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud){
    cloud_lock.lock();
    pcl::copyPointCloud(*cloud,*current_cloud);
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*current_cloud,*current_cloud,indices);
    ROS_INFO_STREAM(current_cloud->width << " " << current_cloud->height);
    cloud_lock.unlock();
}