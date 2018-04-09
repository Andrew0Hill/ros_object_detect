//
// Created by ros on 3/31/18.
//
#include "../include/object_detect/graph_builder.h"
#include "PoseGraph.h"

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
pcl::VoxelGrid<pcl::PointXYZRGB> filter_grid;
ros::Rate* rate;
ros::Publisher p,graph;
PoseGraph* poseGraph;
int main(int argc, char** argv){
    // Set up ApproximateTimeSync Subscriber for this node.
    ros::init(argc,argv,"graph_builder");
    ros::NodeHandle node;

    poses_cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    current_cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    icp_gen.setMaximumIterations(10);
    icp_gen.setMaxCorrespondenceDistance(0.05);
    icp_gen.setTransformationEpsilon(1e-6);
    icp_gen.setRANSACOutlierRejectionThreshold(1.5);
    // Initialize the graph_builder node.
    odom_trans = std::make_shared<Eigen::Affine3d>();
    camera_trans = std::make_shared<Eigen::Affine3d>();
    poseGraph = new PoseGraph();
    buffer = std::make_shared<tf2_ros::Buffer>();
    listener = std::make_shared<tf2_ros::TransformListener>(*buffer);
    rate = new ros::Rate(1);
    // Subscribe to Odometry, and RGBXYZ Point Cloud
    ros::Subscriber odom_sub = node.subscribe("odom",1,odom_callback);
    ros::Subscriber pc_sub = node.subscribe("camera2/cloudRGB",1,pointcloud_callback);
    // Publish the stitched cloud
    poses_cloud->header.frame_id = "odom";


    p = node.advertise<pcl::PointCloud<pcl::PointXYZRGB>>("full_cloud",1);
    graph = node.advertise<visualization_msgs::MarkerArray>("graph_nodes",1);
    boost::thread graph_thread(build_graph);
    ros::spin();
}

void graph_callback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud, const nav_msgs::Odometry::ConstPtr& odom){

}


void build_graph(){
    // Read from odometry message.
    while(ros::ok()) {
        odom_lock.lock();
        cloud_lock.lock();
        if(current_cloud->size() > 0) {
            ROS_INFO("Starting filter.");
            ROS_INFO_STREAM("Current Cloud TF frame: " << current_cloud->header.frame_id);
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr voxel_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
            filter_grid.setInputCloud(current_cloud);
            filter_grid.setLeafSize(0.05,0.05,0.05);
            filter_grid.filter(*voxel_cloud);
            geometry_msgs::TransformStamped st;
            try {
                st = buffer->lookupTransform("odom", "camera_rgb_optical_frame", ros::Time());
            }catch(tf2::TransformException &te){
                ROS_WARN_STREAM(te.what());
            }

            Eigen::Affine3d eigen_trans(Eigen::Translation3d(st.transform.translation.x,
                                                             st.transform.translation.y,
                                                             st.transform.translation.z) *
                                        Eigen::Quaterniond(st.transform.rotation.w,
                                                           st.transform.rotation.x,
                                                           st.transform.rotation.y,
                                                           st.transform.rotation.z));

            pcl::transformPointCloud(*voxel_cloud,*voxel_cloud,eigen_trans);

            voxel_cloud->header.frame_id = "odom";

            if(poses_cloud->size() != 0) {
                ROS_INFO_STREAM("Finished Filter, filtered cloud size is: " << voxel_cloud->size() << " from initial size of: " << current_cloud->size());
                ROS_INFO_STREAM("Started ICP alignment.");
                icp_gen.setInputTarget(poses_cloud);
                icp_gen.setInputSource(voxel_cloud);
                icp_gen.align(*out_cloud);
                out_cloud->header.frame_id = "odom";
                ROS_INFO("Finished ICP alignment.");
                ROS_INFO("Concatenating Clouds.");

                *poses_cloud += *out_cloud;
                //*poses_cloud += *voxel_cloud;
                ROS_INFO_STREAM("Finished concatenating. Cloud size is: " << poses_cloud->size());
            }else{
                ROS_INFO("Poses cloud is empty! Adding current cloud.");
                *poses_cloud = *voxel_cloud;
            }
        }
        ROS_INFO_STREAM("Poses Cloud TF Frame: " << poses_cloud->header.frame_id);
        p.publish(poses_cloud);
        if(poseGraph->poses.empty()){
            ROS_INFO_STREAM("Pose graph empty. Adding intial pose.");
            poseGraph->add_vertex(odom);
        }else{
            if(odom.header.stamp != poseGraph->poses.back()->timestamp) {
                ROS_INFO_STREAM("Adding new pose. Graph has " << poseGraph->poses.size() << " nodes and "
                                                              << poseGraph->edges.size() << " edges.");
                poseGraph->add_vertex_previous(odom);
            }else{
                ROS_INFO_STREAM("Pose has same timestamp as last gather pose. Skipping.");
            }
        }
        visualization_msgs::MarkerArray markerArray;
        poseGraph->get_graph_markers(markerArray.markers);
        graph.publish(markerArray);
        odom_lock.unlock();
        cloud_lock.unlock();
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
    odom_lock.unlock();
}

void pointcloud_callback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud){
    cloud_lock.lock();
    pcl::copyPointCloud(*cloud,*current_cloud);
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*current_cloud,*current_cloud,indices);
    cloud_lock.unlock();
}