//
// Created by ros on 4/7/18.
//

#include "../include/object_detect/graph_builder_sync.h"
int main(int argc, char** argv) {
    ros::init(argc,argv,"sync_graph_builder");
    SyncGraphBuilder graphBuilder;
    ros::spin();
}

SyncGraphBuilder::SyncGraphBuilder(){
    // Create subscribers
    cloud_sub =  new message_filters::Subscriber<pcl::PointCloud<pcl::PointXYZRGB>>(n,"camera2/cloudRGB/throttled",15);
    odom_sub = new message_filters::Subscriber<nav_msgs::Odometry>(n,"odom", 200);

    // Create synchronized subscriber.
    synchronizer = new message_filters::Synchronizer<graph_sync_policy>(graph_sync_policy(50),*cloud_sub,*odom_sub);

    // Register the callback with the subscriber.
    synchronizer->registerCallback(boost::bind(&SyncGraphBuilder::graph_callback, this, _1, _2));

    // Create publishers for cloud and graph node/edge visualization.
    cloud_pub = n.advertise<pcl::PointCloud<pcl::PointXYZRGB>>("/full_cloud",1);
    graph_pub = n.advertise<visualization_msgs::MarkerArray>("/graph_nodes",1);

    // Create TF2 Buffer and Listener.
    buffer = std::make_shared<tf2_ros::Buffer>();
    listener = std::make_shared<tf2_ros::TransformListener>(*buffer);

    // Create Pose Graph.
    poseGraph = std::make_shared<PoseGraph>();

    poses_cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

    // Set ICP parameters.
    icp_gen.setMaximumIterations(30);
    icp_gen.setMaxCorrespondenceDistance(0.05);
    icp_gen.setTransformationEpsilon(1e-8);
    icp_gen.setEuclideanFitnessEpsilon (1);
    icp_gen.setRANSACOutlierRejectionThreshold(1.5);

    // Set Voxel Grid parameters.
    filter_grid.setLeafSize(0.08,0.08,0.08);

}

SyncGraphBuilder::~SyncGraphBuilder(){
    delete synchronizer;
    delete cloud_sub;
    delete odom_sub;
}

void SyncGraphBuilder::graph_callback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud, const nav_msgs::Odometry::ConstPtr& odom){
    ROS_INFO("Starting filter.");
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr current_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::copyPointCloud(*cloud,*current_cloud);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr voxel_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filt_pose_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    filter_grid.setInputCloud(cloud);
    filter_grid.filter(*voxel_cloud);

    ros::Duration diff = odom->header.stamp - pcl_conversions::fromPCL(cloud->header.stamp);
    ROS_INFO_STREAM("Diff in time: " << diff.toSec());
    // Lookup transform for point cloud
    geometry_msgs::TransformStamped camera_to_base;
    geometry_msgs::TransformStamped base_to_world;
    try {
        camera_to_base = buffer->lookupTransform("base_link", "camera_rgb_optical_frame", odom->header.stamp);
    }catch(tf2::TransformException &te){
        ROS_WARN_STREAM(te.what());
    }


    Eigen::Affine3d c2b_trans(Eigen::Translation3d(camera_to_base.transform.translation.x,
                                                     camera_to_base.transform.translation.y,
                                                     camera_to_base.transform.translation.z) *
                                Eigen::Quaterniond(camera_to_base.transform.rotation.w,
                                                   camera_to_base.transform.rotation.x,
                                                   camera_to_base.transform.rotation.y,
                                                   camera_to_base.transform.rotation.z));

    Eigen::Affine3d b2w_trans(Eigen::Translation3d(odom->pose.pose.position.x,
                                                   odom->pose.pose.position.y,
                                                   odom->pose.pose.position.z) *
                              Eigen::Quaterniond(odom->pose.pose.orientation.w,
                                                 odom->pose.pose.orientation.x,
                                                 odom->pose.pose.orientation.y,
                                                 odom->pose.pose.orientation.z));

    Eigen::Affine3d c2w_trans = b2w_trans * c2b_trans;

    voxel_cloud->header.frame_id = "odom";
    filt_pose_cloud->header.frame_id = "odom";

    ROS_INFO_STREAM("Poses Cloud TF Frame: " << poses_cloud->header.frame_id);


    pcl::transformPointCloud(*voxel_cloud,*voxel_cloud,c2w_trans);

    if(poseGraph->poses.empty()){
        ROS_INFO_STREAM("Pose graph empty. Adding initial pose...");
        ROS_INFO("Poses cloud is empty. Adding current cloud...");
        *poses_cloud = *voxel_cloud;
        poseGraph->add_vertex(*odom,voxel_cloud);

    }else{
        ROS_INFO_STREAM("Adding new pose. Graph has " << poseGraph->poses.size() << " nodes and "
                                                      << poseGraph->edges.size() << " edges.");
        ROS_INFO_STREAM("Finished Filter, filtered cloud size is: " << voxel_cloud->size() << " from initial size of: " << current_cloud->size());
        ROS_INFO_STREAM("Started ICP alignment.");
        icp_gen.setInputTarget(poses_cloud);
        icp_gen.setInputSource(voxel_cloud);
        icp_gen.align(*out_cloud);
        out_cloud->header.frame_id = "odom";
        ROS_INFO_STREAM("Finished ICP alignment. Score is: " << icp_gen.hasConverged());
        ROS_INFO("Concatenating Clouds.");
        ROS_INFO_STREAM("Finished concatenating. Cloud size is: " << poses_cloud->size());

        *poses_cloud += *out_cloud;

        ROS_INFO_STREAM("Voxel-izing stitched cloud...");
        filter_grid.setInputCloud(poses_cloud);
        filter_grid.filter(*filt_pose_cloud);
        *poses_cloud = *filt_pose_cloud;
        ROS_INFO_STREAM("Adding pose to graph.");
        poseGraph->add_vertex_previous(*odom,filt_pose_cloud);
    }
    cloud_pub.publish(poses_cloud);
    visualization_msgs::MarkerArray markerArray;
    poseGraph->get_graph_markers(markerArray.markers);
    graph_pub.publish(markerArray);
    poseGraph->write_graph_to_file();
}