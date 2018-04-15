//
// Created by ros on 4/15/18.
//

#include "slam_node.h"

int main(int argc, char** argv){
    ros::init(argc,argv,"slam_node");
    SLAMNode slamNode;
    ros::spin();

}
SLAMNode::SLAMNode() {
    first_measurement = true;
    // Create featurizer
    featurizer = std::make_shared<ORB_Featurizer>();

    // Create buffer
    buffer = std::make_shared<tf2_ros::Buffer>();

    // Create listener and pass it the buffer.
    listener = std::make_shared<tf2_ros::TransformListener>(*buffer);

    // Create the pose graph
    poseGraph = std::make_shared<PoseGraph>();

    // Create the poses cloud
    poses_cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

    // Set ICP parameters.
    icp_gen.setMaximumIterations(30);
    icp_gen.setMaxCorrespondenceDistance(0.05);
    icp_gen.setTransformationEpsilon(1e-8);
    icp_gen.setEuclideanFitnessEpsilon (1);
    icp_gen.setRANSACOutlierRejectionThreshold(1.5);

    // Set Voxel Grid parameters.
    filter_grid.setLeafSize(0.08,0.08,0.08);

    // Set up circular buffer
    //images = boost::circular_buffer_space_optimized<std::shared_ptr<FeaturizedImage>>(500);
    // Set up ApproxTimeSync subscribers
    cloud_sub = new message_filters::Subscriber<pcl::PointCloud<pcl::PointXYZRGB>>(node,"/camera2/cloudRGB/throttled",10);
    image_sub = new message_filters::Subscriber<sensor_msgs::Image>(node, "/camera/rgb/image_raw/throttled",10);
    odom_sub = new message_filters::Subscriber<nav_msgs::Odometry>(node, "/odom",200);

    // Set up main ApproxTimeSync subscriber.
    synchronizer = new message_filters::Synchronizer<frame_sync_policy>(frame_sync_policy(50),*image_sub,*cloud_sub,*odom_sub);

    // Register the callback for the subscriber.
    synchronizer->registerCallback(boost::bind(&SLAMNode::frame_callback,this,_1,_2,_3));

    // Create a matcher and set it up to use ORB descriptors.
    matcher = cv::FlannBasedMatcher(new cv::flann::LshIndexParams(20,15,2));

    // Advertise the cloud topic
    cloud_pub = node.advertise<pcl::PointCloud<pcl::PointXYZRGB>>("/full_cloud",1);
    // Advertise the graph nodes topic.
    graph_pub = node.advertise<visualization_msgs::MarkerArray>("/graph_nodes",1);

}
void
SLAMNode::frame_callback(const sensor_msgs::Image::ConstPtr &rgb,
                         const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud,
                         const nav_msgs::Odometry::ConstPtr &odom) {
    // Pointer to hold the voxelized version of the cloud in this message, since we can't modify it.
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr current_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    filter_grid.setInputCloud(cloud);
    filter_grid.filter(*current_cloud);

    /*
     * Pose Graph Start
     */
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

    pcl::transformPointCloud(*current_cloud,*current_cloud,c2w_trans);
    current_cloud->header.frame_id = "odom";

    std::shared_ptr<Pose> pose;
    if(first_measurement){
        ROS_INFO_STREAM("Pose graph is empty! Adding first pose...");
        ROS_INFO_STREAM("Pose cloud is empty. Adding first cloud...");
        *poses_cloud = *current_cloud;
        pose = poseGraph->add_vertex(*odom,current_cloud);
        first_measurement = false;
    }else{
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr filt_pose_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

        ROS_INFO_STREAM("Adding new pose. Graph has " << poseGraph->poses.size() << " nodes and "
                                                      << poseGraph->edges.size() << " edges.");
        ROS_INFO_STREAM("Finished Filter, filtered cloud size is: " << current_cloud->size() << " from initial size of: " << cloud->size());
        ROS_INFO_STREAM("Started ICP alignment.");
        icp_gen.setInputTarget(poses_cloud);
        icp_gen.setInputSource(current_cloud);
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
        pose = poseGraph->add_vertex_previous(*odom,out_cloud);
    }
    visualization_msgs::MarkerArray markerArray;
    poseGraph->get_graph_markers(markerArray.markers);
    graph_pub.publish(markerArray);
    cloud_pub.publish(poses_cloud);
    poseGraph->write_graph_to_file();

    /*
     * Pose Graph End
     */

    /*
     * Image Matching Start
     */
    // Pointer to image.
    cv_bridge::CvImagePtr im_ptr = cv_bridge::toCvCopy(rgb);

    // Create FeaturizedImage object and featurized the current image.
    std::shared_ptr<FeaturizedImage> featurized_image = std::make_shared<FeaturizedImage>();
    featurizer->featurizeImage(im_ptr->image,featurized_image->keypoints, featurized_image->descriptors);
    featurized_image->pose = pose;
    images.push_back(featurized_image);
    // Try to detect loop closures between this image and all previous images.
    for(int i = 0; i < images.size()-1; ++i){
        // Match the image we featurized to the current image.
        std::vector<std::vector<cv::DMatch>> matches;
        matcher.knnMatch(featurized_image->descriptors,images[i]->descriptors,matches,2);

        // Filter the bad matches
        std::vector<cv::DMatch> filt_matches;
        for(int j = 0; j < matches.size(); ++j){
            if(matches[j].size() == 2 && matches[j][0].distance < matches[j][1].distance * NN_THRESH){
                filt_matches.push_back(matches[j][0]);
            }
        }

        if(filt_matches.size() < MIN_FEATURES){
            ROS_WARN("Not enough features! Skipping match");
            continue;
        }

        // Perform geometric matching with findHomography
        std::vector<cv::Point2f> query_pts;
        std::vector<cv::Point2f> train_pts;

        // Get filtered matching points from both sets.
        for(int j = 0; j < filt_matches.size(); ++j){
            query_pts.push_back(featurized_image->keypoints[filt_matches[j].queryIdx].pt);
            train_pts.push_back(images[i]->keypoints[filt_matches[j].trainIdx].pt);
        }

        cv::Mat mask;

        cv::findHomography(query_pts,train_pts,CV_RANSAC,3,mask);

        // Loop Closure
        if(cv::countNonZero(mask) > INLIERS_THRESH){
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr aligned_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
            // Get ICP transformation between the two matching poses.
            icp_gen.setInputSource(pose->cloud);
            icp_gen.setInputTarget(images[i]->pose->cloud);
            icp_gen.align(*aligned_cloud);

            if(icp_gen.hasConverged()) {
                // Make a Affine transformation from the ICP results.
                Eigen::Affine3f relative_trans(icp_gen.getFinalTransformation());
                // Make a Vector3d for the graph optimizer.
                Eigen::Vector3d relative_trans_vec(relative_trans.translation().x(),
                                                   relative_trans.translation().y(),
                                                   relative_trans.rotation().eulerAngles(0, 1, 2)(2));
                Eigen::Matrix3d covariance = Eigen::Matrix3d::Identity();
                poseGraph->add_edge(pose,images[i]->pose,relative_trans_vec,covariance,true);
                ROS_WARN_STREAM(
                        "Relative Transformation: " << relative_trans.matrix() << relative_trans.translation().x()
                                                    << " " << relative_trans.translation().y() << " "
                                                    << relative_trans.rotation().eulerAngles(0, 1, 2)(2));
                poseGraph->optimize_graph();
                return;
            }else{
                ROS_WARN_STREAM("Could not compute loop closure transformation between poses!");
            }
        }

    }
}


