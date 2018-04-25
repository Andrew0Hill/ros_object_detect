//
// Created by ros on 4/14/18.
//


#include <tf2_ros/transform_broadcaster.h>
#include "object_detect_node_sync.h"

int main(int argc, char** argv) {
    ros::init(argc,argv,"object_detect_sync");
    ROS_INFO_STREAM("Creating ObjectDetector object.");
    ObjectDetector detector;
    ROS_INFO_STREAM("Setup complete.");
    ros::spin();
    // Dump images to file
    for (auto type_it = detector.memory.type_dict.begin(); type_it != detector.memory.type_dict.end(); ++type_it) {
        for (auto instance_it = type_it->second->obj_insts.begin();
             instance_it != type_it->second->obj_insts.end(); ++instance_it) {
            int i = 0;
            for(auto img_it = (*instance_it)->images.begin(); img_it != (*instance_it)->images.end(); ++img_it){
                std::stringstream filename;
                filename << ClassMap::get_class(type_it->second->id) << "_" << (*instance_it)->get_id() << "_" << i << ".jpg";
                cv::imwrite(filename.str(),(*img_it)->image);
                ++i;
            }
        }
    }
    // close analytics file
    detector.analytics_file.close();
    // Dump Point cloud of all stitched poses.
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr final_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    auto pose_graph = detector.getPoseGraph();
    auto pose_graph_it = pose_graph->poses.begin();
    for(pose_graph_it; pose_graph_it < pose_graph->poses.end(); ++pose_graph_it)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        *temp_cloud = *((*pose_graph_it)->cloud);
        pcl::transformPointCloud(*temp_cloud,*temp_cloud,(*pose_graph_it)->base_to_world.matrix());
        *final_cloud += *temp_cloud;
    }
    pcl::io::savePCDFileASCII("final_cloud.pcd",*final_cloud);
    pose_graph->optimize_graph();
    pose_graph_it = pose_graph->poses.begin();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr optimized_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    for(pose_graph_it; pose_graph_it < pose_graph->poses.end(); ++pose_graph_it)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        *temp_cloud = *((*pose_graph_it)->cloud);
        pcl::transformPointCloud(*temp_cloud,*temp_cloud,(*pose_graph_it)->base_to_world.matrix());
        *optimized_cloud += *temp_cloud;
    }
    pcl::io::savePCDFileASCII("optimized_final_cloud.pcd",*optimized_cloud);
}

ObjectDetector::ObjectDetector(){
    node = ros::NodeHandle("~");
    // Set the first measurement boolean to true.
    first_measurement = true;
    // Set the frame number to 0.
    frame_num = 0;
    // Set up TF2 buffer and listener.
    buffer = std::make_shared<tf2_ros::Buffer>();
    listener = std::make_shared<tf2_ros::TransformListener>(*buffer);
    // Set up TF2 transform broadcaster.
    broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>();
    // Set up the transform.
    ts.header.frame_id = "odom";
    ts.child_frame_id = "map";
    ts.transform.rotation.x = 0;
    ts.transform.rotation.y = 0;
    ts.transform.rotation.z = 0;
    ts.transform.rotation.w = 1;
    // Open the file stream for analytics
    analytics_file.open("analytics.txt");
    // Create featurizer for images.
    featurizer = std::make_shared<ORB_Featurizer>();
    // Set ICP parameters.
    // Create new SVDScale estimator and pass it to the ICP implementation.
    transform_estimator = Trans_Est_2D::Ptr(new Trans_Est_2D);
    icp_gen.setTransformationEstimation(transform_estimator);
    int icp_iterations;
    double max_corr_dist, transform_eps, ransac_outlier_thresh;
    if(node.getParam("icp_max_iters",icp_iterations)){
        icp_gen.setMaximumIterations(icp_iterations);
    }else{
        icp_gen.setMaximumIterations(30);
    }
    ROS_INFO_STREAM("Default Transform Epsilon: " << icp_gen.getTransformationEpsilon());
    ROS_INFO_STREAM("Default Euclidean Fitness Epsilon: " << icp_gen.getEuclideanFitnessEpsilon());
    ROS_INFO_STREAM("Default RANSAC Outlier Rejection Threshold: " << icp_gen.getRANSACOutlierRejectionThreshold());
    ROS_INFO_STREAM("Default Max Corrsepondence Dist: " << icp_gen.getMaxCorrespondenceDistance());
    if(node.getParam("max_corr_dist",max_corr_dist)){
        icp_gen.setMaxCorrespondenceDistance(max_corr_dist);
    }else{
        icp_gen.setMaxCorrespondenceDistance(0.025);
    }
    if(node.getParam("transform_eps", transform_eps)){
        icp_gen.setTransformationEpsilon(transform_eps);
    }else{
        icp_gen.setTransformationEpsilon(1e-8);
    }

    //icp_gen.setEuclideanFitnessEpsilon (1);
    //icp_gen.setRANSACOutlierRejectionThreshold(1.5);
    ROS_INFO_STREAM("Transform Epsilon: " << icp_gen.getTransformationEpsilon());
    ROS_INFO_STREAM("Euclidean Fitness Epsilon: " << icp_gen.getEuclideanFitnessEpsilon());
    ROS_INFO_STREAM("RANSAC Outlier Rejection Threshold: " << icp_gen.getRANSACOutlierRejectionThreshold());
    ROS_INFO_STREAM("Max Corrsepondence Dist: " << icp_gen.getMaxCorrespondenceDistance());
    // Set Voxel Grid parameters.
    float voxel_grid_leaf_size;
    if(node.getParam("voxel_grid_leaf_size",voxel_grid_leaf_size)){
        filter_grid.setLeafSize(voxel_grid_leaf_size,voxel_grid_leaf_size,voxel_grid_leaf_size);
    }else {
        filter_grid.setLeafSize(0.08, 0.08, 0.08);
    }
    // Set up Normal Estimation object
    search_tree = pcl::search::KdTree<pcl::PointXYZRGB>::Ptr(new pcl::search::KdTree<pcl::PointXYZRGB>);
    norm_estimator.setSearchMethod(search_tree);
    norm_estimator.setKSearch(30);
    // Create Pose Graph.
    poseGraph = std::make_shared<PoseGraph>();
    // Create new Pose Cloud (Concatenated pose clouds).
    poses_cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    // Set up ApproxTimeSync subscribers
    cloud_sub = new message_filters::Subscriber<pcl::PointCloud<pcl::PointXYZRGB>>(node,"/camera2/cloudRGB/throttled",10);
    image_sub = new message_filters::Subscriber<sensor_msgs::Image>(node, "/camera/rgb/image_raw/throttled",10);
    odom_sub = new message_filters::Subscriber<nav_msgs::Odometry>(node, "/odom",200);
    // Set up main ApproxTimeSync subscriber.

    synchronizer = new message_filters::Synchronizer<frame_sync_policy>(frame_sync_policy(50),*image_sub,*cloud_sub,*odom_sub);
    // Register the callback for the subscriber.
    synchronizer->registerCallback(boost::bind(&ObjectDetector::frame_callback,this,_1,_2,_3));

    //signal(SIGINT, ObjectDetector::sigint_handler);
    // Create a new TF detection model.
    dm = std::make_shared<DetectionModel>();
    if(!node.getParam("min_feature_num",min_feature_num))
        min_feature_num = MIN_FEATURE_NUM;
    ROS_INFO_STREAM("Minimum Features: " << min_feature_num);
    // Advertise the detected objects topic.
    anomaly_pub = node.advertise<std_msgs::Int32MultiArray>("/objects",1);
    // Advertise the object labels topic.
    markers = node.advertise<visualization_msgs::MarkerArray>("/map_objects",1);
    // Advertise the cloud topic
    cloud_pub = node.advertise<pcl::PointCloud<pcl::PointXYZRGB>>("/full_cloud",1);
    // Advertise the graph nodes topic.
    graph_pub = node.advertise<visualization_msgs::MarkerArray>("/graph_nodes",1);
    // Start the separate thread to publish object labels.
    boost::thread inst_thread(boost::bind(&ObjectDetector::publish_objects_vis,this));
    boost::thread map_to_odom(boost::bind(&ObjectDetector::publish_map_trans,this));
}

void ObjectDetector::publish_objects_vis(){
    ROS_WARN("Entered Instances Thread.");
    ros::Rate loop_rate(1);
    // Get the memory lock
    while(ros::ok()) {
        std::vector<visualization_msgs::Marker> marker_vec;
        std::hash<std::string> id_gen;
        memory_lock.lock();
        for (auto type_it = memory.type_dict.begin(); type_it != memory.type_dict.end(); ++type_it) {
            for (auto instance_it = type_it->second->obj_insts.begin();
                 instance_it != type_it->second->obj_insts.end();
                 ++instance_it) {
                std::stringstream obj_strm;
                obj_strm << (*instance_it)->get_type() << " ID: " << (*instance_it)->get_id();
                std::string obj_str = obj_strm.str();
                //ROS_INFO_STREAM("Object Type: " << ClassMap::get_class(type_it->second->id) << "ID: " << (*instance_it)->get_id());
                visualization_msgs::Marker temp_mark;

                temp_mark.pose.position.x = (*instance_it)->world_pos[0];
                temp_mark.pose.position.y = (*instance_it)->world_pos[1];
                temp_mark.pose.position.z = (*instance_it)->world_pos[2] + 0.25;
                temp_mark.pose.orientation.x = 0.0;
                temp_mark.pose.orientation.y = 0.0;
                temp_mark.pose.orientation.z = 0.0;
                temp_mark.pose.orientation.w = 1.0;
                temp_mark.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
                temp_mark.scale.z = 0.25;
                temp_mark.text = obj_str;
                temp_mark.color.a = 1.0;
                temp_mark.color.r = 1.0;
                temp_mark.color.g = 1.0;
                temp_mark.color.b = 1.0;
                temp_mark.header.frame_id = "odom";
                temp_mark.header.stamp = ros::Time();
                temp_mark.lifetime = ros::Duration(1.5);
                temp_mark.id = id_gen(obj_str);
                marker_vec.push_back(temp_mark);
            }
        }
        // Release the memory lock
        memory_lock.unlock();
        visualization_msgs::MarkerArray marker_msg;
        marker_msg.markers = marker_vec;
        markers.publish(marker_msg);
        loop_rate.sleep();
    }
    ROS_WARN("Exited Instances Thread.");
}
void ObjectDetector::publish_map_trans() {
    ros::Rate loop_rate(20);
    while(ros::ok()) {
        tf_lock.lock_shared();
        ts.header.stamp = ros::Time::now();
        broadcaster->sendTransform(ts);
        tf_lock.unlock_shared();
        loop_rate.sleep();
    }
}
void ObjectDetector::sigint_handler(int code){
    // Dump images associated with each object instance.
    ROS_INFO("SIGINT received. Dumping images.");
    for (auto type_it = memory.type_dict.begin(); type_it != memory.type_dict.end(); ++type_it) {
        for (auto instance_it = type_it->second->obj_insts.begin();
             instance_it != type_it->second->obj_insts.end(); ++instance_it) {
            int i = 0;
            for(auto img_it = (*instance_it)->images.begin(); img_it != (*instance_it)->images.end(); ++img_it){
                std::stringstream filename;
                filename << ClassMap::get_class(type_it->second->id) << "_" << (*instance_it)->get_id() << "_" << i << ".jpg";
                cv::imwrite(filename.str(),(*img_it)->image);
                ++i;
            }
        }
    }
    ros::shutdown();
}
void ObjectDetector::frame_callback(const sensor_msgs::Image::ConstPtr& rgb,
                    const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud,
                    const nav_msgs::Odometry::ConstPtr& odom){
    double start_time = std::clock();
    double loop_closure_time = -1;
    ROS_INFO_STREAM("Frame Callback!");

    // Check if we should use this set of measurements as a keyframe (i.e. if the pose is different enough from the
    // previous pose.
    if (first_measurement){
        ROS_INFO_STREAM("First pose!");
        Eigen::Translation3d coords(odom->pose.pose.position.x,
                                    odom->pose.pose.position.y,
                                    odom->pose.pose.position.z);
        Eigen::Quaterniond rotation(odom->pose.pose.orientation.w,
                                    odom->pose.pose.orientation.x,
                                    odom->pose.pose.orientation.y,
                                    odom->pose.pose.orientation.z);

        origin_pose = Eigen::Affine3d(Eigen::Translation3d(odom->pose.pose.position.x,
                                                           odom->pose.pose.position.y,
                                                           odom->pose.pose.position.z) *
                                      Eigen::Quaterniond(odom->pose.pose.orientation.w,
                                                         odom->pose.pose.orientation.x,
                                                         odom->pose.pose.orientation.y,
                                                         odom->pose.pose.orientation.z));
        prev_pose = *odom;
    }
    else if(!odom_has_changed(odom)){
        ROS_INFO_STREAM("Odometry hasn't changed. Skipping adding keyframe.");
        return;
    }
    // Voxel filter the cloud.
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr current_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    filter_grid.setInputCloud(cloud);
    filter_grid.filter(*current_cloud);
    // Estimate normals for the cloud.
    //pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    //pcl::copyPointCloud(*current_cloud,*xyz_cloud);
    pcl::PointCloud<pcl::PointNormal>::Ptr normal_cloud(new pcl::PointCloud<pcl::PointNormal>);
    norm_estimator.setInputCloud(current_cloud);
    norm_estimator.compute(*normal_cloud);
    pcl::copyPointCloud(*current_cloud,*normal_cloud);
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

    Eigen::Quaterniond b2w_rot(odom->pose.pose.orientation.w,
                               odom->pose.pose.orientation.x,
                               odom->pose.pose.orientation.y,
                               odom->pose.pose.orientation.z);

    Eigen::Translation3d b2w_tl(odom->pose.pose.position.x,
                                odom->pose.pose.position.y,
                                odom->pose.pose.position.z);

    Eigen::Affine3d b2w_trans(b2w_tl * b2w_rot);

    Eigen::Affine3d c2w_trans = b2w_trans * c2b_trans;
    /*Eigen::Affine3d origin_trans;
    if(first_measurement) {
        origin_trans = c2b_trans;
    }else{
        origin_trans = origin_pose.inverse() * c2b_trans;
    }*/
    pcl::transformPointCloud(*current_cloud,*current_cloud,c2b_trans);
    pcl::transformPointCloud(*normal_cloud,*normal_cloud,c2b_trans);
    //pcl::transformPointCloud(*current_cloud,*current_cloud,c2w_trans);
    current_cloud->header.frame_id = "odom";
    normal_cloud->header.frame_id = "odom";
    std::shared_ptr<Pose> pose;

    geometry_msgs::TransformStamped odom_to_map;
    try {
        odom_to_map = buffer->lookupTransform("map","odom",ros::Time(0));
    }catch(tf2::TransformException &te){
        ROS_WARN_STREAM(te.what());
    }

    Eigen::Affine3d o2m_trans(Eigen::Translation3d(odom_to_map.transform.translation.x,
                                                     odom_to_map.transform.translation.y,
                                                   odom_to_map.transform.translation.z) *
                                Eigen::Quaterniond(odom_to_map.transform.rotation.w,
                                                   odom_to_map.transform.rotation.x,
                                                   odom_to_map.transform.rotation.y,
                                                   odom_to_map.transform.rotation.z));

    Eigen::Affine3d rect_odom = o2m_trans * b2w_trans;

    nav_msgs::Odometry rel_odom;
    rel_odom.pose.pose.position.x = rect_odom.translation().x();
    rel_odom.pose.pose.position.y = rect_odom.translation().y();
    rel_odom.pose.pose.position.z = 0;

    Eigen::Quaterniond rel_rot(rect_odom.rotation());

    rel_odom.pose.pose.orientation.x = rel_rot.x();
    rel_odom.pose.pose.orientation.y = rel_rot.y();
    rel_odom.pose.pose.orientation.z = rel_rot.z();
    rel_odom.pose.pose.orientation.w = rel_rot.w();


    if(first_measurement){
        ROS_INFO_STREAM("Pose graph is empty! Adding first pose...");
        ROS_INFO_STREAM("Pose cloud is empty. Adding first cloud...");
        *poses_cloud = *current_cloud;
        pose = poseGraph->add_vertex(rel_odom,current_cloud,normal_cloud);
        first_measurement = false;
    }else{
        /*
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
        pcl::io::savePCDFileASCII("/home/ros/catkin_ws/devel/lib/object_detect/cloud_" + std::to_string(frame_num)+ ".pcd", *out_cloud);
        ROS_INFO_STREAM("Voxel-izing stitched cloud...");
        filter_grid.setInputCloud(poses_cloud);
        filter_grid.filter(*filt_pose_cloud);
        *poses_cloud = *filt_pose_cloud;
        ROS_INFO_STREAM("Adding pose to graph.");
        pose = poseGraph->add_vertex_previous(*odom,out_cloud);
         */
        pose = poseGraph->add_vertex_previous(rel_odom,current_cloud,normal_cloud);
    }
    visualization_msgs::MarkerArray markerArray;
    poseGraph->get_graph_markers(markerArray.markers);
    prev_pose = rel_odom;
    graph_pub.publish(markerArray);
    //cloud_pub.publish(poses_cloud);
    //poseGraph->write_graph_to_file();

    /*
     * Pose Graph End
     */


    /*
     * Image Matching Start
     */


    // Get image pointer
    cv_bridge::CvImagePtr  im_ptr = cv_bridge::toCvCopy(rgb);
    std::vector<std::shared_ptr<DetectedObject>> objects;
    std::vector<int> object_vals(27,0);
    // Object detection on image.
    objects = dm->detectImage(im_ptr->image);
    std::shared_ptr<DetectedObject> matched_object = NULL;
    // Print out list of objects detected in frame
    for (int i = 0; i < objects.size(); ++i) {
        object_vals[ClassMap::get_class_index(objects[i]->oclass)] += 1;
        // Print out object name and info.
        ROS_INFO_STREAM(objects[i]->to_string());
        // Featurize RGB image.
        // TODO: Match with mask? add a parameter to the featurizeImage() call.
        featurizer->featurizeImage(objects[i]->image, objects[i]->keypoints, objects[i]->descriptors);
        // Print # of Keypoints
        //std::cout << kps.size() << std::endl;

        // If enough features?
        if (objects[i]->keypoints.size() < min_feature_num) {
            ROS_INFO_STREAM("Object does not contain enough features! Need: " << min_feature_num << " Found: "
                                                                              << objects[i]->keypoints.size());
            continue;
        }

        objects[i]->pose = pose;

        //std::cout << "Object has: " << objects[i]->descriptors.cols << " columns and: " << objects[i]->descriptors.rows << " rows." << std::endl;
        //std::cout << objects[i] -> descriptors;
        // Get depth image ROI and centroid
        std::vector<int> obj_indices;

        ROS_INFO_STREAM("Cloud size: " << cloud->size());
        for (int j = 0; j < objects[i]->mask.rows; ++j) {
            for (int k = 0; k < objects[i]->mask.cols; ++k) {
                int num = objects[i]->mask.at<uchar>(j, k);
                if (objects[i]->mask.at<uchar>(j, k) == 255) {
                    int x_low = objects[i]->xmin;
                    int y_low = objects[i]->ymin;

                    obj_indices.push_back((y_low + j) * 640 + (x_low + k));
                }
            }
        }
        int size = obj_indices.size();
        Eigen::Matrix<double, 4, 1> centroid;
        int num = pcl::compute3DCentroid(*cloud, obj_indices, centroid);

        //ROS_INFO_STREAM("Centroid: " << centroid << " points used: " << num);
        geometry_msgs::TransformStamped st;

        try {
            st = buffer->lookupTransform("odom", "camera2_depth_optical_frame",
                                         pcl_conversions::fromPCL(cloud->header.stamp));
        } catch (tf2::TransformException &te) {
            ROS_WARN_STREAM(te.what());
        }

        Eigen::Affine3d eigen_trans(Eigen::Translation3d(st.transform.translation.x,
                                                         st.transform.translation.y,
                                                         st.transform.translation.z) *
                                    Eigen::Quaterniond(st.transform.rotation.w,
                                                       st.transform.rotation.x,
                                                       st.transform.rotation.y,
                                                       st.transform.rotation.z));

        Eigen::Matrix<double, 4, 1> output_position = eigen_trans * centroid;
        // Set the position in world space of this object.
        objects[i]->world_pos = output_position;


        // Match object into memory.
        //ROS_INFO_STREAM("Matching object into Memory...");
        memory_lock.lock();
        memory.match(objects[i], matched_object);
        memory_lock.unlock();

        /*
         * Image Matching End
         */

        /*
         * Loop Closure Detection
         */
        bool loop_closure;
        node.param<bool>("loop_closure", loop_closure, false);
        if (loop_closure) {
            if (matched_object != NULL) {
                loop_closure_time = std::clock();
                ROS_WARN_STREAM("LOOP CLOSURE DETECTED between" << pose->id << " and " << matched_object->pose->id);
                //pcl::PointCloud<pcl::PointXYZRGB>::Ptr aligned_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
                pcl::PointCloud<pcl::PointNormal>::Ptr aligned_cloud(new pcl::PointCloud<pcl::PointNormal>);
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr pub_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
                //pcl::PointCloud<pcl::PointNormal>::Ptr pub_norm_cloud(new pcl::PointCloud<pcl::PointNormal>);
                pcl::PointCloud<pcl::PointNormal>::Ptr copy_pose_normal_cloud(new pcl::PointCloud<pcl::PointNormal>);
                pcl::copyPointCloud(*pose->normal_cloud, *copy_pose_normal_cloud);

                pcl::PointCloud<pcl::PointXYZRGB>::Ptr copy_pose_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
                //pcl::copyPointCloud(*pose->cloud, *copy_pose_cloud);
                //icp_gen.setInputTarget(pose->cloud);
                //icp_gen.setInputSource(matched_object->pose->cloud);
                Eigen::Affine3f node_rel_trans = (matched_object->pose->base_to_world.inverse() *
                                                  pose->base_to_world).cast<float>();

                pcl::PointCloud<pcl::PointNormal>::Ptr output_odom_cloud(new pcl::PointCloud<pcl::PointNormal>);
                pcl::copyPointCloud(*copy_pose_normal_cloud, *output_odom_cloud);
                pcl::transformPointCloud(*output_odom_cloud, *output_odom_cloud, node_rel_trans);
                //pcl::transformPointCloud(*copy_pose_normal_cloud,*copy_pose_normal_cloud,matched_object->pose->base_to_world);
                //pcl::transformPointCloud(*copy_pose_cloud,*copy_pose_cloud,matched_object->pose->base_to_world);
                pcl::io::savePCDFileASCII(
                        "/home/ros/catkin_ws/devel/lib/object_detect/cloud_" + std::to_string(pose->id) + "_" +
                        std::to_string(matched_object->pose->id) + "_src.pcd", *output_odom_cloud);
                pcl::io::savePCDFileASCII(
                        "/home/ros/catkin_ws/devel/lib/object_detect/cloud_" + std::to_string(pose->id) + "_" +
                        std::to_string(matched_object->pose->id) + "_tgt.pcd", *matched_object->pose->normal_cloud);

                //icp_gen.setInputTarget(matched_object->pose->normal_cloud);
                //icp_gen.setInputSource(copy_pose_normal_cloud);
                //icp_gen.align(*aligned_cloud,node_rel_trans.matrix());

                icp_gen.setInputTarget(matched_object->pose->normal_cloud);
                icp_gen.setInputSource(copy_pose_normal_cloud);
                icp_gen.align(*aligned_cloud, node_rel_trans.matrix());
                ROS_ERROR_STREAM("ICP Fitness Score: " << icp_gen.getFitnessScore());
                if (icp_gen.hasConverged() && icp_gen.getFitnessScore() < 0.05 && pose->id - matched_object->pose->id > 40) {
                    //pcl::transformPointCloud(*copy_pose_cloud,*copy_pose_cloud,icp_gen.getFinalTransformation());
                    pcl::transformPointCloud(*copy_pose_cloud, *copy_pose_cloud, icp_gen.getFinalTransformation());
                    pcl::copyPointCloud(*matched_object->pose->cloud, *pub_cloud);
                    *pub_cloud += *copy_pose_cloud;

                    cloud_pub.publish(pub_cloud);
                    // Make a Affine transformation from the ICP results.
                    Eigen::Affine3f relative_trans(icp_gen.getFinalTransformation());
                    // Make a Vector3d for the graph optimizer.
                    float theta = relative_trans.rotation().eulerAngles(0, 1, 2)(2);
                    Eigen::Vector3d relative_trans_vec(relative_trans.translation().x(),
                                                       relative_trans.translation().y(),
                                                       theta);
                    Eigen::Matrix3d covariance = Eigen::Matrix3d::Identity();
                    Eigen::Affine3d old_pose = poseGraph->poses.back()->base_to_world;
                    poseGraph->add_edge(matched_object->pose, pose, relative_trans_vec, covariance, true);
                    ROS_WARN_STREAM(
                            "Relative Transformation: " << relative_trans.matrix() << std::endl
                                                        << relative_trans.translation().x()
                                                        << " " << relative_trans.translation().y() << " "
                                                        << theta);

                    ROS_WARN_STREAM("Node Transformation: " << "X: " << node_rel_trans.translation().x() << " Y: "
                                                            << node_rel_trans.translation().y() << " Theta: "
                                                            << node_rel_trans.rotation().eulerAngles(0, 1, 2)(2));

                    poseGraph->optimize_graph();
                    Eigen::Affine3d new_pose = poseGraph->poses.back()->base_to_world;
                    // Set up relative transform between map and odom.

                    Eigen::Affine3d correction = old_pose.inverse() * new_pose;
                    Eigen::Quaterniond rotation(correction.rotation());
                    tf_lock.lock();
                    ROS_ERROR_STREAM("MODIFIED TRANFORM");
                    ts.transform.translation.x = correction.translation()(0);
                    ts.transform.translation.y = correction.translation()(1);
                    ts.transform.rotation.x = rotation.x();
                    ts.transform.rotation.y = rotation.y();
                    ts.transform.rotation.z = rotation.z();
                    ts.transform.rotation.w = rotation.w();
                    tf_lock.unlock();
                } else {
                    ROS_WARN_STREAM("Could not compute loop closure transformation between poses!");
                }
            }
        }
    }
    double end_total_time = std::clock() - start_time;
    double end_loop_closure_time = loop_closure_time == -1 ? -1 : std::clock() - loop_closure_time;
    analytics_file << frame_num << "," << (end_total_time/CLOCKS_PER_SEC) << "," << (end_loop_closure_time/CLOCKS_PER_SEC) << std::endl;
    std_msgs::Int32MultiArray obj_msg;
    obj_msg.data = object_vals;
    anomaly_pub.publish(obj_msg);
    poseGraph->write_graph_to_file();
    frame_num++;

}

