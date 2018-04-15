//
// Created by ros on 4/14/18.
//


#include "object_detect_node_sync.h"

int main(int argc, char** argv) {
    ros::init(argc,argv,"sync_obj_detect");
    ROS_INFO_STREAM("Creating ObjectDetector object.");
    ObjectDetector detector;
    ROS_INFO_STREAM("Setup complete.");
    ros::spin();
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
}

ObjectDetector::ObjectDetector(){
    // Set the first measurement boolean to true.
    first_measurement = true;
    // Set the frame number to 0.
    frame_num = 0;
    // Set up TF2 buffer and listener.
    buffer = std::make_shared<tf2_ros::Buffer>();
    listener = std::make_shared<tf2_ros::TransformListener>(*buffer);
    // Create featurizer for images.
    featurizer = std::make_shared<ORB_Featurizer>();
    // Set ICP parameters.
    icp_gen.setMaximumIterations(30);
    icp_gen.setMaxCorrespondenceDistance(0.05);
    icp_gen.setTransformationEpsilon(1e-8);
    icp_gen.setEuclideanFitnessEpsilon (1);
    icp_gen.setRANSACOutlierRejectionThreshold(1.5);
    // Set Voxel Grid parameters.
    filter_grid.setLeafSize(0.08,0.08,0.08);
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

    // Advertise the detected objects topic.
    anomaly_pub = node.advertise<std_msgs::Int32MultiArray>("objects",1);
    // Advertise the object labels topic.
    markers = node.advertise<visualization_msgs::MarkerArray>("map_objects",1);
    // Advertise the cloud topic
    cloud_pub = node.advertise<pcl::PointCloud<pcl::PointXYZRGB>>("/full_cloud",1);
    // Advertise the graph nodes topic.
    graph_pub = node.advertise<visualization_msgs::MarkerArray>("/graph_nodes",1);
    // Start the separate thread to publish object labels.
    boost::thread inst_thread(boost::bind(&ObjectDetector::publish_objects_vis,this));
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
    ROS_INFO_STREAM("Frame Callback!");

    // Check if we should use this set of measurements as a keyframe (i.e. if the pose is different enough from the
    // previous pose.
    if (first_measurement){
        ROS_INFO_STREAM("First pose!");
    }
    else if(!odom_has_changed(odom)){
        ROS_INFO_STREAM("Odometry hasn't changed. Skipping adding keyframe.");
        return;
    }
    // Voxel filter the cloud.
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
    //cloud_pub.publish(poses_cloud);
    poseGraph->write_graph_to_file();

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
    for (int i = 0; i < objects.size(); ++i){
        object_vals[ClassMap::get_class_index(objects[i]->oclass)] += 1;
        // Print out object name and info.
        ROS_INFO_STREAM(objects[i]->to_string());
        // Featurize RGB image.
        // TODO: Match with mask? add a parameter to the featurizeImage() call.
       featurizer->featurizeImage(objects[i]->image,objects[i]->keypoints,objects[i]->descriptors);
        // Print # of Keypoints
        //std::cout << kps.size() << std::endl;

        // If enough features?
        if (objects[i]->keypoints.size() < MIN_FEATURE_NUM){
            ROS_INFO_STREAM("Object does not contain enough features! Need: " << MIN_FEATURE_NUM << " Found: " << objects[i]->keypoints.size());
            continue;
        }

        objects[i]->pose = pose;

        std::cout << "Object has: " << objects[i]->descriptors.cols << " columns and: " << objects[i]->descriptors.rows << " rows." << std::endl;
       //std::cout << objects[i] -> descriptors;
        // Get depth image ROI and centroid
        std::vector<int> obj_indices;

        ROS_INFO_STREAM("Cloud size: " << cloud->size());
        for (int j = 0; j < objects[i]->mask.rows; ++j){
            for (int k = 0; k < objects[i]->mask.cols; ++k){
                int num = objects[i]->mask.at<uchar>(j,k);
                if(objects[i]->mask.at<uchar>(j,k) == 255){
                    int x_low = objects[i]->xmin;
                    int y_low = objects[i]->ymin;

                    obj_indices.push_back((y_low + j) * 640 + (x_low + k));
                }
            }
        }
        int size = obj_indices.size();
        Eigen::Matrix<double,4,1> centroid;
        int num = pcl::compute3DCentroid(*cloud,obj_indices,centroid);

        ROS_INFO_STREAM("Centroid: " << centroid << " points used: " << num);
        geometry_msgs::TransformStamped st;

        try {
            st = buffer->lookupTransform("odom", "camera2_depth_optical_frame", pcl_conversions::fromPCL(cloud->header.stamp));
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

        Eigen::Matrix<double,4,1> output_position = eigen_trans * centroid;
        // Set the position in world space of this object.
        objects[i]->world_pos = output_position;


        // Match object into memory.
        ROS_INFO_STREAM("Matching object into Memory...");
        memory_lock.lock();
        memory.match(objects[i],matched_object);
        memory_lock.unlock();

        /*
         * Image Matching End
         */

        /*
         * Loop Closure Detection
         */
        if(matched_object != NULL){
            ROS_WARN_STREAM("LOOP CLOSURE DETECTED between" << pose->id << " and " << matched_object->pose->id);
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr aligned_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr pub_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

            //icp_gen.setInputTarget(pose->cloud);
            //icp_gen.setInputSource(matched_object->pose->cloud);
            icp_gen.setInputTarget(matched_object->pose->cloud);
            icp_gen.setInputSource(pose->cloud);
            icp_gen.align(*aligned_cloud);



            if(icp_gen.hasConverged()) {
                *pub_cloud = *(pose->cloud);
                *pub_cloud += *aligned_cloud;
                cloud_pub.publish(pub_cloud);
                // Make a Affine transformation from the ICP results.
                Eigen::Affine3f relative_trans(icp_gen.getFinalTransformation());
                // Make a Vector3d for the graph optimizer.
                Eigen::Vector3d relative_trans_vec(relative_trans.translation().x(),
                                                   relative_trans.translation().y(),
                                                   relative_trans.rotation().eulerAngles(0, 1, 2)(2));
                Eigen::Matrix3d covariance = Eigen::Matrix3d::Identity();
                poseGraph->add_edge(pose,matched_object->pose,relative_trans_vec,covariance,true);
                ROS_WARN_STREAM(
                        "Relative Transformation: " << relative_trans.matrix() << relative_trans.translation().x()
                                                    << " " << relative_trans.translation().y() << " "
                                                    << relative_trans.rotation().eulerAngles(0, 1, 2)(2));
                poseGraph->optimize_graph();
            }else{
                ROS_WARN_STREAM("Could not compute loop closure transformation between poses!");
            }
        }
    }


    std_msgs::Int32MultiArray obj_msg;
    obj_msg.data = object_vals;
    anomaly_pub.publish(obj_msg);
    frame_num++;

}

