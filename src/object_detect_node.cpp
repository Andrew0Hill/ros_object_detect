//
// Created by ros on 3/1/18.
//

#include <nav_msgs/Odometry.h>
#include "../include/object_detect/object_detect_node.h"


int frame_num = 0;
std::shared_ptr<DetectionModel> dm;
ros::Publisher p,markers;
auto featurizer = std::make_shared<ORB_Featurizer>();
Memory memory;
std::shared_ptr<tf2_ros::Buffer> buffer;
std::shared_ptr<tf2_ros::TransformListener> listener;
boost::shared_mutex writer_lock;
boost::shared_mutex memory_lock;
pcl::PointCloud<pcl::PointXYZ> obj_cloud;
cv::Mat image;

//pcl::ExtractIndices extractor = pcl::ExtractIndices();
void frame_callback(const sensor_msgs::Image::ConstPtr& rgb, const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud, const nav_msgs::Odometry::ConstPtr& odom) {
    // Get image pointer

    cv_bridge::CvImagePtr  im_ptr = cv_bridge::toCvCopy(rgb);
    std::vector<std::shared_ptr<DetectedObject>> objects;
    std::vector<int> object_vals(27,0);
    // Object detection on image.
    objects = dm->detectImage(im_ptr->image);

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

        std::shared_ptr<DetectedObject> FIX_THIS_LATER;
        // Match object into memory.
        ROS_INFO_STREAM("Matching object into Memory...");
        memory_lock.lock();
        memory.match(objects[i], FIX_THIS_LATER);
        memory_lock.unlock();
    }

    std_msgs::String s;
    s.data = "test";
    std_msgs::Int32MultiArray obj_msg;
    obj_msg.data = object_vals;
    p.publish(obj_msg);
    frame_num++;
}
void publish_objects_vis(){
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
void on_sigint(int code){
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
int main(int argc, char** argv) {
    // Initialize the ROS node.
    ros::init(argc, argv, "hybrid_matcher", ros::init_options::NoSigintHandler);
    buffer = std::make_shared<tf2_ros::Buffer>();
    listener = std::make_shared<tf2_ros::TransformListener>(*buffer);
    ros::NodeHandle node;
    //image_transport::ImageTransport itnode(node);

    // Create two message filter subscribers: One for point clouds, and one for Images.
    message_filters::Subscriber<pcl::PointCloud<pcl::PointXYZRGB>> cloud_sub(node,"/camera2/cloudRGB/throttled",10);
    message_filters::Subscriber<sensor_msgs::Image> image_sub(node, "/camera/rgb/image_raw/throttled",10);
    message_filters::Subscriber<nav_msgs::Odometry> odom_sub(node, "/odom",200);
    // Create the approximate time synchronizer.
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, pcl::PointCloud<pcl::PointXYZRGB>, nav_msgs::Odometry> frame_sync_policy;
    message_filters::Synchronizer<frame_sync_policy> synchronizer(frame_sync_policy(50),image_sub,cloud_sub,odom_sub);
    synchronizer.registerCallback(boost::bind(&frame_callback,_1,_2,_3));

    // Create a custom SIGINT handler, so we can dump images when we shut down the node.
    signal(SIGINT, on_sigint);
    // Create a new TF detection model.
    dm = std::make_shared<DetectionModel>();

    // Advertise the detected objects topic.
    p = node.advertise<std_msgs::Int32MultiArray>("objects",1);
    markers = node.advertise<visualization_msgs::MarkerArray>("map_objects",1);
    boost::thread inst_thread(publish_objects_vis);
    ros::spin();

    cv::destroyAllWindows();
}

