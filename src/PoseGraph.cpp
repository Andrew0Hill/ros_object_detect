//
// Created by ros on 4/5/18.
//

#include "PoseGraph.h"

bool PoseGraph::add_vertex_previous(nav_msgs::Odometry odom, std::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cloud){

    Eigen::Affine3d world_transform(Eigen::Translation3d(odom.pose.pose.position.x,
                                                         odom.pose.pose.position.y,
                                                         odom.pose.pose.position.z) *
                                    Eigen::Quaterniond(odom.pose.pose.orientation.w,
                                                       odom.pose.pose.orientation.x,
                                                       odom.pose.pose.orientation.y,
                                                       odom.pose.pose.orientation.z));
    geometry_msgs::TransformStamped st = buffer->lookupTransform("base_link","camera2_depth_optical_frame",ros::Time());

    Eigen::Affine3d camera_transform(Eigen::Translation3d(st.transform.translation.x,
                                                          st.transform.translation.y,
                                                          st.transform.translation.z) *
                                     Eigen::Quaterniond(st.transform.rotation.w,
                                                        st.transform.rotation.x,
                                                        st.transform.rotation.y,
                                                        st.transform.rotation.z));

    std::shared_ptr<Pose> new_pose = std::make_shared<Pose>(camera_transform,world_transform,cloud);

    poses.push_back(new_pose);

    return true;
}
