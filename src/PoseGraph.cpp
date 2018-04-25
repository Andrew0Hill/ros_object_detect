//
// Created by ros on 4/5/18.
//

#include "PoseGraph.h"

std::shared_ptr<Pose> PoseGraph::add_vertex(nav_msgs::Odometry odom,
                                            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr,
                                            pcl::PointCloud<pcl::PointNormal>::Ptr normal_ptr){

    Eigen::Translation3d coords(odom.pose.pose.position.x,
                                odom.pose.pose.position.y,
                                odom.pose.pose.position.z);
    Eigen::Quaterniond rotation(odom.pose.pose.orientation.w,
                                odom.pose.pose.orientation.x,
                                odom.pose.pose.orientation.y,
                                odom.pose.pose.orientation.z);
    Eigen::Affine3d world_transform(coords * rotation);
    std::shared_ptr<Pose> pose = std::make_shared<Pose>(this->alloc_id++,
                                                        world_transform,
                                                        coords,
                                                        rotation,
                                                        odom.header.stamp,
                                                        cloud_ptr,
                                                        normal_ptr);
    optimizer.addVertex(coords.vector());
    poses.push_back(pose);
    return pose;
}

bool PoseGraph::add_edge(std::shared_ptr<Pose> p1, std::shared_ptr<Pose> p2, Eigen::Vector3d relativeTransform, Eigen::Matrix3d covariance, bool loop_closure){
    // Add edge to the list (for visualization)
    std::shared_ptr<PoseEdge> edge = std::make_shared<PoseEdge>(p1,p2,loop_closure);
    edge->information = covariance.inverse();
    edges.push_back(edge);
    // Add edge to the optimizer.
    optimizer.addEdge(p1->id,p2->id,relativeTransform,covariance.inverse());
}
bool PoseGraph::optimize_graph() {
    optimizer.optimizeGraph();
    std::ofstream output("changes_file.txt");
    // Get updated poses after optmization
    for(int i = 0; i < poses.size(); ++i) {
        g2o::SE2 out_pose = optimizer.vertices[i]->estimate();
        Eigen::Vector3d out_vec(out_pose[0],out_pose[1],out_pose[2]);
        Eigen::Translation3d coords(out_vec(0),out_vec(1),0.0);
        Eigen::Quaterniond rotation;
        rotation = Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX()) *
                   Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX()) *
                   Eigen::AngleAxisd(out_vec(2),Eigen::Vector3d::UnitZ());
        Eigen::Affine3d transform(coords * rotation);
        Eigen::Matrix<double,4,4> old_pose = poses[i]->base_to_world.matrix();
        poses[i]->update_pose(transform,coords);
        output << "Pose difference: " << poses[i]->base_to_world.matrix() - old_pose << std::endl;
    }
    output.close();
    optimizer.saveGraph("optimized_graph.g2o");
}

std::shared_ptr<Pose> PoseGraph::add_vertex_previous(nav_msgs::Odometry odom, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr, pcl::PointCloud<pcl::PointNormal>::Ptr normal_ptr){

    Eigen::Translation3d coords(odom.pose.pose.position.x,
                                odom.pose.pose.position.y,
                                odom.pose.pose.position.z);
    Eigen::Quaterniond rotation(odom.pose.pose.orientation.w,
                                odom.pose.pose.orientation.x,
                                odom.pose.pose.orientation.y,
                                odom.pose.pose.orientation.z);
    Eigen::Affine3d world_transform(coords * rotation);

    Eigen::Matrix<double,3,3> covariance = Eigen::Matrix3d::Identity();
    /*covariance(0,0) = odom.pose.covariance[0];
    covariance(0,1) = odom.pose.covariance[1];
    covariance(0,2) = odom.pose.covariance[5];
    covariance(1,0) = odom.pose.covariance[6];
    covariance(1,1) = odom.pose.covariance[7];
    covariance(1,2) = odom.pose.covariance[11];
    covariance(2,0) = odom.pose.covariance[30];
    covariance(2,1) = odom.pose.covariance[31];
    covariance(2,2) = odom.pose.covariance[35];*/
    ROS_INFO_STREAM("Covariance matrix: " << covariance);
    // Make a new pose
    std::shared_ptr<Pose> pose = std::make_shared<Pose>(this->alloc_id++,
                                                        world_transform,
                                                        coords,
                                                        rotation,
                                                        odom.header.stamp,
                                                        cloud_ptr,
                                                        normal_ptr);
    int prev_edge_ind = optimizer.vertexIndex - 1;
    int curr_edge_ind = optimizer.vertexIndex;
    auto theta = rotation.toRotationMatrix().eulerAngles(0,1,2)(2);
    Eigen::Vector3d curr_pose(coords.vector()(0),coords.vector()(1),theta);
    optimizer.addVertex(curr_pose);

    auto prev_theta = poses[prev_edge_ind]->quat.toRotationMatrix().eulerAngles(0,1,2)(2);
    Eigen::Vector3d prev_pose(poses[prev_edge_ind]->coords.vector()(0),poses[prev_edge_ind]->coords.vector()(1),prev_theta);
    optimizer.addOdometryEdge(prev_edge_ind,curr_edge_ind,covariance.inverse());
    //optimizer.addEdge(prev_edge_ind,curr_edge_ind,(pose->coords.vector() - poses[prev_edge_ind]->coords.vector()),covariance.inverse());
    // Make a new edge between this pose and the prior pose.
    std::shared_ptr<PoseEdge> edge = std::make_shared<PoseEdge>(poses.back(),pose);

    // Set the information matrix for the edge.
    edge->information = covariance.inverse();

    // Add the pose and the edge to the list.
    poses.push_back(pose);
    edges.push_back(edge);

    return pose;
}

void PoseGraph::get_graph_markers(std::vector<visualization_msgs::Marker> &markers) {
    // Iterate the poses in the list
    visualization_msgs::Marker vertices;
    vertices.header.frame_id = "map";
    vertices.header.stamp = ros::Time();
    vertices.color.a = 1.0;
    vertices.color.r = 0.0;
    vertices.color.g = 0.0;
    vertices.color.b = 0.5;
    vertices.scale.x = 0.1;
    vertices.scale.y = 0.1;
    vertices.type = visualization_msgs::Marker::POINTS;
    vertices.id = 0;
    visualization_msgs::Marker pose_edges;
    pose_edges.header.frame_id = "map";
    pose_edges.header.stamp = ros::Time();
    pose_edges.color.a = 1.0;
    pose_edges.color.r = 0.0;
    pose_edges.color.g = 0.5;
    pose_edges.color.b = 0.0;
    pose_edges.scale.x = 0.05;
    pose_edges.header.stamp;
    pose_edges.type = visualization_msgs::Marker::LINE_LIST;
    pose_edges.id = 1;
    visualization_msgs::Marker loop_closure_edges;
    loop_closure_edges.header.frame_id = "map";
    loop_closure_edges.header.stamp = ros::Time();
    loop_closure_edges.color.a = 1.0;
    loop_closure_edges.color.r = 0.0;
    loop_closure_edges.color.g = 0.5;
    loop_closure_edges.color.b = 0.5;
    loop_closure_edges.scale.x = 0.05;
    loop_closure_edges.type = visualization_msgs::Marker::LINE_LIST;
    loop_closure_edges.id = 2;
    for (int i = 0; i < poses.size(); ++i){
        geometry_msgs::Point vertex;
        vertex.x = poses[i]->coords.x();
        vertex.y = poses[i]->coords.y();
        vertex.z = poses[i]->coords.z();
        vertices.points.push_back(vertex);
        //pose_edges.points.push_back(vertex);
    }
    for (int i = 0; i < edges.size(); ++i){
        geometry_msgs::Point from,to;
        from.x = edges[i]->v1->coords.x();
        from.y = edges[i]->v1->coords.y();
        from.z = edges[i]->v1->coords.z();

        to.x = edges[i]->v2->coords.x();
        to.y = edges[i]->v2->coords.y();
        to.z = edges[i]->v2->coords.z();

        if(edges[i]->loop_closure){
            loop_closure_edges.points.push_back(from);
            loop_closure_edges.points.push_back(to);
        }else{
            pose_edges.points.push_back(from);
            pose_edges.points.push_back(to);
        }
    }
    markers.push_back(vertices);
    markers.push_back(pose_edges);
    markers.push_back(loop_closure_edges);
}

void PoseGraph::get_prev_transform(Eigen::Affine3d &transform, nav_msgs::Odometry odom) {
    Eigen::Translation3d coords(odom.pose.pose.position.x,
                                odom.pose.pose.position.y,
                                odom.pose.pose.position.z);
    Eigen::Affine3d world_transform(coords  *
                                    Eigen::Quaterniond(odom.pose.pose.orientation.w,
                                                       odom.pose.pose.orientation.x,
                                                       odom.pose.pose.orientation.y,
                                                       odom.pose.pose.orientation.z));

    transform = world_transform.inverse() * poses.back()->base_to_world;
}

void PoseGraph::write_graph_to_file(){
    optimizer.saveGraph("graph.g2o");
}