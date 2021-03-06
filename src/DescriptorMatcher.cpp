//
// Created by ros on 3/26/18.
//


#include <ros/assert.h>
#include <opencv-3.3.1/opencv2/highgui.hpp>
#include "../include/object_detect/DescriptorMatcher.h"

/*
 * match()
 *
 * Matches a low-level set of descriptors (a cv::Mat from a featurizer) into a set of high-level
 * Descriptor objects. With this system, we can keep track of how many time a descriptor appears in the tree, and reduce
 * the number of descriptors we need to store.
 */
int DescriptorMatcher::match(std::shared_ptr<DetectedObject> obj, std::shared_ptr<DetectedObject>& match) {

    if(this->descriptor_list.cols == 0) {
        ROS_INFO("No Objects in List!, adding all descriptors");
        create_descriptors(obj,obj->descriptors,obj->keypoints);
        return false;
    }

/*    for(auto it = descriptor_map.begin(); it != descriptor_map.end(); ++it){
        std::cout << it->first << "  " << it->second << std::endl;
    }*/
    // Match the descriptors from the list to the descriptors we were given.
    ROS_DEBUG("Matching using KNN matcher");
    ROS_INFO_STREAM("object descriptor size: " << obj->descriptors.rows << " memory descriptor size: " << descriptor_list.rows << " memory map size: " << descriptor_map.size());

    // Vector of Vector of DMatch to hold the two best matches between descriptors.
    std::vector<std::vector<cv::DMatch>> matches;
    matcher.knnMatch(obj->descriptors,descriptor_list,matches,2);

    std::vector<int> filtered_train_indices;
    std::vector<int> filtered_query_indices;
    std::vector<int> rejected_query_indices;
    // Keep track of the matched keypoints from the query image (new image).
    std::vector<cv::Point2f> filtered_query_points;
    std::vector<cv::KeyPoint> filtered_query_keypoints;
    std::vector<cv::KeyPoint> rejected_query_keypoints;
    // Run NNDR on the pairs of matches so that we can tell if we should keep them or not.
    // Output is a filtered set of indices into the training set (the descriptors in descriptor_list)
    ROS_INFO("Filtering with NNDR");
    for (int i = 0; i < matches.size(); ++i){
        if(matches[i].size() < 2){
            ROS_INFO("Two matches could not be found, skipping.");
            continue;
        }

        //ROS_INFO_STREAM(matches[i][0].distance << "  " << matches[i][1].distance);
        if(matches[i][0].distance < matches[i][1].distance * NN_THRESH){
            filtered_train_indices.push_back(matches[i][0].trainIdx);
            filtered_query_indices.push_back(matches[i][0].queryIdx);
            filtered_query_points.push_back(obj->keypoints[matches[i][0].queryIdx].pt);
            filtered_query_keypoints.push_back(obj->keypoints[matches[i][0].queryIdx]);
        }else{
            rejected_query_indices.push_back(matches[i][0].queryIdx);
            rejected_query_keypoints.push_back(obj->keypoints[matches[i][0].queryIdx]);
        }
    }
    ROS_INFO_STREAM(filtered_query_indices.size() << " descriptors out of" << obj->descriptors.rows << " pass filtering.");
    int num_of_inliers = 0;
    std::shared_ptr<DetectedObject> best_obj;
    if(!filtered_query_indices.empty()) {
        // Create descriptor vector.
        std::vector<std::shared_ptr<Descriptor>> matched_descriptors;
        // Pass descriptor vector to the get_descriptors function.
        get_descriptors(filtered_train_indices, matched_descriptors);
        // Get the best image match for this set of descriptors.
        std::vector<std::shared_ptr<Descriptor>> filtered_descriptors;


        std::vector<int> filtered_indices;

        // TODO: Replace this with weighting scheme (TF-IDF?) so that common features don't throw off the match.
        best_obj = get_best_img_match(matched_descriptors,filtered_descriptors,filtered_indices);

        float update_val = 1 - ((float)filtered_descriptors.size() / best_obj->descriptors.rows);
        std::cout << "Update Rate: " << update_val << std::endl;

        // Vector of matched keypoint for each Descriptor in matched_descriptors.
        std::vector<cv::KeyPoint> matched_keypoints;
        // Get the image KeyPoints for each Descriptor, and put them in matched_keypoints.
        get_img_keypoints(best_obj, filtered_descriptors, matched_keypoints);

        std::vector<cv::KeyPoint> matched_query_keypoints;
        for (int i = 0; i < filtered_indices.size(); ++i){
            matched_query_keypoints.push_back(filtered_query_keypoints[filtered_indices[i]]);
        }

        num_of_inliers = get_inliers(matched_keypoints, matched_query_keypoints);

        if(num_of_inliers > INLIERS_THRESH){
        #ifdef DRAW_MATCHED_IMAGES
            std::vector<cv::DMatch> matches;
            for(int i = 0; i < matched_keypoints.size(); ++i){
                cv::DMatch temp;
                temp.trainIdx = i;
                temp.queryIdx = i;
                matches.push_back(temp);

            }
            cv::Mat out_img;
            cv::drawMatches(obj->image,matched_query_keypoints,best_obj->image,matched_keypoints,matches,out_img);
            cv::imshow("images", out_img);
            cv::waitKey(3);
        #endif
        }
        ROS_INFO_STREAM("Updated " << matched_descriptors.size() << " existing descriptors");
        add_reference(matched_descriptors, obj, filtered_query_keypoints);
        ROS_INFO("Finished Adding References.");

    }else{
        ROS_ERROR_STREAM("No Descriptive features for this object! Skipping!");
        match = NULL;
        return -1;
    }
    create_descriptors(obj,rejected_query_indices,obj->descriptors,rejected_query_keypoints);
    ROS_INFO_STREAM("Added " << rejected_query_indices.size() << " new descriptors");
    ROS_INFO("Finished Creating Descriptors.");


    if(num_of_inliers > 15){
        ROS_INFO("Enough Inliers to match!");
        obj->set_parent(best_obj->parent);
        best_obj->parent->add_image(obj);
        match = best_obj;
        return 1;
    }

    ROS_INFO("Not enough inliers to match!");
    match = NULL;
    return 0;

}

std::shared_ptr<Descriptor> DescriptorMatcher::get_descriptor(int index) {
    // If we can find a Descriptor at this index, return the pointer.
    if (descriptor_map.find(index) != descriptor_map.end()){
        ROS_INFO("IN get_descriptor()");
        return descriptor_map.at(index);
    }
    return nullptr;
}

void DescriptorMatcher::get_descriptors(std::vector<int> &indices, std::vector<std::shared_ptr<Descriptor>> &out_vec) {
    auto ptr = descriptor_map.at(indices[0]);
    for(int i = 0; i < indices.size(); ++i){
        //std::cout << indices[i] << std::endl;
        out_vec.push_back(descriptor_map.at(indices[i]));
    }
}

std::shared_ptr<DetectedObject>
DescriptorMatcher::get_best_img_match(std::vector<std::shared_ptr<Descriptor>> &descriptors,
                                      std::vector<std::shared_ptr<Descriptor>> &filtered_descriptors,
                                      std::vector<int> &filtered_indices) {
    std::unordered_map<std::shared_ptr<DetectedObject>, std::pair<int, std::vector<int>>> match_occurrences;
    // Iterate each descriptor
    for(int i = 0; i < descriptors.size(); ++i){
        // Iterate the image_set (vector of DetectedObject pointers) for each Descriptor, and count the occurrences
        for (auto set_it = descriptors[i]->image_set.begin(); set_it != descriptors[i]->image_set.end(); ++set_it){
            // If this DetectedObject has been seen before:
            if(match_occurrences.find(*set_it) != match_occurrences.end()){
                // Increment the count by one.
                match_occurrences[*set_it].first += 1;
            }else{
                // Add a new key value of 1 for the first count of this DetectedObject.
                match_occurrences[*set_it].first = 1;
            }
            match_occurrences[*set_it].second.push_back(i);
        }
    }
    // Pointer for the best DetectedObject.
    std::shared_ptr<DetectedObject> best_obj;
    // Value for the current best count.
    int best_count = 0;
    // Iterate the map to find the "best" (highest # of occurrences) DetectedObject
    for(auto it = match_occurrences.begin(); it != match_occurrences.end(); ++it){
        // If the count for this DetectedObject is larger than the current max
        if (it->second.first > best_count){
            // Update the current max and best_obj pointer.
            best_count = it->second.first;
            best_obj = it->first;
        }
    }
    for(int i = 0; i < match_occurrences[best_obj].second.size(); ++i){
        filtered_descriptors.push_back(descriptors[match_occurrences[best_obj].second[i]]);
        filtered_indices.push_back(match_occurrences[best_obj].second[i]);
    }

    return best_obj;
}

void DescriptorMatcher::get_img_keypoints(std::shared_ptr<DetectedObject> obj,
                                          std::vector<std::shared_ptr<Descriptor>> descriptors,
                                          std::vector<cv::KeyPoint> &keypoints) {
    for(int i = 0; i < descriptors.size(); ++i){
        // For each Descriptor, get the KeyPoint of that descriptor that corresponds to
        // the DetectedObject pointer we have.
        //std::cout << descriptors[i] << std::endl;
        keypoints.push_back(*(descriptors[i]->getKeyPoint(obj)));
    }
}
// Perform geometric matching between the training keypoints and the query keypoints.
// TODO: We will want to return the matrix computed from findHomography or findFundamentalMat in the future.
int DescriptorMatcher::get_inliers(std::vector<cv::KeyPoint> &train_kps, std::vector<cv::KeyPoint> &query_kps) {
    std::vector<cv::Point2f> train_pts;
    std::vector<cv::Point2f> query_pts;
    if(train_kps.size() != query_kps.size()){
        ROS_ERROR("Training and Query KeyPoints sets are not the same size!");
    }else if(train_kps.size() == 0){
        ROS_ERROR("train_kps and query_kps are empty!");
    }

    for(int i = 0; i < train_kps.size(); ++i){
        train_pts.push_back(train_kps[i].pt);
        query_pts.push_back(query_kps[i].pt);
    }
    // findEssentialMatrix
    // TODO: Only want to compute relative pose if there are enough inliers in the result of findEssentialMatrix().
    // TODO: Maybe return the Essential matrix as a parameter to this function so that we can recoverPose() outside this scope.
    //
    //cv::Mat mask = cv::findHomography(train_pts,query_pts,CV_RANSAC);
    cv::Mat mask;
    cv::findHomography(train_pts,query_pts,CV_RANSAC,3,mask);
    //cv::findFundamentalMat(train_pts,query_pts,CV_FM_RANSAC,3.0,0.99,mask);
    return cv::countNonZero(mask);
}

void DescriptorMatcher::add_reference(std::vector<std::shared_ptr<Descriptor>> update_descs,
                                      std::shared_ptr<DetectedObject> new_obj,
                                      std::vector<cv::KeyPoint> new_kps) {
    if(update_descs.size() != new_kps.size())
        ROS_ERROR_STREAM("Update Descriptor list and KeyPoint list are not the same size! update_descs: " << update_descs.size() << " new_kps: " << new_kps.size());
    for(int i = 0; i < update_descs.size(); ++i){
        update_descs[i]->addParent(new_obj,std::make_shared<cv::KeyPoint>(new_kps[i]));
    }
}

void DescriptorMatcher::create_descriptors(std::shared_ptr<DetectedObject> new_obj,
                                           std::vector<int> &rejected_indices,
                                           cv::Mat &descriptors,
                                           std::vector<cv::KeyPoint> &rejected_keypoints) {

    for(int i = 0; i < rejected_indices.size(); ++i){
        // Create a new Descriptor object, pass it the Descriptor and KeyPoint it refers to.
        std::shared_ptr<Descriptor> new_desc = std::make_shared<Descriptor>();
        new_desc->addParent(new_obj,std::make_shared<cv::KeyPoint>(rejected_keypoints[i]));
        // Update the Descriptor list by adding the raw descriptor to the descriptor list,
        // and a mapping between the index and the Descriptor object to the descriptor_map.
        descriptor_list.push_back(descriptors.row(rejected_indices[i]));
        //ROS_INFO_STREAM("Inserted index: " << descriptor_list.rows-1);
        descriptor_map[descriptor_list.rows-1] = new_desc;
    }
}

void DescriptorMatcher::create_descriptors(std::shared_ptr<DetectedObject> new_obj,
                                           cv::Mat &descriptors,
                                           std::vector<cv::KeyPoint> &keypoints) {
    if (descriptors.rows != keypoints.size())
        ROS_ERROR("Descriptor list size is not equal to KeyPoint list size!");
    for(int i = 0; i < keypoints.size(); ++i){
        std::shared_ptr<Descriptor> new_desc = std::make_shared<Descriptor>();
        new_desc->addParent(new_obj,std::make_shared<cv::KeyPoint>(keypoints[i]));

        descriptor_list.push_back(descriptors.row(i));
        //ROS_INFO_STREAM("Inserted index: " << descriptor_list.rows-1);
        descriptor_map[descriptor_list.rows-1] = new_desc;
    }
}



