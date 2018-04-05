//
// Created by ros on 3/19/18.
//


#include "../include/object_detect/Type.h"

// Match a DetectedObject against all of the DetectedObjects in this type.
void Type::match(std::shared_ptr<DetectedObject> object) {
    static int num = 0;
    // If there are no objects in this type yet,
    // add this as the first object.
   /* if (obj_insts.empty()) {
        std::cout << "Empty object instance list." << std::endl;
        add_new_obj(object);
        return;
    }*/
    // Match the descriptors from this object into our descriptor memory.
    //ROS_INFO("Calling DescriptorMatcher.match()");

    if(!desc_matcher.match(object) && object->descriptors.rows){
        ROS_INFO("Adding new object instance!");
        std::shared_ptr<ObjectInstance> obj_inst = std::make_shared<ObjectInstance>(allocate_id(),object);
        object->set_parent(obj_inst);
        obj_inst->world_pos = object->world_pos;
        obj_inst->type = Type::id;
        obj_insts.push_back(obj_inst);
    }



    /*
    // Vector of Vectors for the KNN match.
    std::vector<std::vector<cv::DMatch>> matches;
    // Run KNN match with K=2.
    std::cout << "Object Descriptors: " << object->descriptors.size << std::endl;
    std::cout << "Current Descriptors: " << this->descriptors.size << std::endl;



    matcher.knnMatch(object->descriptors, this->descriptors, matches, 2);
    //std::cout << "Matches: " << matches.size() << std::endl;
    std::vector<cv::DMatch> filtered_matches;
    // Use NNDR test to filter out the bad matches.
    for (int i = 0; i < matches.size(); ++i) {
        if (matches[i].size() < 2) {
            std::cout << "There aren't 2 matches" << std::endl;
            continue;
        }
        //std::cout << matches[i][0].distance << "  " << matches[i][1].distance << std::endl;
        if (matches[i][0].distance < matches[i][1].distance * NN_THRESH) {
            filtered_matches.push_back(matches[i][0]);
        }
    }
    //std::cout << "Past filter loop" << std::endl;
    std::vector<std::pair<int, int>> query_train_indices;
    // Get indices for each
    for (int i = 0; i < filtered_matches.size(); ++i) {
        query_train_indices.push_back({filtered_matches[i].queryIdx, filtered_matches[i].trainIdx});
    }

    std::vector<int> match_imgs;

    for (int i = 0; i < query_train_indices.size(); ++i) {
        match_imgs.push_back(desc_map.at(query_train_indices[i].second));
    }


    std::cout << best_val << std::endl;
    std::cout << best_img_index << std::endl;
    if (best_img_index == -1) {
        //std::cout << "No Best Image." << std::endl;
        add_new_obj(object);
        return;
    }
    //ROS_INFO_STREAM("Best image match is: " << best_img_index << " with " << best_val << " occurrences.");

    std::shared_ptr<DetectedObject> best_img_ptr = this->objs[best_img_index];

    int best_total_feat_num = best_img_ptr->descriptors.rows;

    std::vector<std::vector<cv::DMatch>> s2_matches;

    matcher.knnMatch(object->descriptors, best_img_ptr->descriptors, s2_matches, 2);
    std::cout << "S2 Matches: " << s2_matches.size() << std::endl;
    std::vector<cv::DMatch> good_s2_matches;
    for (int i = 0; i < s2_matches.size(); ++i) {
        if (s2_matches[i].size() < 2) {
            //std::cout << "Can't perform NNDR, skipping" << std::endl;
            continue;
        }
        if (s2_matches[i][0].distance < s2_matches[i][1].distance * NN_THRESH) {
            good_s2_matches.push_back(s2_matches[i][0]);
        }
    }
    //std::cout << "Out of loop" << std::endl;
    if (good_s2_matches.size() == 0) {
        //std::cout << "No good s2 matches" << std::endl;
        add_new_obj(object);
        return;
    }

    std::vector<cv::Point2f> s2_query_kps;
    std::vector<cv::Point2f> s2_training_kps;
    std::vector<cv::KeyPoint> s2_query_drawpoints;
    std::vector<cv::KeyPoint> s2_training_drawpoints;
    //std::cout << "Matching Homography" << std::endl;
    for (int i = 0; i < good_s2_matches.size(); ++i) {
        s2_query_drawpoints.push_back(object->keypoints.at(good_s2_matches[i].queryIdx));
        s2_query_kps.push_back(object->keypoints.at(good_s2_matches[i].queryIdx).pt);
        s2_training_drawpoints.push_back(best_img_ptr->keypoints[good_s2_matches[i].trainIdx]);
        s2_training_kps.push_back(best_img_ptr->keypoints[good_s2_matches[i].trainIdx].pt);
    }

    //std::cout << "Finding Homography" << std::endl;
    cv::Mat mask;
    // Get the homography matrix for the transformation between the two sets of points.
    cv::findHomography(s2_query_kps, s2_training_kps, mask);

    //std::cout << "Finished finding Homography" << std::endl;
    if (cv::countNonZero(mask) > INLIERS_THRESH) {
#ifdef SHOW_MATCHES
        cv::Mat outImg;
        cv::drawMatches(object->image,object->keypoints,best_img_ptr->image,best_img_ptr->keypoints,good_s2_matches,outImg);
        cv::imshow("matches", outImg);
        cv::waitKey(3);
#endif
        //std::cout << "Enough Inliers to match" << std::endl;

        ROS_INFO_STREAM("Match found with object of type " << ClassMap::get_class(id) << " with ID: "
                                                           << best_img_ptr->parent->get_id());

        int desc_len = this->descriptors.rows;
        this->descriptors.push_back(object->descriptors);
        //cv::hconcat(this->descriptors,object->descriptors,this->descriptors);

        //std::cout << "Inserting KeyPoints" << std::endl;
        this->keypoints.insert(this->keypoints.end(), object->keypoints.begin(), object->keypoints.end());

        int img_list_ind = this->objs.size();

        this->objs.push_back(object);

        for (int i = desc_len; i < this->descriptors.rows; ++i) {
            desc_map[i] = img_list_ind;
        }

        best_img_ptr->parent->add_image(object);
        object->set_parent(best_img_ptr->parent);
    } else {
        //std::cout << "Not enough Inliers to match!" << std::endl;
        ROS_INFO_STREAM(
                "Not enough inliers for matched image " << best_img_index << " expected: " << INLIERS_THRESH << " got: "
                                                        << cv::countNonZero(mask));
        add_new_obj(object);
    }
*/

}

// Add a new ObjectInstance to this type.
void Type::add_new_obj(std::shared_ptr<DetectedObject> object) {
    // ????
    int desc_ind = this->descriptors.rows;
    unsigned long obj_inst_ind = this->obj_insts.size();
    unsigned long obj_ind = this->objs.size();

    // Allocate a new ID for this object
    int new_id = allocate_id();
    // Allocate a new ObjectInstance, and pass it a unique ID and DetectedObject pointer.
    std::shared_ptr<ObjectInstance> obj_inst = std::make_shared<ObjectInstance>(new_id, object);
    object->set_parent(obj_inst);
    // Add the ObjectInstance to the Type's list of ObjectInstances
    this->obj_insts.push_back(obj_inst);

    // Add the DetectedObject instance to the Type's list of DetectedObjects.
    this->objs.push_back(object);

    // Concatenate the list of descriptors from the DetectedObject to this list
    // for this ObjectInstance
    this->descriptors.push_back(object->descriptors);

    // Append all cv::KeyPoints from the DetectedObject to the type's list of KeyPoints
    this->keypoints.insert(this->keypoints.end(), object->keypoints.begin(), object->keypoints.end());

    // Update the map so that when we match an image, we can look up the value to see which DetectedObject
    // and ObjectInstance this descriptor came from.
    for (unsigned long i = desc_ind; i < descriptors.rows; ++i) {
        desc_map[i] = obj_ind;
    }

    ROS_INFO_STREAM("Added new Object of type: " << ClassMap::get_class(object->oclass) << " with ID: " << new_id);


}
