//
// Created by ros on 4/13/18.
//

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv-3.3.1/opencv2/highgui.hpp>
#include "ObjectMatcher.h"

int ObjectMatcher::match_object(std::shared_ptr<DetectedObject> target, std::shared_ptr<ObjectInstance> cand, std::shared_ptr<DetectedObject>& match) {
    ROS_WARN_STREAM("Started Matching Object.");
    // Iterate through each image in this ObjectInstance's set of images, and match it.
    auto it = cand->images.begin();
    // Initial set of matches.
    std::vector<std::vector<cv::DMatch>> matches;
    // Filtered set of matches after NNDR.
    std::vector<cv::DMatch> filtered_matches;
    // Vectors for query and train keypoints.
    std::vector<cv::Point2f> train_pts;
    std::vector<cv::Point2f> query_pts;
    ROS_WARN_STREAM("Entering Loop");
    for(it; it != cand->images.end(); ++it){
        matches.clear();
        filtered_matches.clear();
        train_pts.clear();
        query_pts.clear();
        // KNN match with K = 2.
        ROS_WARN_STREAM("Dereferencing Iterator");
        matcher.knnMatch(target->descriptors,(*it)->descriptors,matches,2);
        // Use NNDR to filter out bad matches.
        //ROS_WARN_STREAM("Finished Dereferencing Iterator");
        for (int i = 0; i < matches.size(); ++i){
            if(matches[i].size() == 2 && matches[i][0].distance < matches[i][1].distance * NN_THRESH){
                filtered_matches.push_back(matches[i][0]);
            }
        }
        ROS_WARN_STREAM("Getting keypoints.");
        for(int j = 0; j < filtered_matches.size(); ++j){
            query_pts.push_back(target->keypoints[filtered_matches[j].queryIdx].pt);
            train_pts.push_back((*it)->keypoints[filtered_matches[j].trainIdx].pt);
        }
        if(filtered_matches.size() < FILT_FEATS_SIZE){
            ROS_ERROR_STREAM("Not enough features passed filtering!");
            continue;
        }
        cv::Mat mask;
        cv::findHomography(query_pts,train_pts, CV_RANSAC, 3, mask);

        if(cv::countNonZero(mask) > 0.5*query_pts.size()){
            ROS_INFO_STREAM(cv::countNonZero(mask) << " inliers out of " << train_pts.size() << " possible points." );
            #ifdef DRAW_MATCHED_IMAGES
            cv::Mat out_img;
            cv::drawMatches(target->image,target->keypoints,(*it)->image,(*it)->keypoints,filtered_matches,out_img);
            cv::imshow("images", out_img);
            cv::waitKey(3);
            #endif
            match = *it;
            target->set_parent(cand);
            cand->add_image(target);
            ROS_WARN_STREAM("Finished Matching: Found Match.");
            return 1;
        }

    }
    ROS_WARN_STREAM("Finished Matching: New Instance.");
    return 0;
}
