//
// Created by ros on 3/21/18.
//

#include "ORB_Featurizer.h"

void ORB_Featurizer::featurizeImage(cv::Mat &image, std::vector<cv::KeyPoint> &keypoints, cv::Mat &descriptors) {
    // Just run the ORB detect and compute function.
    orb->detectAndCompute(image,cv::noArray(),keypoints,descriptors);
}
