//
// Created by ros on 3/21/18.
//

#ifndef OBJECT_DETECT_ORB_FEATURIZER_H
#define OBJECT_DETECT_ORB_FEATURIZER_H


#include "Featurizer.h"
#include <opencv2/features2d/features2d.hpp>
class ORB_Featurizer : Featurizer{

public:
    void featurizeImage(cv::Mat &image, std::vector<cv::KeyPoint> &keypoints, cv::Mat &descriptors) override;
    ORB_Featurizer(){
        orb = cv::ORB::create();
    }
private:
    cv::Ptr<cv::ORB> orb;
};


#endif //OBJECT_DETECT_ORB_FEATURIZER_H
