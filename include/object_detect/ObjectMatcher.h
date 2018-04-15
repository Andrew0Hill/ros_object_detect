//
// Created by ros on 4/13/18.
//

#ifndef OBJECT_DETECT_OBJECTMATCHER_H
#define OBJECT_DETECT_OBJECTMATCHER_H

#define NN_THRESH 0.75
#define FILT_FEATS_SIZE 25
//#define DRAW_MATCHED_IMAGES
#include "DetectedObject.h"
#include <ros/ros.h>
#include <rosconsole/macros_generated.h>
class ObjectMatcher {
public:
    ObjectMatcher(){
        matcher = cv::FlannBasedMatcher(new cv::flann::LshIndexParams(20,15,2));
    }
    int match_object(std::shared_ptr<DetectedObject> target, std::shared_ptr<ObjectInstance> cand,std::shared_ptr<DetectedObject>& match);
private:
    cv::FlannBasedMatcher matcher;
};


#endif //OBJECT_DETECT_OBJECTMATCHER_H
