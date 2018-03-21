//
// Created by ros on 3/19/18.
//

#ifndef OBJECT_DETECT_MEMORY_H
#define OBJECT_DETECT_MEMORY_H
#define MIN_FEAT_NUM = 10

#include <unordered_map>
#include "Type.h"
#include <opencv2/xfeatures2d.hpp>
#include <memory>

class Memory {
public:
    std::unordered_map<int, std::shared_ptr<Type>> type_dict;
    void add_type(int tid){ type_dict.insert({tid, std::make_shared<Type>(tid)}); }
private:
    cv::FlannBasedMatcher matcher;

};


#endif //OBJECT_DETECT_MEMORY_H
