//
// Created by ros on 3/19/18.
//

#ifndef OBJECT_DETECT_CLASSMAP_H
#define OBJECT_DETECT_CLASSMAP_H
#include <string>
#include <unordered_map>

class ClassMap {
public:
    static const std::unordered_map<int, std::string> class_map;
};


#endif //OBJECT_DETECT_CLASSMAP_H
