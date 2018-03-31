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
    static bool class_exists(int c_num){ return class_map.find(c_num) != class_map.end(); }
    static std::string get_class(int c_num){ return class_map.at(c_num); }
    static const std::unordered_map<int,int> index_map;
    static int get_class_index(int ind) {return index_map.at(ind);}
};


#endif //OBJECT_DETECT_CLASSMAP_H
