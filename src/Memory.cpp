//
// Created by ros on 3/19/18.
//

#include "../include/object_detect/Memory.h"

void Memory::match(std::shared_ptr<DetectedObject> object) {
    // If this type has not been seen before in the dicionary:
    if (type_dict.find(object->oclass) == type_dict.end()){
        std::cout << "Added new type: " << ClassMap::class_map.at(object->oclass) << std::endl;
        add_type(object->oclass);
    }

    type_dict.at(object->oclass)->match(object);
}