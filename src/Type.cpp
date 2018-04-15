//
// Created by ros on 3/19/18.
//


#include "../include/object_detect/Type.h"

// Match a DetectedObject against all of the DetectedObjects in this type.
void Type::match(std::shared_ptr<DetectedObject> object, std::shared_ptr<DetectedObject>& match) {
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
#ifdef FULL_MATCHING
    auto it = obj_insts.begin();
    for (it; it != obj_insts.end(); ++it) {
        if (obj_matcher.match_object(object,*it,match) == 1) {
            return;
        }
    }

    ROS_INFO("Adding new object instance!");
    std::shared_ptr<ObjectInstance> obj_inst = std::make_shared<ObjectInstance>(allocate_id(), object);
    object->set_parent(obj_inst);
    obj_inst->world_pos = object->world_pos;
    obj_inst->type = Type::id;
    obj_insts.push_back(obj_inst);
#else
    if(desc_matcher.match(object,match) == 0 && object->descriptors.rows){
        ROS_INFO("Adding new object instance!");
        std::shared_ptr<ObjectInstance> obj_inst = std::make_shared<ObjectInstance>(allocate_id(),object);
        object->set_parent(obj_inst);
        obj_inst->world_pos = object->world_pos;
        obj_inst->type = Type::id;
        obj_insts.push_back(obj_inst);
    }
#endif
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
