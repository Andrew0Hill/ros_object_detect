//
// Created by ros on 3/19/18.
//

#ifndef OBJECT_DETECT_TYPE_H
#define OBJECT_DETECT_TYPE_H


class Type {
public:
    // ID number to uniquely identify this type.
    // This should be the ID number we get from the DetectedObje
    int id;
    Type(int oid){id = oid;}
    //static int alloc_id = 0;

};


#endif //OBJECT_DETECT_TYPE_H
