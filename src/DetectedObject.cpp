//
// Created by ros on 3/18/18.
//

#include "DetectedObject.h"
const std::unordered_map<int, std::string> DetectedObject::class_map =
        {
            {1,"person"},
            {2,"bicycle"},
            {15,"bench"},
            {27,"backpack"},
            {31,"handbag"},
            {32,"tie"},
            {33,"suitcase"},
            {44,"bottle"},
            {46,"wine glass"},
            {47,"cup"},
            {49,"knife"},
            {62,"chair"},
            {63,"couch"},
            {64,"potted plant"},
            {65,"bed"},
            {67,"dining table"},
            {70,"toilet"},
            {72,"tv"},
            {73,"laptop"},
            {74,"mouse"},
            {75,"remote"},
            {76,"keyboard"},
            {77,"cell phone"},
            {81,"sink"},
            {82,"refrigerator"},
            {84,"book"},
            {85,"clock"}
        };