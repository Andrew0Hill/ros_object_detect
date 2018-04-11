//
// Created by ros on 3/19/18.
//

#include "../include/object_detect/ClassMap.h"
const std::unordered_map<int, std::string> ClassMap::class_map =
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
              //  {82,"refrigerator"},
                {84,"book"},
                {85,"clock"}
        };
const std::unordered_map<int,int> ClassMap::index_map = {
        {27,0},
        {65,1},
        {15,2},
        {2,3},
        {84,4},
        {44,5},
        {77,6},
        {62,7},
        {85,8},
        {63,9},
        {47,10},
        {67,11},
        {31,12},
        {76,13},
        {49,14},
        {73,15},
        {74,16},
        {1,17},
        {64,18},
        {82,19},
        {75,20},
        {81,21},
        {33,22},
        {32,23},
        {70,24},
        {72,25},
        {46,26}
};