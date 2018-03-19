//
// Created by ros on 3/18/18.
//

#ifndef OBJECT_DETECT_DETECTEDOBJECT_H
#define OBJECT_DETECT_DETECTEDOBJECT_H

#include <unordered_map>
#include <vector>
#include <opencv2/core/core.hpp>


class DetectedObject {
private:
   /* static std::unordered_map<int,std::string> class_map =
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
            };*/
public:
    int ymin,ymax,xmin,xmax;
    float score;
    cv::Mat mask;
    void setCoords(int ym, int yx, int xm, int xx){
        ymin = ym;
        ymax = yx;
        xmin = xm;
        xmax = xx;
    }
};


#endif //OBJECT_DETECT_DETECTEDOBJECT_H
