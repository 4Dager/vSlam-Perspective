/**
 * File: demo_brief.cpp
 * Date: November 2011
 * Author: Dorian Galvez-Lopez
 * Description: demo application of DLoopDetector
 * License: see the LICENSE.txt file
 */

// ----------------------------------------------------------------------------
#include "loop_closure.hpp"

using namespace DLoopDetector;
using namespace DBoW2;
using namespace DVision;

LoopClosure::LoopClosure(const char *_voc_file, int _image_w, int _image_h)
:demo(_voc_file,_image_w, _image_h), IMAGE_W(_image_w), IMAGE_H(_image_h) 
{
    printf("loop vocfile %s\n",_voc_file);
    printf(" loop closure init finish\n");
}


bool LoopClosure::startLoopClosure(std::vector<cv::KeyPoint> &keys, std::vector<BRIEF::bitset> &descriptors, std::vector<cv::Point2f> &cur_pts, std::vector<cv::Point2f> &old_pts, int &old_index)
{
    try
    {
        bool loop_succ = false;
        loop_succ = demo.run("BRIEF", keys, descriptors, cur_pts, old_pts, old_index);
        return loop_succ;
    }
    catch(const std::string &ex)
    {
        printf("Error loop: %s\n", ex.c_str());
        return false;
    }
}

void LoopClosure::eraseIndex(std::vector<int> &erase_index)
{
    demo.eraseIndex(erase_index);
}
