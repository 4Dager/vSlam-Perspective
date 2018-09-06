#ifndef __LOOP_CLOSURE__
#define __LOOP_CLOSURE__

#include <iostream>
#include <vector>
#include <string>

// DLoopDetector and DBoW2
#include "DBoW2.h" // defines BriefVocabulary
#include "DLoopDetector.h" // defines BriefLoopDetector
#include "DVision.h" // Brief

// OpenCV
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "DemoDetector.h"
//#include "brief_extractor.h"


class LoopClosure
{
public:
	LoopClosure(const char *voc_file, int _image_w, int _image_h);

    bool startLoopClosure(std::vector<cv::KeyPoint> &keys, std::vector<DVision::BRIEF::bitset> &descriptors,
                          std::vector<cv::Point2f> &cur_pts,
                          std::vector<cv::Point2f> &old_pts,
                          int &old_index);
    void eraseIndex(std::vector<int> &erase_index);
	/* data */
    DemoDetector<BriefVocabulary, BriefLoopDetector, FBrief::TDescriptor> demo;
	int IMAGE_W;
	int IMAGE_H;
};

#endif
