/**
 * File: DemoDetector.h
 * Date: November 2011
 * Author: Dorian Galvez-Lopez
 * Description: demo application of DLoopDetector
 * License: see the LICENSE.txt file
 */

#ifndef __DEMO_DETECTOR__
#define __DEMO_DETECTOR__

#include <iostream>
#include <vector>
#include <string>

// OpenCV
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

// DLoopDetector and DBoW2
#include "DBoW2.h"
#include "DLoopDetector.h"

//for time
#include <cctype>
#include <iostream>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include "dirent.h"
#include <unistd.h>
#include <string>
#include <ctime>
#include <cstdlib>
#include <chrono>


/// Generic class to create functors to extract features
template<class TDescriptor>
class FeatureExtractor
{
public:
  /**
   * Extracts features
   * @param im image
   * @param keys keypoints extracted
   * @param descriptors descriptors extracted
   */
  virtual void operator()(const cv::Mat &im, const std::vector<cv::Point2f> window_pts,
                          std::vector<cv::KeyPoint> &keys, std::vector<TDescriptor> &descriptors) const = 0;
};

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

/// @param TVocabulary vocabulary class (e.g: Surf64Vocabulary)
/// @param TDetector detector class (e.g: Surf64LoopDetector)
/// @param TDescriptor descriptor class (e.g: vector<float> for SURF)
template<class TVocabulary, class TDetector, class TDescriptor>
/// Class to run the demo 
class DemoDetector
{
public:

  /**
   * @param vocfile vocabulary file to load
   * @param imagedir directory to read images from
   * @param posefile pose file
   * @param width image width
   * @param height image height
   */
  DemoDetector(const std::string &vocfile, int width, int height);
    
  ~DemoDetector(){}


  /**
   * Runs the demo
   * @param name demo name
   * @param extractor functor to extract features
   */
    bool run(const std::string &name,
             const std::vector<cv::KeyPoint> &keys,
             const std::vector<TDescriptor> &descriptors,
             std::vector<cv::Point2f> &cur_pts,
             std::vector<cv::Point2f> &old_pts,
             int &old_index);
    
    void eraseIndex(std::vector<int> &erase_index);
    
  /*Data*/
  typename TDetector::Parameters params;
  TVocabulary voc;
  TDetector detector;
  int m_width;
  int m_height;

protected:

  /**
   * Reads the robot poses from a file
   * @param filename file
   * @param xs
   * @param ys
   */
  void readPoseFile(const char *filename, std::vector<double> &xs, 
    std::vector<double> &ys) const;

protected:

  std::string m_vocfile;
  //std::string m_imagedir;
  //std::string m_posefile;
};

// ---------------------------------------------------------------------------

template<class TVocabulary, class TDetector, class TDescriptor>
DemoDetector<TVocabulary, TDetector, TDescriptor>::DemoDetector
  (const std::string &vocfile, int width, int height)
  : m_vocfile(vocfile), m_width(width), m_height(height),
    params(height, width), voc(vocfile), detector(voc, params)
{
    //params.use_nss = true; // use normalized similarity score instead of raw score
    //params.alpha = 0.3; // nss threshold
    //params.k = 1; // a loop must be consistent with 1 previous matches
     // use direct index for geometrical checking
    //params.di_levels = 2; // use two direct index levels
    //printf("load vocfile %s finish\n", vocfile);
    
    printf("loop image size width: %d height: %d\n", params.image_cols,params.image_rows);
}

// ---------------------------------------------------------------------------
// ---------------------------------------------------------------------------
template<class TVocabulary, class TDetector, class TDescriptor>
void DemoDetector<TVocabulary, TDetector, TDescriptor>::eraseIndex
(std::vector<int> &erase_index)
{
    detector.eraseIndex(erase_index);
}
// ---------------------------------------------------------------------------

template<class TVocabulary, class TDetector, class TDescriptor>
bool DemoDetector<TVocabulary, TDetector, TDescriptor>::run
(const std::string &name, const std::vector<cv::KeyPoint> &keys,
 const std::vector<TDescriptor> &descriptors,
 std::vector<cv::Point2f> &cur_pts,
 std::vector<cv::Point2f> &old_pts,
 int &old_index)
{
    int count = 0;
    
    DLoopDetector::DetectionResult result;
    
    detector.detectLoop(keys, descriptors, result, cur_pts, old_pts);
    
    if(result.detection())
    {
        printf("- loop found with image %d !\n\n", result.match);
        ++count;
        old_index = result.match;
        return true;
    }
    else
    {
        std::cout << "- No loop: ";
        printf("- No loop: ");
        switch(result.status)
        {
            case DLoopDetector::CLOSE_MATCHES_ONLY:
                printf("All the images in the database are very recent\n");
                break;
                
            case DLoopDetector::NO_DB_RESULTS:
                printf("There are no matches against the database (few features in the image?)\n");
                break;
                
            case DLoopDetector::LOW_NSS_FACTOR:
                printf("Little overlap between this image and the previous one\n");
                break;
                
            case DLoopDetector::LOW_SCORES:
                printf("No match reaches the score threshold (alpha: %f)\n", params.alpha);
                break;
                
            case DLoopDetector::NO_GROUPS:
                printf("Not enough close matches to create groups. Best candidate: %d \n", result.match);
                break;
                
            case DLoopDetector::NO_TEMPORAL_CONSISTENCY:
                printf("No temporal consistency (k: %d ) Best candidate: %d \n",params.k, result.match);
                break;
                
            case DLoopDetector::NO_GEOMETRICAL_CONSISTENCY:
                printf("No geometrical consistency. Best candidate: %d \n", result.match);
                break;
                
            default:
                break;
        }
        printf("\n");
        return false;
    }
}

// ---------------------------------------------------------------------------

template<class TVocabulary, class TDetector, class TDescriptor>
void DemoDetector<TVocabulary, TDetector, TDescriptor>::readPoseFile
  (const char *filename, std::vector<double> &xs, std::vector<double> &ys)
  const
{
    xs.clear();
    ys.clear();
  
    std::fstream f(filename, std::ios::in);
  
    std::string s;
    double ts, x, y, t;
    while(!f.eof())
    {
        getline(f, s);
        if(!f.eof() && !s.empty())
        {
            sscanf(s.c_str(), "%lf, %lf, %lf, %lf", &ts, &x, &y, &t);
            xs.push_back(x);
            ys.push_back(y);
        }
    }
  
    f.close();
}

// ---------------------------------------------------------------------------

#endif

