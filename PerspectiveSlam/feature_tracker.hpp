//
//  feature_tracker.hpp
//  PerspectiveSlam
//
//  Created by HKUST Aerial Robotics on 2016/10/18.
//  Copyright © 2017 HKUST Aerial Robotics. All rights reserved.
//

#ifndef feature_tracker_hpp
#define feature_tracker_hpp

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>
#include "global_param.hpp"
#include <string>
#include <list>
#include "utility.hpp"
#include <opencv2/core/eigen.hpp>
#include "vins_pnp.hpp"

#define MAX_CNT 70
#define MIN_DIST 30
#define COL 480
#define ROW 640
#define F_THRESHOLD 1.0
#define EQUALIZE 1

/*
 image frame
 --------> x:480
 |
 |
 |
 |
 |
 | y:640
 */
struct max_min_pts{
    cv::Point2f min;
    cv::Point2f max;
};

struct IMU_MSG_LOCAL {
    double header;
    Eigen::Vector3d acc;
    Eigen::Vector3d gyr;
};

class FeatureTracker
{
public:
    FeatureTracker();
    bool solveVinsPnP(double header, Eigen::Vector3d &P, Eigen::Matrix3d &R, bool vins_normal);
    void readImage(const cv::Mat &_img, cv::Mat &result, int _frame_cnt, std::vector<cv::Point2f> &good_pts, std::vector<double> &track_len, double header, Eigen::Vector3d &P, Eigen::Matrix3d &R, bool vins_normal);
    void setMask();
    void rejectWithF();
    void addPoints();
    bool updateID(unsigned int i);
    
    /*
     varialbles
     */
    int frame_cnt;
    cv::Mat mask;
    cv::Mat cur_img, pre_img, forw_img;
    
    std::vector<cv::Point2f> n_pts,cur_pts,pre_pts,forw_pts;
    
    std::vector<int> ids,track_cnt;
    std::vector<max_min_pts> parallax_cnt;
    static int n_id;
    int img_cnt;
    double current_time;

    VinsPnP vins_pnp;

    
    /*
     interface
     */
    std::map<int, Eigen::Vector3d> image_msg;
    bool update_finished;
    std::list<IMG_MSG_LOCAL> solved_features;
    VINS_RESULT solved_vins;
    std::vector<IMU_MSG_LOCAL> imu_msgs;
    
};
#endif /* feature_tracker_hpp */
