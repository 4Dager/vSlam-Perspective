//
//  initial_aligment.hpp
//  PerspectiveSlam
//
//  Created by HKUST Aerial Robotics on 2016/12/26.
//  Copyright © 2017 HKUST Aerial Robotics. All rights reserved.
//

#ifndef initial_aligment_hpp
#define initial_aligment_hpp

#include <stdio.h>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include "imu_factor.h"
#include "utility.hpp"
#include <map>
#include "feature_manager.hpp"


class ImageFrame
{
public:
    ImageFrame(){};
    ImageFrame(const std::map<int, Eigen::Vector3d>& _points, double _t):points{_points},t{_t},is_key_frame{false}
    {
    };
    std::map<int, Eigen::Vector3d> points;
    double t;
    Eigen::Matrix3d R;
    Eigen::Vector3d T;
//    IntegrationBase *pre_integration;
    std::shared_ptr<IntegrationBase> pre_integration;
    bool is_key_frame;
};

bool VisualIMUAlignment(std::map<double, ImageFrame> &all_image_frame, Eigen::Vector3d* Bgs, Eigen::Vector3d &g, Eigen::VectorXd &x);

#endif /* initial_aligment_hpp */
