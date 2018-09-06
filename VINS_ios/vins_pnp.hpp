//
//  vins_pnp.hpp
//  VINS_ios
//
//  Created by HKUST Aerial Robotics on 2016/10/22.
//  Copyright © 2017 HKUST Aerial Robotics. All rights reserved.
//

#ifndef vins_pnp_hpp
#define vins_pnp_hpp

#include <stdio.h>
#include "utility.hpp"
#include "projection_facor.hpp"
#include "pose_local_parameterization.hpp"
#include "global_param.hpp"
#include <ceres/ceres.h>
#include "imu_factor_pnp.h"
#include "perspective_factor.hpp"
#include <opencv2/core/eigen.hpp>
#include <map>
#include <vector>



struct VINS_RESULT {
    double header;
    Eigen::Vector3d Ba;
    Eigen::Vector3d Bg;
    Eigen::Vector3d P;
    Eigen::Matrix3d R;
    Eigen::Vector3d V;
};

struct IMG_MSG_LOCAL {
    int id;
    Eigen::Vector2d observation;
    Eigen::Vector3d position;
    int track_num;
};

class VinsPnP
{
public:
    
    typedef IMUFactorPnP IMUFactor_t;
    
    VinsPnP();
    int frame_count;
    
    Eigen::Matrix3d ric;
    Eigen::Vector3d tic;
    
    Eigen::Vector3d Ps[(PNP_SIZE + 1)];
    Eigen::Vector3d Vs[(PNP_SIZE + 1)];
    Eigen::Matrix3d Rs[(PNP_SIZE + 1)];
    Eigen::Vector3d Bas[(PNP_SIZE + 1)];
    Eigen::Vector3d Bgs[(PNP_SIZE + 1)];
    
    double para_Pose[PNP_SIZE + 1][SIZE_POSE];
    double para_Speed[PNP_SIZE + 1][SIZE_SPEED];
    double para_Bias[PNP_SIZE + 1][SIZE_BIAS];
    double para_Feature[NUM_OF_F][SIZE_FEATURE];
    double para_Ex_Pose[NUM_OF_CAM][SIZE_POSE];
    
    IntegrationBase *pre_integrations[(PNP_SIZE + 1)];
    std::vector<IMG_MSG_LOCAL> features[PNP_SIZE + 1];  //condition
    bool first_imu;
    Eigen::Vector3d acc_0, gyr_0;
    std::vector<double> dt_buf[(PNP_SIZE + 1)];
    std::vector<Eigen::Vector3d> linear_acceleration_buf[(PNP_SIZE + 1)];
    std::vector<Eigen::Vector3d> angular_velocity_buf[(PNP_SIZE + 1)];
    double Headers[(PNP_SIZE + 1)];
    bool find_solved[PNP_SIZE + 1];
    VINS_RESULT find_solved_vins[PNP_SIZE + 1];
    Eigen::Vector3d g;
    
    void solve_ceres();
    void old2new();
    void new2old();
    void clearState();
    void setIMUModel();
    void setExtrinsic();
    void setInit(VINS_RESULT vins_result);
    void slideWindow();
    void updateFeatures(std::vector<IMG_MSG_LOCAL> &feature_msg);
    void processImage(std::vector<IMG_MSG_LOCAL> &feature_msg, double header);
    void processIMU(double t, const Eigen::Vector3d &linear_acceleration, const Eigen::Vector3d &angular_velocity);
    void changeState();
};
#endif /* VINS_hpp */
