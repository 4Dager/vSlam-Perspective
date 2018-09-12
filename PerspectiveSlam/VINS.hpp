//
//  VINS.hpp
//  PerspectiveSlam
//
//  Created by HKUST Aerial Robotics on 2016/10/22.
//  Copyright © 2017 HKUST Aerial Robotics. All rights reserved.
//

#ifndef VINS_hpp
#define VINS_hpp

#include <stdio.h>
#include "feature_manager.hpp"
#include "utility.hpp"
#include "projection_facor.hpp"
#include "pose_local_parameterization.hpp"
#include "global_param.hpp"
#include <ceres/ceres.h>
#include "marginalization_factor.hpp"
#include "imu_factor.h"
#include "draw_result.hpp"
#include <opencv2/core/eigen.hpp>
#include "inital_sfm.hpp"
#include "initial_aligment.hpp"
#include "motion_estimator.hpp"

struct RetriveData
{
    /* data */
    int old_index;
    int cur_index;
    double header;
    Eigen::Vector3d P_old;
    Eigen::Quaterniond Q_old;
    Eigen::Vector3d P_cur;
    Eigen::Quaterniond Q_cur;
    std::vector<cv::Point2f> measurements;
    std::vector<int> features_ids;
    bool use;
    Eigen::Vector3d relative_t;
    Eigen::Quaterniond relative_q;
    double relative_yaw;
    double loop_pose[7];
};

class VINS
{
public:
    
    typedef IMUFactor IMUFactor_t;
    
    VINS();
    
    enum SolverFlag
    {
        INITIAL,
        NON_LINEAR
    };
    enum MarginalizationFlag
    {
        MARGIN_OLD = 0,
        MARGIN_SECOND_NEW = 1
    };
    
    FeatureManager f_manager;
    MotionEstimator m_estimator;
    int frame_count;
    
    Eigen::Matrix3d ric;
    Eigen::Vector3d tic;
    MarginalizationFlag  marginalization_flag;
    Eigen::Vector3d Ps[10 * (WINDOW_SIZE + 1)];
    Eigen::Vector3d Vs[10 * (WINDOW_SIZE + 1)];
    Eigen::Matrix3d Rs[10 * (WINDOW_SIZE + 1)];
    Eigen::Vector3d Bas[10 * (WINDOW_SIZE + 1)];
    Eigen::Vector3d Bgs[10 * (WINDOW_SIZE + 1)];
    
    double para_Pose[WINDOW_SIZE + 1][SIZE_POSE];
    double para_SpeedBias[WINDOW_SIZE + 1][SIZE_SPEEDBIAS];
    double para_Feature[NUM_OF_F][SIZE_FEATURE];
    double para_Ex_Pose[NUM_OF_CAM][SIZE_POSE];
    
    //for loop closure
    RetriveData retrive_pose_data, front_pose;
    bool loop_enable;
    std::vector<Eigen::Vector3f> correct_point_cloud;
    Eigen::Vector3f correct_Ps[WINDOW_SIZE];
    Eigen::Matrix3f correct_Rs[WINDOW_SIZE];
    Eigen::Vector3d t_drift;
    Eigen::Matrix3d r_drift;
    
    MarginalizationInfo* last_marginalization_info;
    std::vector<double *> last_marginalization_parameter_blocks;
    std::vector<Eigen::Vector3f> point_cloud;
    
    int feature_num;
    std::shared_ptr<IntegrationBase> pre_integrations[10 * (WINDOW_SIZE + 1)];
//    IntegrationBase *pre_integrations[10 * (WINDOW_SIZE + 1)];
    bool first_imu;
    Eigen::Vector3d acc_0, gyr_0;
    std::vector<double> dt_buf[10 * (WINDOW_SIZE + 1)];
    std::vector<Eigen::Vector3d> linear_acceleration_buf[10 * (WINDOW_SIZE + 1)];
    std::vector<Eigen::Vector3d> angular_velocity_buf[10 * (WINDOW_SIZE + 1)];
    Eigen::Matrix<double, 7, 1> IMU_linear[10 * (WINDOW_SIZE + 1)];
    Eigen::Matrix3d IMU_angular[10 * (WINDOW_SIZE + 1)];
    double Headers[10 * (WINDOW_SIZE + 1)];
    Eigen::Vector3d g;
    
    std::vector<Eigen::Vector3d> init_poses;
    double initial_timestamp;
    Eigen::Vector3d init_P;
    Eigen::Vector3d init_V;
    Eigen::Matrix3d init_R;
    
    SolverFlag solver_flag;
    Eigen::Matrix3d Rc[10 * (WINDOW_SIZE + 1)];
    
    //for initialization
    std::map<double, ImageFrame> all_image_frame;
    std::shared_ptr<IntegrationBase> tmp_pre_integration;
    //    IntegrationBase *tmp_pre_integration;
    Eigen::Matrix3d back_R0;
    Eigen::Vector3d back_P0;
    //for falure detection
    bool failure_hand;
    bool failure_occur;
    Eigen::Matrix3d last_R, last_R_old;
    Eigen::Vector3d last_P, last_P_old;
    
    //for visulization
    DrawResult drawresult;
    cv::Mat image_show;
    cv::Mat imageAI;
    enum InitStatus
    {
        FAIL_IMU,
        FAIL_PARALLAX,
        FAIL_RELATIVE,
        FAIL_SFM,
        FAIL_PNP,
        FAIL_ALIGN,
        FAIL_CHECK,
        SUCC
    };
    InitStatus init_status;
    int parallax_num_view;
    int fail_times;
    int initProgress;
    double final_cost;
    double visual_cost;
    int visual_factor_num;
    
    void solve_ceres(int buf_num);
    void solveCalibration();
    void old2new();
    void new2old();
    void clearState();
    void setIMUModel();
    void setExtrinsic();
    void slideWindow();
    void slideWindowNew();
    void slideWindowOld();
    void processImage(std::map<int, Eigen::Vector3d> &image_msg, double header, int buf_num);
    void processIMU(double t, const Eigen::Vector3d &linear_acceleration, const Eigen::Vector3d &angular_velocity);
    void changeState();
    bool solveInitial();
    bool relativePose(int camera_id, Eigen::Matrix3d &relative_R, Eigen::Vector3d &relative_T, int &l);
    bool visualInitialAlign();
    bool failureDetection();
    void failureRecover();
    void reInit();
    void update_loop_correction();
};
#endif /* VINS_hpp */
