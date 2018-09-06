//
//  keyfame_database.hpp
//  VINS_ios
//
//  Created by HKUST Aerial Robotics on 2017/5/2.
//  Copyright © 2017 HKUST Aerial Robotics. All rights reserved.
//

#ifndef keyfame_database_hpp
#define keyfame_database_hpp

#include <stdio.h>
#include <vector>
#include <list>
#include <assert.h>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include "keyframe.hpp"
#include "utility.hpp"

//for save keyframe result
struct KEYFRAME_DATA
{
    double header;
    Eigen::Vector3d translation;
    Eigen::Quaterniond rotation;
};

class KeyFrameDatabase
{
public:
    KeyFrameDatabase();
    void add(KeyFrame* pKF);
    void resample(std::vector<int> &erase_index);
    void erase(KeyFrame* pKF);
    int size();
    void optimize4DoFLoopPoseGraph(int cur_index, Eigen::Vector3d &loop_correct_t, Eigen::Matrix3d &loop_correct_r);
    KeyFrame* getKeyframe(int index);
    KeyFrame* getLastKeyframe();
    KeyFrame* getLastKeyframe(int last_index);
    KeyFrame* getLastUncheckKeyframe();
    void updateVisualization();
    void addLoop(int loop_index);
    
    std::vector<Eigen::Vector3f> refine_path;
    std::vector<KEYFRAME_DATA> all_keyframes;
    std::vector<int> segment_indexs;
    int max_seg_index, cur_seg_index;
    
private:
    list<KeyFrame*> keyFrameList;
    std::mutex mMutexkeyFrameList;
    int earliest_loop_index;
    Eigen::Vector3d t_drift;
    double yaw_drift;
    Eigen::Matrix3d r_drift;
    int max_frame_num;
    double total_length;
    Eigen::Vector3d last_P;
};

template <typename T>
T NormalizeAngle(const T& angle_degrees) {
    T two_pi(2.0 * 180);
    
    if (angle_degrees > T(0))
        return angle_degrees -
        two_pi * ceres::floor((angle_degrees + T(180)) / two_pi);
    else
        return angle_degrees +
        two_pi * ceres::floor((-angle_degrees + T(180)) / two_pi);
};

class AngleLocalParameterization {
public:
    
    template <typename T>
    bool operator()(const T* theta_radians, const T* delta_theta_radians,
                    T* theta_radians_plus_delta) const {
        *theta_radians_plus_delta =
        NormalizeAngle(*theta_radians + *delta_theta_radians);
        
        return true;
    }
    
    static ceres::LocalParameterization* Create() {
        return (new ceres::AutoDiffLocalParameterization<AngleLocalParameterization,
                1, 1>);
    }
};

template <typename T> inline
void QuaternionInverse(const T q[4], T q_inverse[4])
{
    q_inverse[0] = q[0];
    q_inverse[1] = -q[1];
    q_inverse[2] = -q[2];
    q_inverse[3] = -q[3];
};

struct RelativeTError
{
    RelativeTError(double t_x, double t_y, double t_z)
    :t_x(t_x), t_y(t_y), t_z(t_z){}
    
    template <typename T>
    bool operator()(const T* const w_q_i, const T* ti, const T* tj, T* residuals) const
    {
        T t_w_ij[3];
        t_w_ij[0] = tj[0] - ti[0];
        t_w_ij[1] = tj[1] - ti[1];
        t_w_ij[2] = tj[2] - ti[2];
        
        T i_q_w[4];
        QuaternionInverse(w_q_i, i_q_w);
        
        T t_i_ij[3];
        ceres::QuaternionRotatePoint(i_q_w, t_w_ij, t_i_ij);
        
        residuals[0] = t_i_ij[0] - T(t_x);
        residuals[1] = t_i_ij[1] - T(t_y);
        residuals[2] = t_i_ij[2] - T(t_z);
        
        return true;
    }
    
    static ceres::CostFunction* Create(const double t_x, const double t_y, const double t_z)
    {
        return new ceres::AutoDiffCostFunction<RelativeTError, 3, 4, 3, 3>(new RelativeTError(t_x, t_y, t_z));
    }
    
    double t_x, t_y, t_z;
    
};


struct TError
{
    TError(double t_x, double t_y, double t_z)
    :t_x(t_x), t_y(t_y), t_z(t_z){}
    
    template <typename T>
    bool operator()(const T* tj, T* residuals) const
    {
        residuals[0] = tj[0] - T(t_x);
        residuals[1] = tj[1] - T(t_y);
        residuals[2] = tj[2] - T(t_z);
        
        return true;
    }
    
    static ceres::CostFunction* Create(const double t_x, const double t_y, const double t_z)
    {
        return new ceres::AutoDiffCostFunction<TError, 3, 3>(new TError(t_x, t_y, t_z));
    }
    
    double t_x, t_y, t_z;
    
};

struct RelativeRTError
{
    RelativeRTError(double t_x, double t_y, double t_z, double q_w, double q_x, double q_y, double q_z)
    :t_x(t_x), t_y(t_y), t_z(t_z), q_w(q_w), q_x(q_x), q_y(q_y), q_z(q_z)
    {
        t_norm = sqrt(t_x * t_x + t_y * t_y + t_z * t_z);
    }
    
    template <typename T>
    bool operator()(const T* const w_q_i, const T* ti, const T* w_q_j, const T* tj, T* residuals) const
    {
        T t_w_ij[3];
        t_w_ij[0] = tj[0] - ti[0];
        t_w_ij[1] = tj[1] - ti[1];
        t_w_ij[2] = tj[2] - ti[2];
        
        T i_q_w[4];
        QuaternionInverse(w_q_i, i_q_w);
        
        T t_i_ij[3];
        ceres::QuaternionRotatePoint(i_q_w, t_w_ij, t_i_ij);
        
        //residuals[0] = (t_i_ij[0] - T(t_x)) / T(t_norm);
        //residuals[1] = (t_i_ij[1] - T(t_y)) / T(t_norm);
        //residuals[2] = (t_i_ij[2] - T(t_z)) / T(t_norm);
        residuals[0] = (t_i_ij[0] - T(t_x));
        residuals[1] = (t_i_ij[1] - T(t_y));
        residuals[2] = (t_i_ij[2] - T(t_z));
        
        T relative_q[4];
        relative_q[0] = T(q_w);
        relative_q[1] = T(q_x);
        relative_q[2] = T(q_y);
        relative_q[3] = T(q_z);
        
        T q_i_j[4];
        ceres::QuaternionProduct(i_q_w, w_q_j, q_i_j);
        
        T relative_q_inv[4];
        QuaternionInverse(relative_q, relative_q_inv);
        
        T error_q[4];
        ceres::QuaternionProduct(relative_q_inv, q_i_j, error_q);
        
        residuals[3] = T(2) * error_q[1];
        residuals[4] = T(2) * error_q[2];
        residuals[5] = T(2) * error_q[3];
        
        return true;
    }
    
    static ceres::CostFunction* Create(const double t_x, const double t_y, const double t_z,
                                       const double q_w, const double q_x, const double q_y, const double q_z)
    {
        return new ceres::AutoDiffCostFunction<RelativeRTError, 6, 4, 3, 4, 3>(new RelativeRTError(t_x, t_y, t_z, q_w, q_x, q_y, q_z));
    }
    
    double t_x, t_y, t_z, t_norm;
    double q_w, q_x, q_y, q_z;
    
};

template <typename T>
void YawPitchRollToRotationMatrix(const T yaw, const T pitch, const T roll, T R[9])
{
    
    T y = yaw / T(180.0) * T(M_PI);
    T p = pitch / T(180.0) * T(M_PI);
    T r = roll / T(180.0) * T(M_PI);
    
    
    R[0] = cos(y) * cos(p);
    R[1] = -sin(y) * cos(r) + cos(y) * sin(p) * sin(r);
    R[2] = sin(y) * sin(r) + cos(y) * sin(p) * cos(r);
    R[3] = sin(y) * cos(p);
    R[4] = cos(y) * cos(r) + sin(y) * sin(p) * sin(r);
    R[5] = -cos(y) * sin(r) + sin(y) * sin(p) * cos(r);
    R[6] = -sin(p);
    R[7] = cos(p) * sin(r);
    R[8] = cos(p) * cos(r);
};

template <typename T>
void RotationMatrixTranspose(const T R[9], T inv_R[9])
{
    inv_R[0] = R[0];
    inv_R[1] = R[3];
    inv_R[2] = R[6];
    inv_R[3] = R[1];
    inv_R[4] = R[4];
    inv_R[5] = R[7];
    inv_R[6] = R[2];
    inv_R[7] = R[5];
    inv_R[8] = R[8];
};

template <typename T>
void RotationMatrixRotatePoint(const T R[9], const T t[3], T r_t[3])
{
    r_t[0] = R[0] * t[0] + R[1] * t[1] + R[2] * t[2];
    r_t[1] = R[3] * t[0] + R[4] * t[1] + R[5] * t[2];
    r_t[2] = R[6] * t[0] + R[7] * t[1] + R[8] * t[2];
};

struct FourDOFError
{
    FourDOFError(double t_x, double t_y, double t_z, double relative_yaw, double pitch_i, double roll_i)
    :t_x(t_x), t_y(t_y), t_z(t_z), relative_yaw(relative_yaw), pitch_i(pitch_i), roll_i(roll_i){}
    
    template <typename T>
    bool operator()(const T* const yaw_i, const T* ti, const T* yaw_j, const T* tj, T* residuals) const
    {
        T t_w_ij[3];
        t_w_ij[0] = tj[0] - ti[0];
        t_w_ij[1] = tj[1] - ti[1];
        t_w_ij[2] = tj[2] - ti[2];
        
        // euler to rotation
        T w_R_i[9];
        YawPitchRollToRotationMatrix(yaw_i[0], T(pitch_i), T(roll_i), w_R_i);
        // rotation transpose
        T i_R_w[9];
        RotationMatrixTranspose(w_R_i, i_R_w);
        // rotation matrix rotate point
        T t_i_ij[3];
        RotationMatrixRotatePoint(i_R_w, t_w_ij, t_i_ij);
        
        residuals[0] = (t_i_ij[0] - T(t_x));
        residuals[1] = (t_i_ij[1] - T(t_y));
        residuals[2] = (t_i_ij[2] - T(t_z));
        residuals[3] = NormalizeAngle(yaw_j[0] - yaw_i[0] - T(relative_yaw));
        
        return true;
    }
    
    static ceres::CostFunction* Create(const double t_x, const double t_y, const double t_z,
                                       const double relative_yaw, const double pitch_i, const double roll_i)
    {
        return (new ceres::AutoDiffCostFunction<
                FourDOFError, 4, 1, 3, 1, 3>(
                                             new FourDOFError(t_x, t_y, t_z, relative_yaw, pitch_i, roll_i)));
    }
    
    double t_x, t_y, t_z;
    double relative_yaw, pitch_i, roll_i;
    
};

struct FourDOFWeightError
{
    FourDOFWeightError(double t_x, double t_y, double t_z, double relative_yaw, double pitch_i, double roll_i)
    :t_x(t_x), t_y(t_y), t_z(t_z), relative_yaw(relative_yaw), pitch_i(pitch_i), roll_i(roll_i){
        weight = 10;
    }
    
    template <typename T>
    bool operator()(const T* const yaw_i, const T* ti, const T* yaw_j, const T* tj, T* residuals) const
    {
        T t_w_ij[3];
        t_w_ij[0] = tj[0] - ti[0];
        t_w_ij[1] = tj[1] - ti[1];
        t_w_ij[2] = tj[2] - ti[2];
        
        // euler to rotation
        T w_R_i[9];
        YawPitchRollToRotationMatrix(yaw_i[0], T(pitch_i), T(roll_i), w_R_i);
        // rotation transpose
        T i_R_w[9];
        RotationMatrixTranspose(w_R_i, i_R_w);
        // rotation matrix rotate point
        T t_i_ij[3];
        RotationMatrixRotatePoint(i_R_w, t_w_ij, t_i_ij);
        
        residuals[0] = (t_i_ij[0] - T(t_x)) * T(weight);
        residuals[1] = (t_i_ij[1] - T(t_y)) * T(weight);
        residuals[2] = (t_i_ij[2] - T(t_z)) * T(weight);
        residuals[3] = NormalizeAngle((yaw_j[0] - yaw_i[0] - T(relative_yaw))) * T(weight) / T(10.0);
        
        return true;
    }
    
    static ceres::CostFunction* Create(const double t_x, const double t_y, const double t_z,
                                       const double relative_yaw, const double pitch_i, const double roll_i)
    {
        return (new ceres::AutoDiffCostFunction<
                FourDOFWeightError, 4, 1, 3, 1, 3>(
                                                   new FourDOFWeightError(t_x, t_y, t_z, relative_yaw, pitch_i, roll_i)));
    }
    
    double t_x, t_y, t_z;
    double relative_yaw, pitch_i, roll_i;
    double weight;
    
};
#endif /* keyfame_database_hpp */
