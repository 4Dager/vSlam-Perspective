//
//  keyfame_database.cpp
//  PerspectiveSlam
//
//  Created by HKUST Aerial Robotics on 2017/5/2.
//  Copyright © 2017 HKUST Aerial Robotics. All rights reserved.
//

#include "keyfame_database.hpp"
using namespace std;
using namespace Eigen;

KeyFrameDatabase::KeyFrameDatabase()
{
    earliest_loop_index = -1;
    t_drift = Eigen::Vector3d(0, 0, 0);
    yaw_drift = 0;
    r_drift = Eigen::Matrix3d::Identity();
    max_frame_num = 50;
    total_length = 0;
    last_P = Eigen::Vector3d(0, 0, 0);
    cur_seg_index = max_seg_index = 0;
}
void KeyFrameDatabase::add(KeyFrame* pKF)
{
    unique_lock<mutex> lock(mMutexkeyFrameList);
    keyFrameList.push_back(pKF);
    Eigen::Vector3d P;
    Matrix3d R;
    pKF->getPose(P, R);
    P = r_drift * P + t_drift;
    R = r_drift * R;
    pKF->updatePose(P, R);
    Eigen::Quaterniond Q;
    Q = R;
    
    total_length += (P - last_P).norm();
    last_P = P;
    
    // add key frame to path for visualization
    refine_path.push_back(P.cast<float>());
    segment_indexs.push_back(pKF->segment_index);
    lock.unlock();
    
}

void KeyFrameDatabase::resample(vector<int> &erase_index)
{
    unique_lock<mutex> lock(mMutexkeyFrameList);
    if ((int)keyFrameList.size() < max_frame_num) {
        printf("[my_debug]: frame num: %lu\n", keyFrameList.size());
        return;
    }
    printf("[my_debug]: oooooops, frame num is bigger than %d.\n", max_frame_num);
    double min_dis = total_length / (max_frame_num * 0.7);
    
    list<KeyFrame*>::iterator it = keyFrameList.begin();
    Vector3d last_P = Vector3d(0, 0, 0);
    for (; it != keyFrameList.end(); )
    {
        Eigen::Vector3d tmp_t;
        Matrix3d tmp_r;
        (*it)->getPose(tmp_t, tmp_r);
        double dis = (tmp_t - last_P).norm();
        if(dis > min_dis || (*it)->has_loop || (*it)->is_looped /*|| !(*it)->check_loop*/)
        {
            last_P = tmp_t;
            it++;
        }
        else
        {
            erase_index.push_back((*it)->global_index);
            it = keyFrameList.erase(it);
        }
    }
    lock.unlock();
    updateVisualization();
}

void KeyFrameDatabase::erase(KeyFrame* pKF)
{
    list<KeyFrame*>::iterator it = find(keyFrameList.begin(), keyFrameList.end(), pKF);
    assert(it != keyFrameList.end());
    if (it != keyFrameList.end())
        keyFrameList.erase(it);
}

int KeyFrameDatabase::size()
{
    unique_lock<mutex> lock(mMutexkeyFrameList);
    return (int)keyFrameList.size();
}

KeyFrame* KeyFrameDatabase::getKeyframe(int index)
{
    unique_lock<mutex> lock(mMutexkeyFrameList);
    list<KeyFrame*>::iterator it = keyFrameList.begin();
    for (; it != keyFrameList.end(); it++)
    {
        if((*it)->global_index == index)
            break;
    }
    if (it != keyFrameList.end())
        return *it;
    else
        return NULL;
}

KeyFrame* KeyFrameDatabase::getLastKeyframe()
{
    unique_lock<mutex> lock(mMutexkeyFrameList);
    list<KeyFrame*>::reverse_iterator rit = keyFrameList.rbegin();
    assert(rit != keyFrameList.rend());
    return *rit;
}

KeyFrame* KeyFrameDatabase::getLastKeyframe(int last_index)
{
    unique_lock<mutex> lock(mMutexkeyFrameList);
    list<KeyFrame*>::reverse_iterator rit = keyFrameList.rbegin();
    for (int i = 0; i < last_index; i++)
    {
        rit++;
        assert(rit != keyFrameList.rend());
    }
    return *rit;
}

KeyFrame* KeyFrameDatabase::getLastUncheckKeyframe()
{
    unique_lock<mutex> lock(mMutexkeyFrameList);
    list<KeyFrame*>::reverse_iterator rit = keyFrameList.rbegin();
    for (; rit != keyFrameList.rend(); rit++)
    {
        if ((*rit)->check_loop == 1)
            break;
    }
    assert(rit != keyFrameList.rbegin());
    return *(--rit);
}

void KeyFrameDatabase::optimize4DoFLoopPoseGraph(int cur_index, Eigen::Vector3d &loop_correct_t, Eigen::Matrix3d &loop_correct_r)
{
    printf("[my_debug]:loop global pose graph\n");
    KeyFrame* cur_kf = getKeyframe(cur_index);
    int loop_index = cur_kf->loop_index;
    printf("loop bug current %d %d\n", cur_kf->global_index, cur_kf->loop_index);
    if (earliest_loop_index > loop_index || earliest_loop_index == -1)
        earliest_loop_index = loop_index;
    assert(cur_kf->has_loop == 1);
    int max_length = cur_index + 1;
    
    // w^t_i   w^q_i
    double t_array[max_length][3];
    Eigen::Quaterniond q_array[max_length];
    double euler_array[max_length][3];
    
    ceres::Problem problem;
    ceres::Solver::Options options;
    //options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    //options.minimizer_progress_to_stdout = true;
    options.max_num_iterations = 5;
    ceres::Solver::Summary summary;
    ceres::LossFunction *loss_function;
    loss_function = new ceres::HuberLoss(1.0);
    ceres::LocalParameterization* angle_local_parameterization = AngleLocalParameterization::Create();
    
    list<KeyFrame*>::iterator it;
    int i = 0;
    for (it = keyFrameList.begin(); it != keyFrameList.end(); it++)
    {
        if ((*it)->global_index < earliest_loop_index)
            continue;
        (*it)->resample_index = i;
        Eigen::Quaterniond tmp_q;
        Matrix3d tmp_r;
        Eigen::Vector3d tmp_t;
        (*it)->getOriginPose(tmp_t, tmp_r);
        tmp_q = tmp_r;
        t_array[i][0] = tmp_t(0);
        t_array[i][1] = tmp_t(1);
        t_array[i][2] = tmp_t(2);
        q_array[i] = tmp_q;
        
        Eigen::Vector3d euler_angle = Utility::R2ypr(tmp_q.toRotationMatrix());
        euler_array[i][0] = euler_angle.x();
        euler_array[i][1] = euler_angle.y();
        euler_array[i][2] = euler_angle.z();
        
        problem.AddParameterBlock(euler_array[i], 1, angle_local_parameterization);
        problem.AddParameterBlock(t_array[i], 3);
        
        if ((*it)->global_index == earliest_loop_index)
        {
            problem.SetParameterBlockConstant(euler_array[i]);
            problem.SetParameterBlockConstant(t_array[i]);
        }
        
        //add edge
        for (int j = 1; j < 5; j++)
        {
            if (i - j >= 0)
            {
                Vector3d euler_conncected = Utility::R2ypr(q_array[i-j].toRotationMatrix());
                Vector3d relative_t(t_array[i][0] - t_array[i-j][0], t_array[i][1] - t_array[i-j][1], t_array[i][2] - t_array[i-j][2]);
                relative_t = q_array[i-j].inverse() * relative_t;
                double relative_yaw = euler_array[i][0] - euler_array[i-j][0];
                ceres::CostFunction* cost_function = FourDOFError::Create( relative_t.x(), relative_t.y(), relative_t.z(),
                                                                          relative_yaw, euler_conncected.y(), euler_conncected.z());
                problem.AddResidualBlock(cost_function, loss_function, euler_array[i-j],
                                         t_array[i-j],
                                         euler_array[i],
                                         t_array[i]);
            }
        }
        
        //add loop edge
        if((*it)->has_loop)
        {
            int connected_index = getKeyframe((*it)->loop_index)->resample_index;
            if((*it)->loop_index < earliest_loop_index)
            {
                printf("loop bug %d %d\n", (*it)->global_index, (*it)->loop_index);
                assert(false);
            }
            Eigen::Vector3d euler_conncected = Utility::R2ypr(q_array[connected_index].toRotationMatrix());
            Eigen::Vector3d relative_t((*it)->loop_info(0), (*it)->loop_info(1), (*it)->loop_info(2));
            double relative_yaw = (*it)->loop_info(7);
            ceres::CostFunction* cost_function = FourDOFWeightError::Create( relative_t.x(), relative_t.y(), relative_t.z(),
                                                                            relative_yaw, euler_conncected.y(), euler_conncected.z());
            problem.AddResidualBlock(cost_function, NULL, euler_array[connected_index],
                                     t_array[connected_index],
                                     euler_array[i],
                                     t_array[i]);
            
        }
        if ((*it)->global_index == cur_index)
            break;
        i++;
    }
    TS(t_global_loop);
    ceres::Solve(options, &problem, &summary);
    TE(t_global_loop);
    std::cout << summary.BriefReport() << "\n";
    
    i = 0;
    int seg_index_cur, seg_index_old;
    seg_index_cur = cur_kf->segment_index;
    seg_index_old = getKeyframe(cur_kf->loop_index)->segment_index;
    cur_seg_index = seg_index_old;
    for (it = keyFrameList.begin(); it != keyFrameList.end(); it++)
    {
        if ((*it)->global_index < earliest_loop_index)
            continue;
        Eigen::Quaterniond tmp_q;
        tmp_q = Utility::ypr2R(Eigen::Vector3d(euler_array[i][0], euler_array[i][1], euler_array[i][2]));
        Eigen::Vector3d tmp_t = Eigen::Vector3d(t_array[i][0], t_array[i][1], t_array[i][2]);
        Matrix3d tmp_r = tmp_q.toRotationMatrix();
        (*it)-> updatePose(tmp_t, tmp_r);
        if ((*it)->global_index == cur_index)
            break;
        i++;
        if((*it)->segment_index == seg_index_cur)
            (*it)->segment_index = seg_index_old;
    }
    
    Eigen::Vector3d cur_t, origin_t;
    Matrix3d cur_r, origin_r;
    cur_kf->getPose(cur_t, cur_r);
    cur_kf->getOriginPose(origin_t, origin_r);
    yaw_drift = Utility::R2ypr(cur_r).x() - Utility::R2ypr(origin_r).x();
    r_drift = Utility::ypr2R(Eigen::Vector3d(yaw_drift, 0, 0));
    t_drift = cur_t - r_drift * origin_t;
    
    for (; it != keyFrameList.end(); it++)
    {
        Eigen::Vector3d P;
        Matrix3d R;
        (*it)->getOriginPose(P, R);
        P = r_drift * P + t_drift;
        R = r_drift * R;
        (*it)-> updatePose(P, R);
    }
    loop_correct_t = t_drift;
    loop_correct_r = r_drift;
    updateVisualization();
}

void KeyFrameDatabase::updateVisualization()
{
    total_length = 0;
    last_P = Eigen::Vector3d(0, 0, 0);
    //update visualization
    list<KeyFrame*>::iterator it;
    
    refine_path.clear();
    segment_indexs.clear();
    all_keyframes.clear();
    for (it = keyFrameList.begin(); it != keyFrameList.end(); it++)
    {
        Eigen::Vector3d P;
        Matrix3d R;
        (*it)->getPose(P, R);
        Eigen::Quaterniond Q;
        Q = R;
        
        total_length += (P - last_P).norm();
        last_P = P;
        
        // add key frame to path for visualization
        refine_path.push_back(P.cast<float>());
        segment_indexs.push_back((*it)->segment_index);
        
        KEYFRAME_DATA keyframe_data;
        keyframe_data.header = (*it)->header;
        keyframe_data.translation = P;
        keyframe_data.rotation = Q;
        all_keyframes.push_back(keyframe_data);
    }
    printf("[my_debug]: key frames count: %lu\n",keyFrameList.size());
    printf("loop update visualization\n");
}

void KeyFrameDatabase::addLoop(int loop_index)
{
    KeyFrame* cur_KF = getLastKeyframe();
    
    KeyFrame* connected_KF = getKeyframe(loop_index);
    Eigen::Vector3d conncected_P, P;
    Matrix3d connected_R, R;
    cur_KF->getPose(P, R);
    connected_KF->getPose(conncected_P, connected_R);
}
