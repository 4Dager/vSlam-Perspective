//
//  draw_result.hpp
//  PerspectiveSlam
//
//  Created by HKUST Aerial Robotics on 2016/11/16.
//  Copyright © 2017 HKUST Aerial Robotics. All rights reserved.
//

#ifndef draw_result_hpp
#define draw_result_hpp

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>
#include "utility.hpp"
#include "global_param.hpp"
#include "delaunay.h"


#define HEIGHT 640
#define WIDTH 480
#define RADIUS 30


struct GroundPoint
{
    int idx;
    Eigen::Vector3f center;
    bool boxflag;
    bool moveflag;
    Eigen::Vector3f ori, cox, coy, coz;
    Eigen::Vector3f lix, liy, liz;
    float size;
    Eigen::Vector4f initPlane;
    
    GroundPoint(int idx_, Eigen::Vector3f center_)
    {
        idx = idx_;
        center = center_;
        boxflag = false;
        moveflag = false;
    }
};

class DrawResult
{
public:
    DrawResult(float _pitch, float _roll, float _yaw, float _Tx, float _Ty, float _Tz);
    
    float roll, pitch, yaw; //in degree
    float Tx, Ty, Tz; //in meters, only in z axis
    float radius, radiusAR;
    float theta, phy;
    float thetaAR, phyAR;
    float locationX, locationY;
    float locationX_p, locationY_p;
    float locationXT2, locationYT2;
    float locationXT2_p, locationYT2_p;
    float locationXP, locationYP;
    float locationXP_p, locationYP_p;
    float locationTapX, locationTapY;
    bool tapFlag;
    float locationLongPressX;
    float locationLongPressY;
    bool longPressFlag;
    
    
    float theta_p, phy_p, radius_p;
    float X0_p, Y0_p;
    int finger_state;
    int finger_s;
    int finger_d;
    int finger_p;
    
    //map<int, pair <int, pair< Eigen::Vector3f, std::vector<Eigen::Vector3f> > > > Ground;
    //map<int, pair <int, Eigen::Vector3f > > Ground;
    std::vector<GroundPoint> Grounds;
    
    int Ground_idx;
    
    
    
    float lengthCube;
    std::vector<Eigen::Vector3f> pose;
    std::vector<int> segment_indexs;
    float Fx,Fy;
    bool change_view_manualy;
    bool planeInit;
    bool startInit;
    Eigen::Vector3f initPoint;
    Eigen::Vector4f initPlane;
    Eigen::Vector3f origin;
    Eigen::Vector3f ConerX, ConerY, ConerZ;
    Eigen::Vector3f lineX,lineY,lineZ;
    Eigen::Vector3f origin_w;
    float X0, Y0;
    float X0AR, Y0AR;
    
    
    bool look_down;
    //for optical flow EKF
    cv::Mat pre_image;
    cv::Mat cur_image;
    std::vector<cv::Point2f> pre_pts;
    std::vector<cv::Point2f> cur_pts;
    std::vector<cv::Point2f> n_pts;
    cv::Point2f flow;  //velocity of the pixel
    bool KF_init;
    
    std::vector<cv::Scalar> trajectory_color;
    std::vector<int> indexs;
    bool change_color;
    //4 sizes
    std::vector<Eigen::Vector2f> pre_status, cur_status;
    std::vector<Eigen::Matrix2f> K;
    std::vector<Eigen::Matrix2f> cur_cov, pre_cov;
    void computeAR(std::vector<Eigen::Vector3f> &point_cloud, Eigen::Vector3f &model);
    
    void drawAR(cv::Mat &result, std::vector<Eigen::Vector3f> &point_cloud, Eigen::Vector3f P_latest, Eigen::Matrix3f R_latest);
    void drawGround(cv::Mat &result, std::vector<Eigen::Vector3f> &point_cloud, Eigen::Vector3f P_latest, Eigen::Matrix3f R_latest);
    void drawBox(cv::Mat &result, Eigen::Vector3f corner_0, Eigen::Vector3f corner_x, Eigen::Vector3f corner_y, Eigen::Vector3f corner_z, float size, Eigen::Vector3f P_latest, Eigen::Matrix3f R_latest, bool inAR);
    void Reprojection(cv::Mat &result, std::vector<Eigen::Vector3f> &point_cloud, const Eigen::Matrix3f *R_window,const Eigen::Vector3f *T_window, bool box_in_trajectory);
    std::vector<Eigen::Vector3f> calculate_camera_pose(Eigen::Vector3f camera_center, Eigen::Matrix3f Rc, float length);
    cv::Point2f World2VirturCam(Eigen::Vector3f xyz, float &depth);
    void drawBoxVirturCam(cv::Mat &result);
    void rejectWithF();
    cv::Scalar newColor();
private:
    Eigen::Vector3f findZfromXY(Eigen::Vector3f point, Eigen::Vector4f plane);
    Eigen::Vector4f findPlane(std::vector<Eigen::Vector3f> &point_cloud);
    Eigen::Vector3f findGround(std::vector<Eigen::Vector3f> &point_cloud, std::vector<Eigen::Vector3f> &inlier_points);
};

#endif /* draw_result_hpp */
