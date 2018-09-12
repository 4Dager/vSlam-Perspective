//
//  MotionRecorder.m
//  PerspectiveSlam
//
//  Created by  zcating on 27/03/2018.
//  Copyright © 2018 栗大人. All rights reserved.
//
#import <CoreMotion/CoreMotion.h>

#import "MotionRecorder.h"
#import "VinsDefine.h"
#import "global_param.hpp"

using namespace std;

@interface MotionRecorder() {
    shared_ptr<IMU_MSG> _currentAcceleratorData;
    // for Interpolation
    vector<IMU_MSG> _gyroBuffer;
    int _imuPrepareCount;
    NSTimeInterval _lateastIMUTime;
}

@property (nonatomic, strong) CMMotionManager *motionManager;
@property (nonatomic, strong) NSLock *condition;

@end

@implementation MotionRecorder


-(instancetype)init {
    if (self) {
        self.motionManager = [CMMotionManager new];
//        self.condition = [NSCondition new];
        _currentAcceleratorData = make_shared<IMU_MSG>(new IMU_MSG());
    }
}

- (void)startRecord {
    if (!self.motionManager.accelerometerAvailable) {
        NSLog(@"没有加速计");
    }
#ifdef DATA_EXPORT
    self.motionManager.accelerometerUpdateInterval = 0.1;
    self.motionManager.gyroUpdateInterval = 0.1;
#else
    self.motionManager.accelerometerUpdateInterval = 0.01;
    self.motionManager.gyroUpdateInterval = 0.01;
#endif
    
    [self.motionManager startDeviceMotionUpdates];
    [self.motionManager startAccelerometerUpdatesToQueue:[NSOperationQueue currentQueue] withHandler:^(CMAccelerometerData *latestAcc, NSError *error) {
        if(_imuPrepareCount < 10) {
            _imuPrepareCount++;
        }
        shared_ptr<IMU_MSG> acceleratorMessage(new IMU_MSG());
        acceleratorMessage->header = latestAcc.timestamp;
        acceleratorMessage->acc << -latestAcc.acceleration.x * GRAVITY,
        -latestAcc.acceleration.y * GRAVITY,
        -latestAcc.acceleration.z * GRAVITY;
        _currentAcceleratorData = acceleratorMessage;
    }];
    
    [self.motionManager startGyroUpdatesToQueue:[NSOperationQueue currentQueue] withHandler:^(CMGyroData *latestGyro, NSError *error) {
        //The time stamp is the amount of time in seconds since the device booted.
        NSTimeInterval header = latestGyro.timestamp;
        if(header <= 0 || _imuPrepareCount < 10) {
            return;
        }
        IMU_MSG gyroMessage;
        gyroMessage.header = header;
        gyroMessage.gyr << latestGyro.rotationRate.x,
        latestGyro.rotationRate.y,
        latestGyro.rotationRate.z;
        
        if(_gyroBuffer.size() == 0) {
            _gyroBuffer.push_back(gyroMessage);
            _gyroBuffer.push_back(gyroMessage);
            return;
        } else {
            _gyroBuffer[0] = _gyroBuffer[1];
            _gyroBuffer[1] = gyroMessage;
        }
        //interpolation
        shared_ptr<IMU_MSG> imuMessage(new IMU_MSG());
        if(_currentAcceleratorData->header >= _gyroBuffer[0].header && _currentAcceleratorData->header < _gyroBuffer[1].header) {
            imuMessage->header = _currentAcceleratorData->header;
            imuMessage->acc = _currentAcceleratorData->acc;
            imuMessage->gyr = _gyroBuffer[0].gyr + (_currentAcceleratorData->header - _gyroBuffer[0].header)*(_gyroBuffer[1].gyr - _gyroBuffer[0].gyr)/(_gyroBuffer[1].header - _gyroBuffer[0].header);
            //printf("imu gyro update %lf %lf %lf\n", _gyroBuffer[0].header, imuMessage->header, _gyroBuffer[1].header);
            //printf("imu inte update %lf %lf %lf %lf\n", imuMessage->header, _gyroBuffer[0].gyr.x(), imuMessage->gyr.x(), _gyroBuffer[1].gyr.x());
        } else {
            printf("imu error %lf %lf %lf\n", _gyroBuffer[0].header, _currentAcceleratorData->header, _gyroBuffer[1].header);
            return;
        }
        _lateastIMUTime = imuMessage->header;
        
        //img_msg callback
        if(USE_PNP) {
            IMU_MSG_LOCAL imuMessage_local;
            imuMessage_local.header = imuMessage->header;
            imuMessage_local.acc = imuMessage->acc;
            imuMessage_local.gyr = imuMessage->  gyr;
            
            [self.condition lock];
            local_imuMessage_buf.push(imuMessage_local);
            [self.condition unlock];
        }
        m_buf.lock();
        imuMessage_buf.push(imuMessage);
        //NSLog(@"IMU_buf timestamp %lf, acc_x = %lf",imuMessage_buf.front()->header,imuMessage_buf.front()->acc.x());
        m_buf.unlock();
        con.notify_one();
    }];
}

@end
