//
//  CameraUtils.m
//  PerspectiveSlam
//
//  Created by Yang Liu on 3/18/17.
//  Copyright © 2017 栗大人. All rights reserved.
//

#import "CameraUtils.h"
#import <AVFoundation/AVFoundation.h>

@implementation CameraUtils

+ (void)setExposureOffset:(float)ev {
    AVCaptureDevice *device = [AVCaptureDevice defaultDeviceWithMediaType:AVMediaTypeVideo];
    NSLog(@"Camera: activeFormat %@", [device activeFormat]);
    
    NSError * error;
    [device lockForConfiguration:&error];
    if (error != nil) {
        NSLog(@"Camera: Error %@", error);
        return;
    }
    
    NSLog(@"Camera: exposure duration (before EV) %f", CMTimeGetSeconds(device.exposureDuration));
    __weak typeof(device) weakDevice = device;
    [weakDevice setExposureTargetBias:ev completionHandler:^(CMTime syncTime) {
        __strong typeof(weakDevice) strongDevice = weakDevice;
        NSLog(@"timestamp %lld", syncTime.value/syncTime.timescale);
        NSLog(@"Camera: exposure duration (after EV) %f", CMTimeGetSeconds(strongDevice.exposureDuration));
    }];

    [device unlockForConfiguration];
}

@end
