//
//  VinsViewController.h
//  PerspectiveSlam
//
//  Created by HKUST Aerial Robotics on 2016/10/18.
//  Copyright © 2017 HKUST Aerial Robotics. All rights reserved.
//

#import <UIKit/UIKit.h>
#import <opencv2/imgcodecs/ios.h>
#import <opencv2/videoio/cap_ios.h>
#import "feature_tracker.hpp"
#import "global_param.hpp"
#import "VINS.hpp"
#import "draw_result.hpp"
#include "keyframe.hpp"
#include "loop_closure.hpp"
#include "keyfame_database.hpp"
#import "VinsDefine.h"

#import <CoreMotion/CoreMotion.h>
#import <mach/mach_time.h>
#import <sys/utsname.h>
#include <queue>


@interface VinsViewController : UIViewController<CvVideoCameraDelegate,UITextViewDelegate>

@end
