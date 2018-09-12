//
//  VinsViewController.m
//  PerspectiveSlam
//
//  Created by HKUST Aerial Robotics on 2016/10/18.
//  Copyright © 2017 HKUST Aerial Robotics. All rights reserved.
//

#import "VinsViewController.h"
#import "utility.hpp"
#import "CameraUtils.h"


// IMU: Inertial measurement unit
//

using namespace std;
using namespace Eigen;
using namespace cv;
using namespace Vins;


@interface VinsViewController () {
    BOOL isCapturing;
    cv::Size frameSize;
    uint64_t prevTime;
}

@property (weak, nonatomic) IBOutlet UILabel *progressLabel;
@property (weak, nonatomic) IBOutlet UIImageView* imageView;
@property (strong, nonatomic) UIActivityIndicatorView *indicator;

@property (nonatomic, strong) CvVideoCamera* videoCamera;

@property (nonatomic, strong) CMMotionManager *motionManager;

@property (nonatomic, strong) NSCondition *condition;

@property (nonatomic, strong) NSThread *mainLoop;
@property (nonatomic, strong) NSThread *draw;
@property (nonatomic, strong) NSThread *saveData;
@property (nonatomic, strong) NSThread *loopThread;
@property (nonatomic, strong) NSThread *globalLoopThread;

//@property (nonatomic, assign) cv::Ptr<FeatureTracker> feature_tracker;

@end

@implementation VinsViewController
/******************************* UI CONFIG *******************************/

// true:  VINS trajectory is the main view, AR image is in left bottom
// false: AR image is the main view, VINS is in left bottom
bool ui_main = false;

bool box_in_AR = false;

bool box_in_trajectory = false;

// If initialized finished, start show is true
bool enableShowing = false;


/******************************* UI CONFIG *******************************/

FeatureTracker featureTracker;

VINS vins;

// Store the fesature data processed by featureTracker
queue<ImageConstPtr> imageMessageBuffer;

// Store the IMU data for vins
queue<IMUConstPtr> imuMessageBuffer;

// Store the IMU data for motion-only vins
queue<IMU_MSG_LOCAL> imuLocalMessageBuffer;

// The number of measurements waiting to be processed
int waitingCount = 0;

int frameCount = 0;

// Lock the feature and imu data buffer
std::mutex bufferMutex;

std::condition_variable con;

NSTimeInterval currentTime = -1;

NSTimeInterval imuLateastTime = -1;

int imuPrepareCount = 0;


// Segment the trajectory using color when re-initialize
int segmentationIndex = 0;

// Lock the solved VINS data feedback to featureTracker
std::mutex depthFeedbackMutex;

// Lock the IMU data feedback to featureTracker
std::mutex imuFeedbackMutex;

// Solved VINS feature feedback to featureTracker
list<IMG_MSG_LOCAL> solvedFeatures;

// Solved VINS status feedback to featureTracker
VINS_RESULT solvedResult;

/******************************* Loop Closure ******************************/

// Raw image data buffer for extracting FAST feature
queue<pair<cv::Mat, double>> imageBufferLoop;

// Lock the imageBufferLoop
std::mutex imageBufferLoopMutex;

// Detect loop
SharedRef<LoopClosure> loopClosure;

// Keyframe database
KeyFrameDatabase database;

// Control the loop detection frequency
int keyFrameFrequence = 0;

// Index the keyframe
int globalFrameCount = 0;

// Record the checked loop frame
int loopCheckedCount = 0;

// Indicate the loop frame index
int oldIndex = -1;


// Store the indexs of the keyframe which need to be erased
Vins::Array<int> erasingIndices;

// Translation drift
Eigen::Vector3d loop_correct_t = Eigen::Vector3d(0, 0, 0);

// Rotation drift
Eigen::Matrix3d loop_correct_r = Eigen::Matrix3d::Identity();

/******************************* Loop Closure ******************************/

// MARK: Unity Camera Mode Switching
// Ground truth from UI switch property "self.switchUIAREnabled"

// Implied, updated by updateCameraMode()
bool imuPredictEnabled = false;

// Implied, updated by updateCameraMode()
bool cameraMode = true;

// Implied, updated by updateCameraMode()
bool imageCacheEnabled = cameraMode && !USE_PNP;


// MARK: Lazy Load Object
- (CMMotionManager *)motionManager {
    if (_motionManager == nil) {
        _motionManager = [[CMMotionManager alloc] init];
    }
    return _motionManager;
}


// MARK: ViewController Methods

- (void)viewDidLoad {
    [super viewDidLoad];
    
    /*******************************************Camera setup*******************************************/
    self.videoCamera = [[CvVideoCamera alloc] initWithParentView:self.imageView];
    self.videoCamera.delegate = self;
    self.videoCamera.defaultAVCaptureDevicePosition = AVCaptureDevicePositionBack;
    self.videoCamera.defaultAVCaptureVideoOrientation = AVCaptureVideoOrientationPortrait;
    self.videoCamera.defaultAVCaptureSessionPreset = AVCaptureSessionPreset640x480;
    self.videoCamera.defaultFPS = 30;
    
    [CameraUtils setExposureOffset: -1.0f];
    [self.videoCamera start];
    
    /***************************************UI configuration*****************************************/
    UIPanGestureRecognizer *pan = [[UIPanGestureRecognizer alloc] initWithTarget:self action:@selector(handlePan:)];
    pan.minimumNumberOfTouches = 1;
    pan.maximumNumberOfTouches = 2;
    [self.imageView addGestureRecognizer:pan];
    
    UIPinchGestureRecognizer *pinch = [[UIPinchGestureRecognizer alloc] initWithTarget:self action:@selector(handlePinch:)];
    [self.imageView addGestureRecognizer:pinch];

    self.indicator = [[UIActivityIndicatorView alloc] initWithActivityIndicatorStyle:UIActivityIndicatorViewStyleWhiteLarge];
    self.indicator.center = CGPointMake(self.imageView.frame.size.width * 0.5, self.imageView.frame.size.height * 0.22);
    self.indicator.color = [UIColor darkGrayColor];
    [self.indicator startAnimating];
    [self.view addSubview:self.indicator];

    
    /****************************************Init all the thread****************************************/
    self.condition = [[NSCondition alloc] init];
    self.mainLoop = [[NSThread alloc]initWithTarget:self selector:@selector(run) object:nil];
    [self.mainLoop setName:@"mainLoop"];
    
    if(ENABLE_LOOP_CLOSURE) {
        //loop closure thread
        self.loopThread = [[NSThread alloc]initWithTarget:self selector:@selector(runLoopThread) object:nil];
        [self.loopThread setName:@"loopThread"];
        [self.loopThread start];
        
        self.globalLoopThread = [[NSThread alloc]initWithTarget:self selector:@selector(runGlobalLoopThread) object:nil];
        [self.globalLoopThread setName:@"globalLoopThread"];
        [self.globalLoopThread start];
    }
    
    bool deviceCheck = setGlobalParam(iPhone7);
    if(!deviceCheck) {
        [self showMessage:@"Unsupported Device" withTitle:@"Supported devices are: iPhone7 Plus, iPhone7, iPhone6s Plus, iPhone6s"];
    }
    vins.setExtrinsic();
    vins.setIMUModel();
    featureTracker.vins_pnp.setExtrinsic();
    featureTracker.vins_pnp.setIMUModel();
    [self imuStartUpdate];
    isCapturing = YES;
    [self.mainLoop start];
    frameSize = cv::Size(self.videoCamera.imageWidth, self.videoCamera.imageHeight);
}

- (void)didReceiveMemoryWarning {
    [super didReceiveMemoryWarning];
    // Dispose of any resources that can be recreated.
}



/*
 Main process image thread: this thread detects and track feature between two continuous images
 and takes the newest VINS result and the corresponding image to draw AR and trajectory.
 */
queue<ImageDataCache> imagePool;
queue<VINSDataCache> vinsPool;
ImageDataCache imageDataCache;
cv::Mat lateastEqua;
UIImage *lateastImage;

//the lateast position and rotation of your device corresponding to AR image.
Vector3f lateastPosition;
Matrix3f lateastRotation;

Vector3d pnpPosition;
Matrix3d pnpRotation;

- (void)processImage:(cv::Mat&)image {
    if(isCapturing == YES && imuLateastTime > 0) {
        // modify opencv library, timestamp was stored at index 0,0
        float lowPart = image.at<float>(0, 0);
        float highPart = image.at<float>(0, 1);
        
        // for storing pointcloud;
        SharedRef<ImageMessage> imageMessage(new ImageMessage());
        imageMessage->header = [[NSProcessInfo processInfo] systemUptime];
        float Group[2];
        Group[0] = lowPart;
        Group[1] = highPart;
        double* time_now_decode = (double*)Group;
        double time_stamp = *time_now_decode;

        //the timestamp of point clound is the img's timestamp;
        imageMessage->header = time_stamp;
        
        BOOL isNeedRotation = image.size() != frameSize;
        
        //for save data
        cv::Mat input_frame;
        input_frame = image;
        
        
        prevTime = mach_absolute_time();
        
        cv::Mat gray;
        cv::cvtColor(input_frame, gray, CV_RGBA2GRAY);
        cv::Mat img_with_feature;
        cv::Mat img_equa;
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(); //1.pre-process img;
        clahe->setClipLimit(3);
        clahe->apply(gray, img_equa);
        
        if(USE_PNP) {
            depthFeedbackMutex.lock();
            featureTracker.solved_features = solvedFeatures;
            featureTracker.solved_vins = solvedResult;
            depthFeedbackMutex.unlock();
            
            imuFeedbackMutex.lock();
            featureTracker.imu_msgs = GetImuMeasurements(imageMessage->header);
            imuFeedbackMutex.unlock();
        }
        vector<Point2f> goodPoints;
        vector<double> track_len;
        bool vins_normal = (vins.solver_flag == VINS::NON_LINEAR);
        
        //after this can get pnpPosition and pnpRotation directly.
        featureTracker.readImage(img_equa, img_with_feature, frameCount, goodPoints, track_len, imageMessage->header, pnpPosition, pnpRotation, vins_normal);
        
        for (int i = 0; i < goodPoints.size(); i++) {
            cv::circle(image, goodPoints[i], 0, cv::Scalar(255 * (1 - track_len[i]), 0, 255 * track_len[i]), 7); //BGR
        }
        
        //image msg buf
        //publish the point cloud to the main process thread.
        if(featureTracker.img_cnt==0) {
            imageMessage->point_clouds = featureTracker.image_msg;
            //img_msg callback
            bufferMutex.lock();
            imageMessageBuffer.push(imageMessage);
            //NSLog(@"Img timestamp %lf",imageMessageBuffer.front()->header);
            bufferMutex.unlock();
            con.notify_one();
            if(imageCacheEnabled) {
                imageDataCache.header = imageMessage->header;
                imageDataCache.image = MatToUIImage(image);
                imagePool.push(imageDataCache);
            }
            
            if(ENABLE_LOOP_CLOSURE) {
                imageBufferLoopMutex.lock();
                cv::Mat loopImage = gray.clone();
                imageBufferLoop.push(make_pair(loopImage, imageMessage->header));//prepare the raw image for loop closure.
                if(imageBufferLoop.size() > WINDOW_SIZE)
                    imageBufferLoop.pop();
                imageBufferLoopMutex.unlock();
            }
        }

         // control the frequency
        featureTracker.img_cnt = (featureTracker.img_cnt + 1) % FREQ;
        // fix by zuohua201804: 缺少openCV画点代码
        for (int i = 0; i < goodPoints.size(); i++)
        {
            cv::circle(image, goodPoints[i], 0, cv::Scalar(255 * (1 - track_len[i]), 0, 255 * track_len[i]), 7); //BGR
        }
        
        /********it is same with the code from line 465 to 468,so I quit it,by cdx on 3.7*******
         for (int i = 0; i < good_pts.size(); i++)
         {
         cv::circle(image, good_pts[i], 0, cv::Scalar(255 * (1 - track_len[i]), 0, 255 * track_len[i]), 7); //BGR
         }
         *****************************************************************************************/
        
        // not use pnp stratege
        if(imageCacheEnabled) {
            //use aligned vins and image
            if(!vinsPool.empty() && !imagePool.empty()) {
                while(vinsPool.size() > 1) {
                    vinsPool.pop();
                }
                while(!imagePool.empty() && imagePool.front().header < vinsPool.front().header) {
                    imagePool.pop();
                }
                if(!vinsPool.empty() && !imagePool.empty()) {
                    lateastImage = imagePool.front().image;
                    lateastPosition = vinsPool.front().P;
                    lateastRotation = vinsPool.front().R;
                    UIImageToMat(lateastImage, image);
                }
            } else if(!imagePool.empty()) {
                if(imagePool.size() > 10)
                    imagePool.pop();
            }
        }
        if(USE_PNP) {
            lateastPosition = pnpPosition.cast<float>();
            lateastRotation = pnpRotation.cast<float>();
        }
        if(ui_main || enableShowing == false || vins.solver_flag != VINS::NON_LINEAR) {
            //show image and AR
            cv::Mat tmp2;
            if(vins.solver_flag == VINS::NON_LINEAR && enableShowing) {
                cv::Mat tmp;
                vins.drawresult.startInit = true;
                vins.drawresult.drawAR(vins.imageAI, vins.correct_point_cloud, lateastPosition, lateastRotation);
                
                cv::cvtColor(image, tmp, CV_RGBA2BGR);
                cv::Mat mask;
                cv::Mat imageAI = vins.imageAI;
                if(!imageAI.empty())
                    cv::cvtColor(imageAI, mask, CV_RGB2GRAY);
                imageAI.copyTo(tmp,mask);
                cv::cvtColor(tmp, image, CV_BGRA2BGR);
            }
            if(DEBUG_MODE) {
                cv::flip(lateastEqua, image, -1);
            } else {
                cv::flip(image,tmp2,-1);
                image = tmp2;
                if(vins.solver_flag != VINS::NON_LINEAR || !enableShowing)
                    cv::cvtColor(image, image, CV_RGBA2BGR);
            }
        } else {
            //show VINS
            if(vins.solver_flag == VINS::NON_LINEAR) {
                vins.drawresult.pose.clear();
                vins.drawresult.pose = database.refine_path;
                vins.drawresult.segment_indexs = database.segment_indexs;
                vins.drawresult.Reprojection(vins.image_show, vins.correct_point_cloud, vins.correct_Rs, vins.correct_Ps, box_in_trajectory);
            }
            cv::Mat tmp2 = vins.image_show;
            
            cv::Mat downOriginImage;
            cv::resize(image.t(), downOriginImage, cv::Size(200, 150));
            cv::cvtColor(downOriginImage, downOriginImage, CV_BGRA2RGB);
            cv::flip(downOriginImage,downOriginImage,0);
            cv::Mat imageROI;
            imageROI = tmp2(cv::Rect(10,COL - downOriginImage.rows- 10, downOriginImage.cols,downOriginImage.rows));
            cv::Mat mask;
            cv::cvtColor(downOriginImage, mask, CV_RGB2GRAY);
            downOriginImage.copyTo(imageROI, mask);
            
            
            cv::cvtColor(tmp2, image, CV_BGRA2BGR);
            cv::flip(image,tmp2,1);
            if (isNeedRotation)
                image = tmp2.t();
        }
    } else {
        // Not capturing, means not started yet
        cv::cvtColor(image, image, CV_BGRA2RGB);
        cv::flip(image,image,-1);
    }
}


/*
 Send imu data and visual data into VINS
 */
std::vector<std::pair<std::vector<IMUConstPtr>, ImageConstPtr>>
GetMeasurements()
{
    std::vector<std::pair<std::vector<IMUConstPtr>, ImageConstPtr>> measurements;
    while (true)
    {
        if (imuMessageBuffer.empty() || imageMessageBuffer.empty())
            return measurements;
        
        if (!(imuMessageBuffer.back()->header > imageMessageBuffer.front()->header))
        {
            NSLog(@"wait for imu, only should happen at the beginning");
            return measurements;
        }
        
        if (!(imuMessageBuffer.front()->header < imageMessageBuffer.front()->header))
        {
            NSLog(@"throw img, only should happen at the beginning");
            imageMessageBuffer.pop();
            continue;
        }
        ImageConstPtr imageMessage = imageMessageBuffer.front();
        imageMessageBuffer.pop();
        
        std::vector<IMUConstPtr> units;
        while (imuMessageBuffer.front()->header <= imageMessage->header)
        {
            units.emplace_back(imuMessageBuffer.front());
            imuMessageBuffer.pop();
        }
        //NSLog(@"IMU_buf = %d",IMUs.size());
        measurements.emplace_back(units, imageMessage);
    }
    return measurements;
}

vector<IMU_MSG_LOCAL> GetImuMeasurements(double header) {
    vector<IMU_MSG_LOCAL> imuMeasurements;
    static double lastHeader = -1;
    if(lastHeader < 0 || imuLocalMessageBuffer.empty()) {
        lastHeader = header;
        return imuMeasurements;
    }
    
    while(!imuLocalMessageBuffer.empty() && imuLocalMessageBuffer.front().header <= lastHeader) {
        imuLocalMessageBuffer.pop();
    }
    while(!imuLocalMessageBuffer.empty() && imuLocalMessageBuffer.front().header <= header) {
        imuMeasurements.emplace_back(imuLocalMessageBuffer.front());
        imuLocalMessageBuffer.pop();
    }
    lastHeader = header;
    return imuMeasurements;
}

void ProcessIMU(const IMUConstPtr &unit) {
    NSTimeInterval t = unit->header;
    if (currentTime < 0)
        currentTime = t;
    double dt = (t - currentTime);
    currentTime = t;
    
    double ba[]{0.0, 0.0, 0.0};
    double bg[]{0.0, 0.0, 0.0};
    
    double dx = unit->acc.x() - ba[0];
    double dy = unit->acc.y() - ba[1];
    double dz = unit->acc.z() - ba[2];
    
    double rx = unit->gyr.x() - bg[0];
    double ry = unit->gyr.y() - bg[1];
    double rz = unit->gyr.z() - bg[2];
    //NSLog(@"IMU %f, dt: %f, acc: %f %f %f, gyr: %f %f %f", t, dt, dx, dy, dz, rx, ry, rz);
    
    vins.processIMU(dt, Vector3d(dx, dy, dz), Vector3d(rx, ry, rz));
}


/*
 VINS thread: this thread tightly fuses the visual measurements and imu data and solves pose, velocity, IMU bias, 3D feature for all frame in WINNDOW
 If the newest frame is keyframe, then push it into keyframe database
 */
- (void)run {
    [self.condition lock];
    while (![[NSThread currentThread] isCancelled])
    {
        [self process];
        [NSThread sleepForTimeInterval:0.01];
        //        NSLog(@"[my_debug]:vins object size: %lu bytes", vins);
    }
    [self.condition unlock];
}


int kf_global_index;
bool enableGlobalOptimization = false;
- (void)process {
    std::vector<std::pair<std::vector<IMUConstPtr>, ImageConstPtr>> measurements;
    std::unique_lock<std::mutex> lk(bufferMutex);
    con.wait(lk, [&]{
        return (measurements = GetMeasurements()).size() != 0;
    });
    lk.unlock();
    
    waitingCount = measurements.size();
    for(auto &measurement : measurements) {
        for(auto &message : measurement.first) {
            ProcessIMU(message);
        }
        
        auto imageMessage = measurement.second;
        map<int, Vector3d> cloudsInfo = imageMessage->point_clouds;
        NSLog(@"Image timestamp = %lf", imageMessage->header);
        
        double header = imageMessage->header;
        vins.processImage(cloudsInfo, header, waitingCount);

        double time_now = [[NSProcessInfo processInfo] systemUptime];
        double time_vins = vins.Headers[WINDOW_SIZE];
        NSLog(@"vins delay %lf", time_now - time_vins);
        
        //update feature position for front-end
        if(vins.solver_flag == vins.NON_LINEAR && USE_PNP) {
            depthFeedbackMutex.lock();
            solvedResult.header = vins.Headers[WINDOW_SIZE - 1];
            solvedResult.Ba = vins.Bas[WINDOW_SIZE - 1];
            solvedResult.Bg = vins.Bgs[WINDOW_SIZE - 1];
            solvedResult.P = vins.correct_Ps[WINDOW_SIZE-1].cast<double>();
            solvedResult.R = vins.correct_Rs[WINDOW_SIZE-1].cast<double>();
            solvedResult.V = vins.Vs[WINDOW_SIZE - 1];
            
            solvedFeatures.clear();
            for (auto &it_per_id : vins.f_manager.feature) {
                it_per_id.used_num = it_per_id.feature_per_frame.size();
                if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
                    continue;
                if (it_per_id.solve_flag != 1)
                    continue;
                int imu_i = it_per_id.start_frame;
                Vector3d pts_i = it_per_id.feature_per_frame[0].point * it_per_id.estimated_depth;
                IMG_MSG_LOCAL tmp_feature;
                tmp_feature.id = it_per_id.feature_id;
                tmp_feature.position = vins.r_drift * vins.Rs[imu_i] * (vins.ric * pts_i + vins.tic) + vins.r_drift * vins.Ps[imu_i] + vins.t_drift;
                tmp_feature.track_num = (int)it_per_id.feature_per_frame.size();
                solvedFeatures.push_back(tmp_feature);
            }
            depthFeedbackMutex.unlock();
        }
        
        if(imageCacheEnabled) {
            //add state into vins buff for align with image
            if(vins.solver_flag == VINS::NON_LINEAR && enableShowing) {
                VINSDataCache dataCache;
                dataCache.header = vins.Headers[WINDOW_SIZE-1];
                dataCache.P = vins.correct_Ps[WINDOW_SIZE-1];
                dataCache.R = vins.correct_Rs[WINDOW_SIZE-1];
                vinsPool.push(dataCache);
            } else if(vins.failure_occur == true) {
                vins.drawresult.change_color = true;
                vins.drawresult.indexs.push_back((int)vins.drawresult.pose.size());
                segmentationIndex++;
                database.max_seg_index++;
                database.cur_seg_index = database.max_seg_index;
                
                while(!vinsPool.empty()) {
                    vinsPool.pop();
                }
            }
        }
        /**
         *** start build keyframe database for loop closure
         **/
        if(ENABLE_LOOP_CLOSURE) {
            static bool first_frame = true;
            if(vins.solver_flag != vins.NON_LINEAR) {
                first_frame = true;
            }
            if(vins.marginalization_flag == vins.MARGIN_OLD && vins.solver_flag == vins.NON_LINEAR && !imageBufferLoop.empty()) {
                first_frame = false;
                if(!first_frame && keyFrameFrequence % LOOP_FREQ == 0) {
                    keyFrameFrequence = 0;
                    /**
                     ** save the newest keyframe to the keyframe database
                     ** only need to save the pose to the keyframe database
                     **/
                    Vector3d T_w_i = vins.Ps[WINDOW_SIZE - 2];
                    Matrix3d R_w_i = vins.Rs[WINDOW_SIZE - 2];
                    imageBufferLoopMutex.lock();
                    while(!imageBufferLoop.empty() && imageBufferLoop.front().second < vins.Headers[WINDOW_SIZE - 2]) {
                        imageBufferLoop.pop();
                    }
                    //assert(vins.Headers[WINDOW_SIZE - 2] == imageBufferLoop.front().second);
                    if(vins.Headers[WINDOW_SIZE - 2] == imageBufferLoop.front().second) {
                        const char *pattern_file = [[[NSBundle bundleForClass:[self class]] pathForResource:@"brief_pattern" ofType:@"yml"] cStringUsingEncoding:[NSString defaultCStringEncoding]];
                        KeyFrame* keyframe = new KeyFrame(vins.Headers[WINDOW_SIZE - 2], globalFrameCount, T_w_i, R_w_i, imageBufferLoop.front().first, pattern_file, database.cur_seg_index);
                        keyframe->setExtrinsic(vins.tic, vins.ric);
/*
** we still need save the measurement to the keyframe(not database) for add connection with looped old pose
** and save the pointcloud to the keyframe for reprojection search correspondance
*/
                        keyframe->buildKeyFrameFeatures(vins);
                        database.add(keyframe);
                        erasingIndices.clear();
                        database.resample(erasingIndices);
                        
                        globalFrameCount++;
                    }
                    imageBufferLoopMutex.unlock();
                } else {
                    first_frame = false;
                }
                // update loop info
                // check if the closure is wrong.
                for (int i = 0; i < WINDOW_SIZE; i++) {
                    if(vins.Headers[i] == vins.front_pose.header) {
                        KeyFrame* currentKeyFrame = database.getKeyframe(vins.front_pose.cur_index);
                        if(currentKeyFrame  == NULL) {
                            break;
                        }
                        if (abs(vins.front_pose.relative_yaw) > 30.0 || vins.front_pose.relative_t.norm() > 10.0) {
                            NSLog(@"Wrong loop");
                            currentKeyFrame->removeLoop();
                            break;
                        }
                        currentKeyFrame->updateLoopConnection(vins.front_pose.relative_t, vins.front_pose.relative_q, vins.front_pose.relative_yaw);
                        break;
                    }
                }
                /*
                 ** update the keyframe pose when this frame slides out the window and optimize loop graph
                 */
                int search_cnt = 0;
                for(int i = 0; i < database.size(); i++) {
                    search_cnt++;
                    KeyFrame* frame = database.getLastKeyframe(i);
                    if(frame->header == vins.Headers[0]) {
                        frame->updateOriginPose(vins.Ps[0], vins.Rs[0]);
                        //update edge
                        // if loop happens in this frame, updatepose  graph;
                        if (frame->has_loop) {
                            kf_global_index = frame->global_index;
                            enableGlobalOptimization = true;
                        }
                        break;
                    } else {
                        if(search_cnt > 2 * WINDOW_SIZE) {
                            break;
                        }
                    }
                }
                keyFrameFrequence++;
            }
        }
        waitingCount--;
        
        //finish solve one frame
        [self performSelectorOnMainThread:@selector(showInputView) withObject:nil waitUntilDone:YES];
        
    }
}


/*
 Loop detection thread: this thread detect loop for newest keyframe and retrieve features
 */
- (void)runLoopThread {
    if(ENABLE_LOOP_CLOSURE && loopClosure == NULL) {
        NSLog(@"loop start load voc");
        //        TS(load_voc);
        const char *voc_file = [[[NSBundle bundleForClass:[self class]] pathForResource:@"brief_k10L6" ofType:@"bin"]
                                cStringUsingEncoding:[NSString defaultCStringEncoding]];
        loopClosure = SharedRef<LoopClosure>(new LoopClosure(voc_file, COL, ROW));
        //        TE(load_voc);
        NSLog(@"loop load voc finish");
    }
    while(![[NSThread currentThread] isCancelled] ) {
        if(!ENABLE_LOOP_CLOSURE) {
            [NSThread sleepForTimeInterval:0.5];
            continue;
        }
        if(!erasingIndices.empty() && loopClosure != NULL) {
            loopClosure->eraseIndex(erasingIndices);
        }
        
        bool loop_succ = false;
        if (loopCheckedCount < globalFrameCount) {
            KeyFrame* cur_kf = database.getLastKeyframe();
            //assert(loopCheckedCount == cur_kf->global_index);
            loopCheckedCount++;
            cur_kf->check_loop = 1;
            
            cv::Mat current_image;
            current_image = cur_kf->image;
            
            std::vector<cv::Point2f> measurements_old;
            std::vector<cv::Point2f> measurements_old_norm;
            std::vector<cv::Point2f> measurements_cur;
            std::vector<int> features_id;
            std::vector<cv::Point2f> measurements_cur_origin = cur_kf->measurements;
            
            vector<cv::Point2f> cur_pts;
            vector<cv::Point2f> old_pts;
            cur_kf->extractBrief(current_image);
            printf("loop extract %lu feature\n", cur_kf->keypoints.size());
            loop_succ = loopClosure->startLoopClosure(cur_kf->keypoints, cur_kf->descriptors, cur_pts, old_pts, oldIndex);
            if(loop_succ && oldIndex != -1) {
                KeyFrame* old_kf = database.getKeyframe(oldIndex);
                if (old_kf == NULL) {
                    printf("NO such frame in database\n");
                    continue;
                    //                    assert(false);
                }
                printf("loop succ with %drd image\n", oldIndex);
                //                assert(oldIndex!=-1);
                
                Vector3d T_w_i_old;
                Matrix3d R_w_i_old;
                
                old_kf->getPose(T_w_i_old, R_w_i_old);
                cur_kf->findConnectionWithOldFrame(old_kf, cur_pts, old_pts,
                                                   measurements_old, measurements_old_norm);
                measurements_cur = cur_kf->measurements;
                features_id = cur_kf->features_id;
                
                if(measurements_old_norm.size()>MIN_LOOP_NUM) {
                    Quaterniond Q_loop_old(R_w_i_old);
                    RetriveData retrive_data;
                    retrive_data.cur_index = cur_kf->global_index;
                    retrive_data.header = cur_kf->header;
                    retrive_data.P_old = T_w_i_old;
                    retrive_data.Q_old = Q_loop_old;
                    retrive_data.use = true;
                    retrive_data.measurements = measurements_old_norm;
                    retrive_data.features_ids = features_id;
                    vins.retrive_pose_data = (retrive_data);
                    printf("loop push\n");
                    
                    //cout << "old pose " << T_w_i_old.transpose() << endl;
                    //cout << "refinded pose " << T_w_i_refine.transpose() << endl;
                    // add loop edge in current frame
                    cur_kf->detectLoop(oldIndex);
                    database.addLoop(oldIndex);
                    old_kf->is_looped = 1;
                }
            }
            cur_kf->image.release();
        }
        
        if(loop_succ) {
            [NSThread sleepForTimeInterval:2.0];
        }
        [NSThread sleepForTimeInterval:0.05];
    }
    //[self process_loop_detection];
}

/*
 GLobal Pose graph thread: optimize global pose graph based on realative pose from vins and update the keyframe database
 */
- (void)runGlobalLoopThread {
    while (![[NSThread currentThread] isCancelled])
    {
        if(enableGlobalOptimization)
        {
            enableGlobalOptimization = false;
            
            database.optimize4DoFLoopPoseGraph(kf_global_index, loop_correct_t, loop_correct_r);
            vins.t_drift = loop_correct_t;
            vins.r_drift = loop_correct_r;
            
            [NSThread sleepForTimeInterval:1.17];
        }
        [NSThread sleepForTimeInterval:0.03];
    }
}

/*
 Z^
 |   /Y
 |  /
 | /
 |/--------->X
 IMU data process and interploration
 
 */
SharedRef<IMUMessage> currentAccelerator(new IMUMessage());
vector<IMUMessage> gyro_buf;  // for Interpolation

- (void)imuStartUpdate {
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
        if(imuPrepareCount<10) {
            imuPrepareCount++;
        }
        shared_ptr<IMUMessage> acceleratorMessage(new IMUMessage());
        acceleratorMessage->header = latestAcc.timestamp;
        acceleratorMessage->acc << -latestAcc.acceleration.x * GRAVITY,
        -latestAcc.acceleration.y * GRAVITY,
        -latestAcc.acceleration.z * GRAVITY;
        currentAccelerator = acceleratorMessage;
    }];
    
    [self.motionManager startGyroUpdatesToQueue:[NSOperationQueue currentQueue] withHandler:^(CMGyroData *latestGyro, NSError *error) {
        //The time stamp is the amount of time in seconds since the device booted.
        NSTimeInterval header = latestGyro.timestamp;
        if(header <= 0 || imuPrepareCount < 10) {
            return;
        }
        IMUMessage gyro_msg;
        gyro_msg.header = header;
        gyro_msg.gyr << latestGyro.rotationRate.x,
        latestGyro.rotationRate.y,
        latestGyro.rotationRate.z;
        
        if(gyro_buf.size() == 0) {
            gyro_buf.push_back(gyro_msg);
            gyro_buf.push_back(gyro_msg);
            return;
        } else {
            gyro_buf[0] = gyro_buf[1];
            gyro_buf[1] = gyro_msg;
        }
        //interpolation
        shared_ptr<IMUMessage> imu_msg(new IMUMessage());
        if(currentAccelerator->header >= gyro_buf[0].header && currentAccelerator->header < gyro_buf[1].header) {
            imu_msg->header = currentAccelerator->header;
            imu_msg->acc = currentAccelerator->acc;
            imu_msg->gyr = gyro_buf[0].gyr + (currentAccelerator->header - gyro_buf[0].header)*(gyro_buf[1].gyr - gyro_buf[0].gyr)/(gyro_buf[1].header - gyro_buf[0].header);
            //printf("imu gyro update %lf %lf %lf\n", gyro_buf[0].header, imu_msg->header, gyro_buf[1].header);
            //printf("imu inte update %lf %lf %lf %lf\n", imu_msg->header, gyro_buf[0].gyr.x(), imu_msg->gyr.x(), gyro_buf[1].gyr.x());
        } else {
            NSLog(@"imu error %lf %lf %lf", gyro_buf[0].header, currentAccelerator->header, gyro_buf[1].header);
            return;
        }
        
        imuLateastTime = imu_msg->header;
        
        //img_msg callback
        if(USE_PNP) {
            IMU_MSG_LOCAL imu_msg_local;
            imu_msg_local.header = imu_msg->header;
            imu_msg_local.acc = imu_msg->acc;
            imu_msg_local.gyr = imu_msg->  gyr;
            
            imuFeedbackMutex.lock();
            imuLocalMessageBuffer.push(imu_msg_local);
            imuFeedbackMutex.unlock();
        }
//        NSLog(@"[thread_debug] buffer mutex lock??");
        bufferMutex.lock();
        imuMessageBuffer.push(imu_msg);
        bufferMutex.unlock();
//        NSLog(@"[thread_debug] buffer mutex unlock??");
        con.notify_one();
    }];
}

- (void)showInputView {
    NSString *stringView;
    static bool finish_init = false;
    if(vins.solver_flag != vins.NON_LINEAR) {
        finish_init = false;
        stringView = [NSString stringWithFormat:@"Initializing: %d%%", vins.initProgress];
        [self.progressLabel setText:stringView];
        
        [self.progressLabel setHidden:NO];
        [self.indicator setHidden:NO];
    } else {
        if(finish_init == false) {
            //Hide init UI
            [self.progressLabel setHidden:YES];
            [self.indicator setHidden:YES];
            
            enableShowing = true;
            finish_init = true;
        }
    }
}


- (void)handlePan:(UIPanGestureRecognizer*) recognizer {
    if (!ui_main) {
        //        CGPoint translation = [recognizer translationInView:self.view];
        CGFloat velocityX = [recognizer velocityInView:self.view].x;
        CGFloat velocityY = [recognizer velocityInView:self.view].y;
        //recognizer.view.center = CGPointMake(recognizer.view.center.x + translation.x,
        static CGFloat vx_last = 0;
        static CGFloat vy_last = 0;
        
        CGFloat vx_smooth = 0.5*velocityX + 0.5*vx_last;
        CGFloat vy_smooth = 0.5*velocityY + 0.5*vy_last;
        vx_last = vx_smooth;
        vy_last = vy_smooth;
        if(recognizer.numberOfTouches == 2) {
            vins.drawresult.Y0 += vx_smooth / 100.0;
            vins.drawresult.X0 += vy_smooth / 100.0;
        } else {
            vins.drawresult.theta += vy_smooth / 100.0;
            vins.drawresult.theta = fmod(vins.drawresult.theta, 360.0);
            vins.drawresult.phy += vx_smooth / 100.0;
            vins.drawresult.phy = fmod(vins.drawresult.phy, 360.0);
        }
        
        vins.drawresult.change_view_manualy = true;
    } else {
        //        CGPoint translation = [recognizer translationInView:self.view];
        CGFloat velocityX = [recognizer velocityInView:self.view].x;
        CGFloat velocityY = [recognizer velocityInView:self.view].y;
        //CGFloat translationX =
        //CGFloat translationY = [recognizer translationInView:self.view].y;
        //NSLog(@"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!pipikk test x: %f y: %f", translationX, translationY);
        //NSLog(@"!!!!!!!!!!!!!!!!!!!!!!%f  %f", imageView.frame.size.height, imageView.frame.size.width);
        CGPoint point = [recognizer locationInView:self.view];
        //NSLog(@"X Location: %f", point.x);
        //NSLog(@"Y Location: %f",point.y);
        
        //recognizer.view.center = CGPointMake(recognizer.view.center.x + translation.x,
        static CGFloat vx_lastAR = 0;
        static CGFloat vy_lastAR = 0;
        
        CGFloat vx_smooth = 0.5*velocityX + 0.5*vx_lastAR;
        CGFloat vy_smooth = 0.5*velocityY + 0.5*vy_lastAR;
        vx_lastAR = vx_smooth;
        vy_lastAR = vy_smooth;
        if(recognizer.numberOfTouches == 2) {
            vins.drawresult.Y0AR += vx_smooth/100.0;
            vins.drawresult.X0AR += vy_smooth/100.0;
            
            vins.drawresult.locationXT2 = point.x * 640.0 / self.imageView.frame.size.width;
            vins.drawresult.locationYT2 = point.y * 480.0 / self.imageView.frame.size.height;
            
            vins.drawresult.finger_s = 0;
            vins.drawresult.finger_p = 0;
            if ((vins.drawresult.finger_d ++) > 7) {
                vins.drawresult.finger_state = 2;
            }
        } else {
            vins.drawresult.thetaAR += vy_smooth/100.0;
            //vins.drawresult.thetaAR = fmod(vins.drawresult.thetaAR, 360.0);
            vins.drawresult.phyAR += vx_smooth/100.0;
            //vins.drawresult.phyAR = fmod(vins.drawresult.phyAR, 360.0);
            
            vins.drawresult.locationX = point.x * 640.0 / self.imageView.frame.size.width;
            vins.drawresult.locationY = point.y * 480.0 / self.imageView.frame.size.height;
            
            vins.drawresult.finger_d = 0;
            vins.drawresult.finger_p = 0;
            if ((vins.drawresult.finger_s ++) > 7)
                vins.drawresult.finger_state = 1;
        }
    }
}

- (void)handlePinch:(UIPinchGestureRecognizer*) recognizer {
    if (!ui_main) {
        vins.drawresult.change_view_manualy = true;
        if(vins.drawresult.radius > 5 || recognizer.velocity < 0) {
            vins.drawresult.radius -= recognizer.velocity * 0.5;
        } else {
            vins.drawresult.Fx += recognizer.velocity * 15;
            if(vins.drawresult.Fx < 50)
                vins.drawresult.Fx = 50;
            vins.drawresult.Fy += recognizer.velocity * 15;
            if(vins.drawresult.Fy < 50)
                vins.drawresult.Fy = 50;
        }
    } else {
        vins.drawresult.finger_s = 0;
        vins.drawresult.finger_d = 0;
        if ((vins.drawresult.finger_p ++) > 7) {
            vins.drawresult.finger_state = 3;
        }
        
        CGPoint point = [recognizer locationInView:self.view];
        vins.drawresult.locationXP = point.x * 640.0 / self.imageView.frame.size.width;
        vins.drawresult.locationYP = point.y * 480.0 / self.imageView.frame.size.height;
        vins.drawresult.radiusAR -= recognizer.velocity * 0.5;
    }
    
}


- (void)viewDidAppear:(BOOL)animated {
    [super viewDidAppear:animated];
}

- (void)viewDidDisappear:(BOOL)animated {
    [super viewDidDisappear:animated];
    if (isCapturing) {
        [self.videoCamera stop];
    }
    [self.mainLoop cancel];
    [self.draw cancel];
}

- (void)dealloc {
    [self.motionManager stopAccelerometerUpdates];
    [self.motionManager stopDeviceMotionUpdates];
    [self.motionManager stopGyroUpdates];
    [self.motionManager stopMagnetometerUpdates];
}


- (void)showMessage:(NSString *)message withTitle:(NSString *)title {
    UIAlertController *alertDevice = [UIAlertController alertControllerWithTitle:title message:message preferredStyle:UIAlertControllerStyleAlert];
    UIAlertAction *okAction = [UIAlertAction actionWithTitle:@"OK" style:UIAlertActionStyleDefault handler:nil];
    [alertDevice addAction:okAction];
    [self presentViewController:alertDevice animated:YES completion:nil];
}

@end

