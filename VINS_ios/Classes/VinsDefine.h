#import <UIKit/UIKit.h>
#import <opencv2/imgcodecs/ios.h>
#import <opencv2/videoio/cap_ios.h>
#import <eigen3/Eigen/Dense>
#import <queue>
#import <map>

namespace Vins {
    
    template<typename T> using SharedRef = std::shared_ptr<T>;
    template<typename T> using Array = std::vector<T>;
    template<typename KeyType, typename ValueType> using Dictionary = std::map<KeyType, ValueType>;


    struct IMUMessage {
        NSTimeInterval header;
        // Accelerator
        Eigen::Vector3d acc;
        //
        Eigen::Vector3d gyr;
    };

    struct ImageMessage {
        NSTimeInterval header;
        Dictionary<int, Eigen::Vector3d> point_clouds;
    };

    struct ImageData {
        NSTimeInterval header;
        UIImage *image;
    };

    struct ImageDataCache {
        NSTimeInterval header;
        cv::Mat equImage;
        UIImage *image;
    };

    struct VINSDataCache {
        NSTimeInterval header;
        Eigen::Vector3f P;
        Eigen::Matrix3f R;
    };

    typedef SharedRef <IMUMessage const> IMUConstPtr;
    typedef SharedRef <ImageMessage const> ImageConstPtr;
}
