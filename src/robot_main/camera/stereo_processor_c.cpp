#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <limits>
#include <functional>

// Conditional CUDA includes - only if available
#ifdef OPENCV_CUDA_AVAILABLE
#include <opencv2/cudaarithm.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/cudastereo.hpp>
#endif

namespace stereo_pointcloud_node
{

class StereoPointCloudGenerator
{
public:
    StereoPointCloudGenerator(ros::NodeHandle& nh) : nh_(nh), it_(nh_), use_cuda_(false)
    {
        // Check CUDA availability
        checkCudaAvailability();
        
        // Subscribe to CameraInfo topics
        left_info_sub_ = nh_.subscribe("/stereo/left/camera_info", 1, 
            &StereoPointCloudGenerator::leftInfoCallback, this);
        right_info_sub_ = nh_.subscribe("/stereo/right/camera_info", 1, 
            &StereoPointCloudGenerator::rightInfoCallback, this);

        ROS_INFO("Waiting for camera calibration info...");

        // Image subscribers
        left_img_sub_.subscribe(it_, "/stereo/left/image_raw", 1);
        right_img_sub_.subscribe(it_, "/stereo/right/image_raw", 1);

        // Synchronizer with more conservative settings
        sync_ = std::make_shared<message_filters::Synchronizer<ApproximateSyncPolicy>>(
            ApproximateSyncPolicy(5), left_img_sub_, right_img_sub_);
        sync_->registerCallback(std::bind(&StereoPointCloudGenerator::imageCallback, 
            this, std::placeholders::_1, std::placeholders::_2));

        // Point cloud publisher
        pointcloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/stereo/points2", 1);

        // Initialize stereo matcher with Jetson Nano optimized parameters
        setupStereoMatcher();
    }

private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    
    // Camera info
    ros::Subscriber left_info_sub_;
    ros::Subscriber right_info_sub_;
    sensor_msgs::CameraInfoConstPtr left_info_;
    sensor_msgs::CameraInfoConstPtr right_info_;
    bool left_info_received_ = false;
    bool right_info_received_ = false;
    bool calibration_ready_ = false;
    
    // Image synchronization
    image_transport::SubscriberFilter left_img_sub_;
    image_transport::SubscriberFilter right_img_sub_;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> ApproximateSyncPolicy;
    std::shared_ptr<message_filters::Synchronizer<ApproximateSyncPolicy>> sync_;
    
    // Publisher
    ros::Publisher pointcloud_pub_;
    
    // Processing flags and objects
    bool use_cuda_;
    cv::Ptr<cv::StereoSGBM> stereo_sgbm_cpu_;
    
#ifdef OPENCV_CUDA_AVAILABLE
    cv::Ptr<cv::cuda::StereoSGBM> stereo_sgbm_cuda_;
    cv::cuda::GpuMat map1_left_gpu_, map2_left_gpu_;
    cv::cuda::GpuMat map1_right_gpu_, map2_right_gpu_;
#endif
    
    // CPU rectification maps (always available as fallback)
    cv::Mat map1_left_cpu_, map2_left_cpu_;
    cv::Mat map1_right_cpu_, map2_right_cpu_;
    
    // Calibration data
    cv::Mat Q_;
    cv::Size image_size_;
    double baseline_;
    
    void checkCudaAvailability()
    {
#ifdef OPENCV_CUDA_AVAILABLE
        try {
            int cuda_devices = cv::cuda::getCudaEnabledDeviceCount();
            if (cuda_devices > 0) {
                use_cuda_ = true;
                ROS_INFO("CUDA available with %d device(s). Using GPU acceleration.", cuda_devices);
            } else {
                use_cuda_ = false;
                ROS_WARN("No CUDA devices found. Using CPU processing.");
            }
        } catch (const cv::Exception& e) {
            use_cuda_ = false;
            ROS_WARN("CUDA check failed: %s. Using CPU processing.", e.what());
        }
#else
        use_cuda_ = false;
        ROS_INFO("OpenCV compiled without CUDA support. Using CPU processing.");
#endif
    }
    
    void setupStereoMatcher()
    {
        // Jetson Nano optimized parameters
        int minDisparity = 0;
        int numDisparities = 64;  // Reduced for performance
        int blockSize = 7;        // Smaller block size
        int P1 = 8 * blockSize * blockSize;
        int P2 = 32 * blockSize * blockSize;
        int disp12MaxDiff = 1;
        int preFilterCap = 63;
        int uniquenessRatio = 10;
        int speckleWindowSize = 50;  // Reduced
        int speckleRange = 16;       // Reduced
        int mode = cv::StereoSGBM::MODE_SGBM;
        
        // Always create CPU version as fallback
        stereo_sgbm_cpu_ = cv::StereoSGBM::create(
            minDisparity, numDisparities, blockSize, P1, P2, 
            disp12MaxDiff, preFilterCap, uniquenessRatio, 
            speckleWindowSize, speckleRange, mode);
            
#ifdef OPENCV_CUDA_AVAILABLE
        if (use_cuda_) {
            try {
                stereo_sgbm_cuda_ = cv::cuda::StereoSGBM::create(
                    minDisparity, numDisparities, blockSize, P1, P2,
                    disp12MaxDiff, preFilterCap, uniquenessRatio,
                    speckleWindowSize, speckleRange, mode);
                ROS_INFO("CUDA StereoSGBM initialized successfully.");
            } catch (const cv::Exception& e) {
                ROS_WARN("Failed to create CUDA StereoSGBM: %s. Falling back to CPU.", e.what());
                use_cuda_ = false;
            }
        }
#endif
    }
    
    void leftInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg)
    {
        if (!left_info_received_) {
            left_info_ = msg;
            left_info_received_ = true;
            ROS_INFO("Received left camera info.");
            checkAndSetupCalibration();
        }
    }
    
    void rightInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg)
    {
        if (!right_info_received_) {
            right_info_ = msg;
            right_info_received_ = true;
            ROS_INFO("Received right camera info.");
            checkAndSetupCalibration();
        }
    }
    
    void checkAndSetupCalibration()
    {
        if (left_info_received_ && right_info_received_ && !calibration_ready_) {
            ROS_INFO("Both camera info messages received. Setting up calibration.");
            
            // Validate camera resolutions match
            if (left_info_->width != right_info_->width || 
                left_info_->height != right_info_->height) {
                ROS_ERROR("Camera resolutions don't match! Left: %dx%d, Right: %dx%d", 
                    left_info_->width, left_info_->height,
                    right_info_->width, right_info_->height);
                return;
            }
            
            image_size_ = cv::Size(left_info_->width, left_info_->height);
            
            // Safely extract matrices by copying data
            cv::Mat K_left = cv::Mat::zeros(3, 3, CV_64F);
            cv::Mat K_right = cv::Mat::zeros(3, 3, CV_64F);
            cv::Mat D_left = cv::Mat::zeros(1, left_info_->D.size(), CV_64F);
            cv::Mat D_right = cv::Mat::zeros(1, right_info_->D.size(), CV_64F);
            cv::Mat R_left = cv::Mat::zeros(3, 3, CV_64F);
            cv::Mat R_right = cv::Mat::zeros(3, 3, CV_64F);
            cv::Mat P_left = cv::Mat::zeros(3, 4, CV_64F);
            cv::Mat P_right = cv::Mat::zeros(3, 4, CV_64F);
            
            // Copy data safely
            std::copy(left_info_->K.begin(), left_info_->K.end(), K_left.ptr<double>());
            std::copy(right_info_->K.begin(), right_info_->K.end(), K_right.ptr<double>());
            std::copy(left_info_->D.begin(), left_info_->D.end(), D_left.ptr<double>());
            std::copy(right_info_->D.begin(), right_info_->D.end(), D_right.ptr<double>());
            std::copy(left_info_->R.begin(), left_info_->R.end(), R_left.ptr<double>());
            std::copy(right_info_->R.begin(), right_info_->R.end(), R_right.ptr<double>());
            std::copy(left_info_->P.begin(), left_info_->P.end(), P_left.ptr<double>());
            std::copy(right_info_->P.begin(), right_info_->P.end(), P_right.ptr<double>());
            
            // Calculate baseline from projection matrices
            double fx_left = P_left.at<double>(0, 0);
            double fx_right = P_right.at<double>(0, 0);
            double Tx = P_right.at<double>(0, 3);
            
            if (std::abs(fx_right) < 1e-6) {
                ROS_ERROR("Invalid focal length in right camera projection matrix");
                return;
            }
            
            baseline_ = -Tx / fx_right;
            if (baseline_ <= 0) {
                ROS_ERROR("Invalid baseline calculated: %f. Check camera calibration.", baseline_);
                return;
            }
            
            ROS_INFO("Calculated baseline: %f meters", baseline_);
            
            // Create CPU rectification maps (always available)
            cv::initUndistortRectifyMap(K_left, D_left, R_left, P_left, image_size_,
                                      CV_16SC2, map1_left_cpu_, map2_left_cpu_);
            cv::initUndistortRectifyMap(K_right, D_right, R_right, P_right, image_size_,
                                      CV_16SC2, map1_right_cpu_, map2_right_cpu_);
            
#ifdef OPENCV_CUDA_AVAILABLE
            // Create GPU rectification maps if CUDA is available
            if (use_cuda_) {
                try {
                    map1_left_gpu_.upload(map1_left_cpu_);
                    map2_left_gpu_.upload(map2_left_cpu_);
                    map1_right_gpu_.upload(map1_right_cpu_);
                    map2_right_gpu_.upload(map2_right_cpu_);
                    ROS_INFO("GPU rectification maps uploaded successfully.");
                } catch (const cv::Exception& e) {
                    ROS_WARN("Failed to upload rectification maps to GPU: %s", e.what());
                    use_cuda_ = false;
                }
            }
#endif
            
            // Construct Q matrix for 3D reprojection
            double fx = P_left.at<double>(0, 0);
            double fy = P_left.at<double>(1, 1);
            double cx = P_left.at<double>(0, 2);
            double cy = P_left.at<double>(1, 2);
            
            Q_ = (cv::Mat_<double>(4, 4) <<
                  1, 0, 0, -cx,
                  0, 1, 0, -cy,
                  0, 0, 0, fx,
                  0, 0, -1.0/baseline_, 0);
            
            calibration_ready_ = true;
            ROS_INFO("Calibration setup complete. Ready to process stereo images.");
        }
    }
    
    cv::Mat imageMsgToCvMat(const sensor_msgs::ImageConstPtr& img_msg)
    {
        if (img_msg->encoding != "mono8" && img_msg->encoding != "bgr8" && img_msg->encoding != "rgb8") {
            ROS_ERROR_THROTTLE(1.0, "Unsupported image encoding: %s", img_msg->encoding.c_str());
            return cv::Mat();
        }
        
        int cv_type = CV_8UC1;
        if (img_msg->encoding == "bgr8" || img_msg->encoding == "rgb8") {
            cv_type = CV_8UC3;
        }
        
        cv::Mat img_mat(img_msg->height, img_msg->width, cv_type, 
                       const_cast<unsigned char*>(img_msg->data.data()), img_msg->step);
        
        // Convert to grayscale if needed
        if (cv_type == CV_8UC3) {
            cv::Mat gray;
            cv::cvtColor(img_mat, gray, img_msg->encoding == "rgb8" ? cv::COLOR_RGB2GRAY : cv::COLOR_BGR2GRAY);
            return gray.clone();
        }
        
        return img_mat.clone(); // Always clone to ensure data persistence
    }
    
    void imageCallback(const sensor_msgs::ImageConstPtr& left_msg, 
                      const sensor_msgs::ImageConstPtr& right_msg)
    {
        if (!calibration_ready_) {
            ROS_WARN_THROTTLE(2.0, "Waiting for camera calibration...");
            return;
        }
        
        // Convert images
        cv::Mat left_image = imageMsgToCvMat(left_msg);
        cv::Mat right_image = imageMsgToCvMat(right_msg);
        
        if (left_image.empty() || right_image.empty()) {
            ROS_ERROR("Failed to convert image messages");
            return;
        }
        
        // Validate image sizes
        if (left_image.size() != image_size_ || right_image.size() != image_size_) {
            ROS_WARN_THROTTLE(1.0, "Image size mismatch. Expected %dx%d", 
                image_size_.width, image_size_.height);
            return;
        }
        
        cv::Mat disparity;
        if (use_cuda_) {
            disparity = processWithCuda(left_image, right_image);
        } else {
            disparity = processWithCpu(left_image, right_image);
        }
        
        if (disparity.empty()) {
            ROS_ERROR("Failed to compute disparity");
            return;
        }
        
        // Generate and publish point cloud
        generateAndPublishPointCloud(disparity, left_image, left_msg);
    }
    
    cv::Mat processWithCpu(const cv::Mat& left_image, const cv::Mat& right_image)
    {
        // Rectify images
        cv::Mat left_rect, right_rect;
        cv::remap(left_image, left_rect, map1_left_cpu_, map2_left_cpu_, cv::INTER_LINEAR);
        cv::remap(right_image, right_rect, map1_right_cpu_, map2_right_cpu_, cv::INTER_LINEAR);
        
        // Compute disparity
        cv::Mat disparity;
        stereo_sgbm_cpu_->compute(left_rect, right_rect, disparity);
        
        return disparity;
    }
    
#ifdef OPENCV_CUDA_AVAILABLE
    cv::Mat processWithCuda(const cv::Mat& left_image, const cv::Mat& right_image)
    {
        try {
            // Upload to GPU
            cv::cuda::GpuMat left_gpu(left_image);
            cv::cuda::GpuMat right_gpu(right_image);
            
            // Rectify on GPU
            cv::cuda::GpuMat left_rect_gpu, right_rect_gpu;
            cv::cuda::remap(left_gpu, left_rect_gpu, map1_left_gpu_, map2_left_gpu_, cv::INTER_LINEAR);
            cv::cuda::remap(right_gpu, right_rect_gpu, map1_right_gpu_, map2_right_gpu_, cv::INTER_LINEAR);
            
            // Compute disparity on GPU
            cv::cuda::GpuMat disparity_gpu;
            stereo_sgbm_cuda_->compute(left_rect_gpu, right_rect_gpu, disparity_gpu);
            
            // Download result
            cv::Mat disparity;
            disparity_gpu.download(disparity);
            
            return disparity;
        } catch (const cv::Exception& e) {
            ROS_WARN("CUDA processing failed: %s. Falling back to CPU.", e.what());
            use_cuda_ = false;
            return processWithCpu(left_image, right_image);
        }
    }
#else
    cv::Mat processWithCuda(const cv::Mat& left_image, const cv::Mat& right_image)
    {
        // Fallback to CPU if CUDA not available
        return processWithCpu(left_image, right_image);
    }
#endif
    
    void generateAndPublishPointCloud(const cv::Mat& disparity, const cv::Mat& left_image,
                                    const sensor_msgs::ImageConstPtr& left_msg)
    {
        // Convert disparity to float
        cv::Mat disparity_float;
        disparity.convertTo(disparity_float, CV_32F, 1.0/16.0);
        
        // Reproject to 3D
        cv::Mat points_3d;
        cv::reprojectImageTo3D(disparity_float, points_3d, Q_, true);
        
        // Create point cloud
        pcl::PointCloud<pcl::PointXYZRGB> cloud;
        cloud.header.frame_id = left_msg->header.frame_id;
        cloud.height = points_3d.rows;
        cloud.width = points_3d.cols;
        cloud.is_dense = false;
        
        size_t num_points = static_cast<size_t>(cloud.width) * cloud.height;
        
        // Check memory constraints
        if (num_points > 2000000) {
            ROS_WARN_THROTTLE(5.0, "Large point cloud (%zu points). Consider reducing image resolution.", num_points);
        }
        
        cloud.points.resize(num_points);
        
        // Fill point cloud
        size_t valid_points = 0;
        for (int y = 0; y < points_3d.rows; ++y) {
            for (int x = 0; x < points_3d.cols; ++x) {
                cv::Vec3f point = points_3d.at<cv::Vec3f>(y, x);
                pcl::PointXYZRGB& pcl_point = cloud.points[y * points_3d.cols + x];
                
                // Filter valid points
                if (std::isfinite(point[0]) && std::isfinite(point[1]) && 
                    std::isfinite(point[2]) && point[2] > 0.1f && point[2] < 50.0f) {
                    
                    pcl_point.x = point[0];
                    pcl_point.y = point[1];
                    pcl_point.z = point[2];
                    
                    // Set color from left image
                    uint8_t intensity = left_image.at<uint8_t>(y, x);
                    uint32_t rgb = (static_cast<uint32_t>(intensity) << 16 |
                                   static_cast<uint32_t>(intensity) << 8 |
                                   static_cast<uint32_t>(intensity));
                    pcl_point.rgb = *reinterpret_cast<float*>(&rgb);
                    
                    valid_points++;
                } else {
                    pcl_point.x = std::numeric_limits<float>::quiet_NaN();
                    pcl_point.y = std::numeric_limits<float>::quiet_NaN();
                    pcl_point.z = std::numeric_limits<float>::quiet_NaN();
                    pcl_point.rgb = 0;
                }
            }
        }
        
        // Publish point cloud
        sensor_msgs::PointCloud2 output_msg;
        pcl::toROSMsg(cloud, output_msg);
        output_msg.header.stamp = left_msg->header.stamp;
        output_msg.header.frame_id = left_msg->header.frame_id;
        
        pointcloud_pub_.publish(output_msg);
        
        ROS_INFO_THROTTLE(1.0, "Published point cloud: %zu total points, %zu valid points", 
                         num_points, valid_points);
    }
};

} // namespace stereo_pointcloud_node

int main(int argc, char** argv)
{
    ros::init(argc, argv, "stereo_processor");
    ros::NodeHandle nh;
    
    try {
        stereo_pointcloud_node::StereoPointCloudGenerator generator(nh);
        ros::spin();
    } catch (const std::exception& e) {
        ROS_FATAL("Exception in stereo processor: %s", e.what());
        return -1;
    }
    
    return 0;
}