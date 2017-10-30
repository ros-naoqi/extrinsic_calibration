#ifndef CALIBRATOR_HPP
#define CALIBRATOR_HPP

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <tf/transform_listener.h>

#include <mutex>

#include "calibration_cameras/pose_estimation.hpp"

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;

struct Camera
{
  cv::Mat image;
  cv::Mat timestamp;
  cam_calib camera_calib_set;
  bool pattern_detected;
} ;

class Calibrator
{
public:
  Calibrator();

  //save image pairs continuously
  void saveImagePairs();

  //save a single image pair
  void saveImagePair();

  //read and process recorded image pairs
  void readAndProcessImages();

private:
  void poseProcess();

  void processCameraInfo(const sensor_msgs::CameraInfoConstPtr& infoMsg,
                         const int &cam_index);

  void processDepthInfo(const sensor_msgs::CameraInfoConstPtr& infoMsg);

  void processRgbInfo(const sensor_msgs::CameraInfoConstPtr& infoMsg);

  //process a single image
  bool processImage(const cv::Mat &image,
                    const int &cam_index);

  //process an image pair
  void processImagePair(const sensor_msgs::ImageConstPtr& msg_rgb,
                        const sensor_msgs::ImageConstPtr& msg_depth);

  bool prepareRgb(const sensor_msgs::ImageConstPtr& msg);

  bool prepareDepth(const sensor_msgs::ImageConstPtr& msg);

  float getSharpness(const cv::Mat &image,
                     const int px_num);

  cv::Mat imageTo8U(const cv::Mat &image,
                    const std::string &encoding);

  bool readImage(const std::string &file_name,
                 cv::Mat *image);

  //write frames to a file
  void writeFrames();

  //read frames from a file
  bool readFrames();

  //node handle
  ros::NodeHandle nh_;

  message_filters::Synchronizer<MySyncPolicy> *sync_;

  //subscriber to RGB image
  message_filters::Subscriber<sensor_msgs::Image> *subscriber_rgb_;

  //subscriber to depth image
  message_filters::Subscriber<sensor_msgs::Image> *subscriber_ir_;

  //subscriber to RGB camera info
  ros::Subscriber sub_cam_rgb_info_;

  //subscriber to depth camera info
  ros::Subscriber sub_cam_depth_info_;

  //instance of an pose estimation class
  PoseEstimation pose_estimator_;

  //storage of images from cameras
  std::vector<Camera> cameras_;

  //number of cameras
  int nbr_cameras_;

  //maximum number of image pairs to record
  int buf_size_;

  //should we process an image or not
  bool processing_;

  //current number of recorded image pairs
  int buf_now_;

  //mutex for the rgb image
  std::mutex img_mutex_rgb_;

  //mutex for the depth image
  std::mutex img_mutex_depth_;

  //transform listener
  tf::TransformListener listener_;

  //input directory to save/read image pairs
  std::string input_dir_;

  //are the frames initialized
  bool is_initialized_;

  //filename to write frames
  std::string file_frames_;

  //cameras frames
  std::vector<std::string> frames_;
};

#endif // CALIBRATOR_HPP
