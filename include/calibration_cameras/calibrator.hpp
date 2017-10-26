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

  void saveImagePairs();
  void saveImagePair();

  void readAndProcessImages();

private:
  void poseProcess(const std::string &frame);

  void process_cam_info(const sensor_msgs::CameraInfoConstPtr& infoMsg, const int &cam_index);
  void process_cam_depth_info(const sensor_msgs::CameraInfoConstPtr& infoMsg);
  void process_cam_rgb_info(const sensor_msgs::CameraInfoConstPtr& infoMsg);

  bool process_image(const cv::Mat &image, const int &cam_index);
  void process_images(const sensor_msgs::ImageConstPtr& msg_rgb, const sensor_msgs::ImageConstPtr& msg_depth);

  bool prepare_rgb(const sensor_msgs::ImageConstPtr& msg);
  bool prepare_depth(const sensor_msgs::ImageConstPtr& msg);

  float getSharpness(const cv::Mat &image);

  cv::Mat imageTo8U(const cv::Mat &image, const std::string &encoding);

  //write frames to a file
  void writeFrames();

  //read frames from a file
  bool readFrames();

  ros::NodeHandle nh_;

  message_filters::Synchronizer<MySyncPolicy> *sync;

  message_filters::Subscriber<sensor_msgs::Image> *subscriber_rgb, *subscriber_ir;
  ros::Subscriber sub_cam_rgb_info_, sub_cam_depth_info_;

  PoseEstimation pose;

  std::vector<Camera> cameras_;

  int n_cameras;
  int buf_size;
  bool processing;
  int px_num;
  int buf_now;

  std::mutex img_mutex_rgb;
  std::mutex img_mutex_depth;

  //transform listener
  tf::TransformListener listener_;

  std::string input_dir_;

  //are the frames initialized
  bool is_initialized_;

  //filename to write frames
  std::string file_frames_;

  //camera frames
  std::vector<std::string> frames_;
};

#endif // CALIBRATOR_HPP
