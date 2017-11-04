#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "dirent.h"

#include <iostream>
#include <fstream>
#include <string>

#include "calibrator.hpp"

Calibrator::Calibrator():
  nh_("~"),
  nbr_cameras_(2),
  buf_size_(40),
  processing_(false),
  buf_now_(0),
  input_dir_("images"),
  file_frames_("frames.txt"),
  is_initialized_(false)
{
  cameras_.reserve(nbr_cameras_);
  cameras_.resize(nbr_cameras_);

  frames_.reserve(nbr_cameras_);
  frames_.resize(nbr_cameras_);

  frames_[0] = "CameraTop_optical_frame";
  frames_[1] = "CameraDepth_optical_frame";

  cameras_[0].camera_calib_set.A = (cv::Mat_<double>(3, 3) <<
                                    303.608524, 0.0, 164.350243,
                                    0.0, 303.307367, 124.363650,
                                    0.0, 0.0, 1.0);

  cameras_[0].camera_calib_set.d = (cv::Mat_<double>(1, 5) <<
                                    0.144703,
                                    -0.394814,
                                    0.003265,
                                    0.001382,
                                    0);

  cameras_[1].camera_calib_set.A = (cv::Mat_<double>(3, 3) <<
                                    286.661252, 0.0, 174.414458,
                                    0.0, 285.291921, 120.318757,
                                    0.0, 0.0, 1.0);

  cameras_[1].camera_calib_set.d = (cv::Mat_<double>(1, 5) <<
                                    -0.020129,
                                    0.038371,
                                    0.001638,
                                    0.016381,
                                    0);

  pose_estimator_.init(nbr_cameras_);

  std::string topic_depth_info = "/naoqi_driver_node/camera/depth/camera_info";
  nh_.getParam("camera_info_depth_topic", topic_depth_info);
  ROS_INFO_STREAM("camera_info_depth_topic: " << topic_depth_info);
  sub_cam_depth_info_ = nh_.subscribe(topic_depth_info.c_str(), 1, &Calibrator::processDepthInfo, this);

  std::string topic_rgb_info = "/naoqi_driver_node/camera/front/camera_info";
  nh_.getParam("camera_info_rgb_topic", topic_rgb_info);
  ROS_INFO_STREAM("camera_info_rgb_topic: " << topic_rgb_info);
  sub_cam_rgb_info_ = nh_.subscribe(topic_rgb_info.c_str(), 1, &Calibrator::processRgbInfo, this);

  std::string topic_depth_img = "/naoqi_driver_node/camera/ir/image_raw";
  nh_.getParam("depth_img_topic", topic_depth_img);
  ROS_INFO_STREAM("topic_depth_img: " << topic_depth_img);

  std::string topic_rgb_img = "/naoqi_driver_node/camera/front/image_raw";
  nh_.getParam("rgb_img_topic", topic_rgb_img);
  ROS_INFO_STREAM("topic_rgb_img: " << topic_rgb_img);

  subscriber_ir_ = new message_filters::Subscriber<sensor_msgs::Image>(nh_, topic_depth_img, 1);
  subscriber_rgb_ = new message_filters::Subscriber<sensor_msgs::Image>(nh_, topic_rgb_img, 1);

  sync_ = new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(10), *subscriber_rgb_, *subscriber_ir_);
  sync_->registerCallback(boost::bind(&Calibrator::processImagePair, this, _1, _2));

  //initialize the poseEstimation class
  int w_pattern(0);
  int h_pattern(0);
  float d_pattern(0);
  nh_.getParam("pattern_cols", w_pattern);
  nh_.getParam("pattern_rows", h_pattern);
  nh_.getParam("pattern_size", d_pattern);
  ROS_INFO_STREAM("pattern_cols: " << w_pattern);
  ROS_INFO_STREAM("pattern_rows: " << h_pattern);
  ROS_INFO_STREAM("pattern_size: " << d_pattern);
  ROS_INFO_STREAM("pattern_size: " << d_pattern);
  pose_estimator_.setPatternSize(w_pattern, h_pattern, d_pattern);

  nh_.getParam("images_folder", input_dir_);
}

void Calibrator::processCameraInfo(const sensor_msgs::CameraInfoConstPtr& infoMsg,
                                   const int &cam_index)
{
  frames_.at(cam_index) = infoMsg->header.frame_id;

  if (cam_index < cameras_.size())
    cameras_[cam_index].camera_calib_set.A = (cv::Mat_<double>(3, 3) <<
                       infoMsg->K[0], infoMsg->K[1], infoMsg->K[2],
                       infoMsg->K[3], infoMsg->K[4], infoMsg->K[5],
                       infoMsg->K[6], infoMsg->K[7], infoMsg->K[8]);

  if (cam_index < cameras_.size())
    cameras_[cam_index].camera_calib_set.d = (cv::Mat_<double>(1, 5) <<
                        infoMsg->D[0],
                        infoMsg->D[1],
                        infoMsg->D[2],
                        infoMsg->D[3],
                        infoMsg->D[4]);
}

void Calibrator::processRgbInfo(const sensor_msgs::CameraInfoConstPtr& infoMsg)
{
  processCameraInfo(infoMsg, 0);

  //unsubscribe from the topic
  sub_cam_rgb_info_.shutdown();
}

void Calibrator::processDepthInfo(const sensor_msgs::CameraInfoConstPtr& infoMsg)
{
  processCameraInfo(infoMsg, 1);

  //unsubscribe from the topic
  sub_cam_depth_info_.shutdown();
}

float Calibrator::getSharpness(const cv::Mat &image,
                               const int px_nbr)
{
  //Convert image using Canny
  cv::Mat edges;
  cv::Canny(image, edges, 225, 175);

  //Count the number of pixel representing an edge
  int nCountCanny = cv::countNonZero(edges);

  //Compute a sharpness grade:
  //< 1.5 = blurred, in movement
  //de 1.5 à 6 = acceptable
  //> 6 =stable, sharp
  float sharpness = static_cast<float>(nCountCanny) * 1000.0f
          / static_cast<float>(px_nbr);

  //ROS_INFO_STREAM("sharpness= " << sharpness);
  return sharpness;
}

bool Calibrator::processImage(const cv::Mat &image,
                              const int &cam_index)
{
  if (cam_index >= cameras_.size())
    return false;

  bool res(false);
  cv::Mat image_vis;
  image.copyTo(image_vis);
  try
  {
    if (pose_estimator_.checkCheckerboardPoints(image_vis))
    {
      //if (verbose)
        //ROS_INFO_STREAM("checkerboard found in " << cam_index);
      res = true;
     }
   }
   catch(cv::Exception &e)
   {
     const char* err_msg = e.what();
     ROS_ERROR(err_msg);
   }

  //visualize
  if (!image_vis.empty())
  {
    if (cam_index == 0)
      cv::imshow("image_rgb", image_vis);
    else
      cv::imshow("image_ir", image_vis);
  }

  return res;
}

bool Calibrator::prepareDepth(const sensor_msgs::ImageConstPtr& msg)
{
  int cam_index = 1;

  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return false;
  }

  if (cameras_[cam_index].image.empty())
  {
     cameras_[cam_index].image.create(cv::Size(msg->height, msg->width), CV_8UC1);
     cameras_[cam_index].px_nbr = msg->height * msg->width;
  }

  /*if (getSharpness(cv_ptr->image, cameras_[cam_index].px_nbr) < 1.5)
    return false;*/

  if (img_mutex_depth_.try_lock())
  {
    cameras_[cam_index].image = imageTo8U(cv_ptr->image, msg->encoding);
    img_mutex_depth_.unlock();
  }

  return true;
}

cv::Mat Calibrator::imageTo8U(const cv::Mat &image,
                              const std::string &encoding)
{
  cv::Mat res;
  try
  {
    if (encoding == sensor_msgs::image_encodings::TYPE_32FC1)
    {
      image.convertTo(res, CV_8UC1, 1);
    }
    else if (encoding == sensor_msgs::image_encodings::TYPE_16UC1)
    {
      image.convertTo(res, CV_8UC1, 1);
      cv::fastNlMeansDenoising(res, res);
      cv::equalizeHist(res, res);
      cv::Mat valid_mask = image == std::numeric_limits<uint16_t>::min();
      res.setTo(std::numeric_limits<float>::quiet_NaN(), valid_mask);
    }
    else if (encoding == sensor_msgs::image_encodings::MONO8)
    {
      image.convertTo(res, CV_8UC1, 1);
      cv::Mat valid_mask = image == std::numeric_limits<uint8_t>::min();
      res.setTo(std::numeric_limits<float>::quiet_NaN(), valid_mask);
    }
    else
    {
      ROS_ERROR("Unknown depth image format");
    }
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
  }
  return res;
}

bool Calibrator::prepareRgb(const sensor_msgs::ImageConstPtr& msg)
{
  int cam_index = 0;

  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return false;
  }

  if (cameras_[cam_index].image.empty())
  {
     cameras_[cam_index].image.create(cv::Size(msg->height, msg->width), CV_8UC1);
     cameras_[cam_index].px_nbr = msg->height * msg->width;
  }

  if (getSharpness(cv_ptr->image, cameras_[cam_index].px_nbr) < 1.5f)
    return false;

  if (img_mutex_rgb_.try_lock())
  {
    cv::cvtColor(cv_ptr->image, cameras_[cam_index].image, CV_BGR2GRAY);
    img_mutex_rgb_.unlock();
  }

  return true;
}

void Calibrator::poseProcess()
{
  if (pose_estimator_.getNumberOfImagePairs() > 0)
  {
    //Perform estimation of extrinsic parameters
    Pose pose_res;

    float rms = pose_estimator_.estimatePose(cameras_[0].camera_calib_set,
                                             cameras_[1].camera_calib_set,
                                             pose_res);

    ROS_INFO_STREAM("Pose estimation error: " << rms);

    pose_res.R.at<double>(0,0) *= -1;
    pose_res.R.at<double>(1,0) *= -1;
    pose_res.R.at<double>(2,0) *= -1;

    pose_res.R.at<double>(0,1) *= -1;
    pose_res.R.at<double>(1,1) *= -1;
    pose_res.R.at<double>(2,1) *= -1;

    pose_res.R.at<double>(0,2) *= -1;
    pose_res.R.at<double>(1,2) *= -1;
    pose_res.R.at<double>(2,2) *= -1;

    double pitch = atan2(pose_res.R.at<double>(2,1),
                         pose_res.R.at<double>(2,2));

    double roll = atan2(-pose_res.R.at<double>(2,0),
                        sqrt(pose_res.R.at<double>(2,1)*pose_res.R.at<double>(2,1)
                        + pose_res.R.at<double>(2,2)*pose_res.R.at<double>(2,2)));

    double yaw = atan2(pose_res.R.at<double>(1,0),
                       pose_res.R.at<double>(0,0));

    //transform in quaternion
    tf::Quaternion pose_quat;
    pose_quat.setRPY(roll, pitch, yaw);

    tf::Stamped<tf::Pose> pose_depth_to_rgb(
          tf::Pose(pose_quat,
                   tf::Vector3(-pose_res.t.at<double>(0,0),
                               -pose_res.t.at<double>(0,1),
                               -pose_res.t.at<double>(0,2))),
          ros::Time(0),
          frames_.at(0));

    //transform from frame_0 to frame_1
    tf::Matrix3x3(pose_depth_to_rgb.getRotation()).getRPY(roll, pitch, yaw);

    ROS_INFO_STREAM(frames_.at(0) << " to " << frames_.at(1) << " : XYZ: "
                    << pose_depth_to_rgb.getOrigin().x() << " "
                    << pose_depth_to_rgb.getOrigin().y() << " "
                    << pose_depth_to_rgb.getOrigin().z());
    ROS_INFO_STREAM(frames_.at(0) << " to " << frames_.at(1) << " : RPY: "
                    << std::fabs(roll)-3.1416 << " " << pitch << " " << yaw);

    //transform the pose to head frame
    std::string frame_head("Head");
    tf::Stamped<tf::Pose> pose_depth_to_head;
    try
    {
      listener_.transformPose(frame_head, pose_depth_to_rgb, pose_depth_to_head);

      tf::Matrix3x3(pose_depth_to_head.getRotation()).getRPY(roll, pitch, yaw);

      ROS_INFO_STREAM(frames_.at(1) << " to " << frame_head << " : "
                      << "XYZ: " << pose_depth_to_head.getOrigin().x() << " "
                      << pose_depth_to_head.getOrigin().y() << " "
                      << pose_depth_to_head.getOrigin().z());
      ROS_INFO_STREAM(frames_.at(1) << " to " << frame_head << " : "
                      << "RPY: " << -1.0*roll << " " << pitch << " " << yaw);
    }
    catch (tf::TransformException ex)
    {
     ROS_ERROR("%s",ex.what());
    }
  }
}

void Calibrator::processImagePair(const sensor_msgs::ImageConstPtr& msg_rgb,
                                  const sensor_msgs::ImageConstPtr& msg_depth)
{
  //prepare RGB image
  if (!prepareRgb(msg_rgb))
    return;

  //prepare depth image
  if (!prepareDepth(msg_depth))
    return;

  //cv::imshow("image_rgb_", cameras_[0].image);
  //cv::imshow("image_depth_", cameras_[1].image);

  //process
  //if (img_mutex_rgb.try_lock() && img_mutex_depth.try_lock())
  {
    for (int i=0; i<cameras_.size(); ++i)
      if (!processImage(cameras_[i].image, i))
      {
        cameras_[i].pattern_detected = false;
        return;
      }
      else
        cameras_[i].pattern_detected = true;
    //img_mutex_rgb.unlock();
    //img_mutex_depth.unlock();
  }

  if (!processing_)
    return;

  //cv::imshow("image_rgb_", cameras_[0].image);
  //cv::imshow("image_depth_", cameras_[1].image);

  if (pose_estimator_.getNumberOfImagePairs() <= buf_size_)
  {
    if (pose_estimator_.addImagePair(cameras_[0].image, cameras_[1].image))
    {
      poseProcess();
      ROS_INFO_STREAM("Added image pair: " << pose_estimator_.getNumberOfImagePairs());
    }
  }
  else
    ROS_ERROR_STREAM("Maximum number of image pairs " << buf_size_ << " achieved");
}

void Calibrator::saveImagePairs()
{
  processing_ = false;

  ROS_INFO_STREAM("Writing images ...");

  if (pose_estimator_.getImagePairs().size() > 0)
    for (int i=0; i<pose_estimator_.getImagePairs()[0].size(); ++i)
      for (int k=0; k<pose_estimator_.getImagePairs().size(); ++k)
      {
        std::stringstream strstr;
        strstr << "pair_" << i << "_" << k << ".jpg";
        try
        {
          cv::imwrite(strstr.str().c_str(), pose_estimator_.getImagePairs()[k][i]);
        }
        catch(cv::Exception &e)
        {
          const char* err_msg = e.what();
          ROS_ERROR(err_msg);
        }
      }
  ROS_INFO_STREAM("Writing images is finished");
}

void Calibrator::writeFrames()
{
  ROS_INFO_STREAM("Writing the file " << input_dir_ << "/" << file_frames_);
  std::ofstream file(input_dir_ + "/" + file_frames_);

  if (!file.is_open())
  {
    ROS_INFO_STREAM("File " << input_dir_ << "/" << file_frames_ << " not found");
    return;
  }

  for (int i=0; i < frames_.size(); ++i)
    file << frames_.at(i) << "\n";

  file.close();
}

bool Calibrator::readFrames()
{
  ROS_INFO_STREAM("Reading the file " << input_dir_ << "/" << file_frames_);
  std::ifstream file(input_dir_ + "/" + file_frames_);

  if (!file.is_open())
  {
    ROS_INFO_STREAM("File " << input_dir_ << "/" << file_frames_ << " not found");
    return false;
  }

  std::string frame;
  std::vector<std::string>::iterator i = frames_.begin();
  while (std::getline(file, frame))
  {
    if (i != frames_.end())
      *i = frame;
    ++i;
  }

  file.close();
  return true;
}

void Calibrator::saveImagePair()
{
  if (!is_initialized_)
  {
    writeFrames();
    is_initialized_ = true;
  }

  if (cameras_.size() < 2)
    return;

  if (cameras_[0].isEmpty())
  {
    ROS_WARN("No image is acquired, check that Naoqi Driver is running.");
    return;
  }

  if (!cameras_[0].patternDetected() || !cameras_[1].patternDetected())
  {
    ROS_WARN("No calibration pattern was detected.");
    return;
  }

  for (int k=0; k<cameras_.size(); ++k)
  {
    std::stringstream strstr;
    strstr << input_dir_ + "/pair_" << buf_now_ << "_" << k << ".jpg";
    ROS_INFO_STREAM("Saving an image to " << strstr.str());
    try
    {
      if (!cameras_[k].image.empty())
        cv::imwrite(strstr.str().c_str(), cameras_[k].image);
    }
    catch(cv::Exception &e)
    {
      const char* err_msg = e.what();
      ROS_ERROR(err_msg);
    }
  }

  ++buf_now_;
}

bool Calibrator::readImage(const std::string &file_name,
                           cv::Mat *image)
{
  //initialize if needed
  if (image->empty())
  {
    cv::Mat image_temp = cv::imread(file_name, CV_LOAD_IMAGE_GRAYSCALE);
    if(image_temp.empty())
      return false;
    image->create(cv::Size(image_temp.rows, image_temp.cols), CV_8UC1);
  }

  *image = cv::imread(file_name, CV_LOAD_IMAGE_GRAYSCALE);
  if(image->empty())
    return false;

  return true;
}

void Calibrator::readAndProcessImages()
{
  if (!listener_.canTransform(frames_.at(1),
                              frames_.at(0),
                              ros::Time(0)))
  {
    ROS_ERROR("Please, start Naoqi Driver at first");
    return;
  }

  processing_ = true;

  //override frames if found in recordings
  readFrames();

  DIR *directory = opendir(input_dir_.c_str());
  struct dirent *_dirent = NULL;
  if(directory == NULL)
  {
    ROS_ERROR_STREAM("Cannot open the Input Folder " << input_dir_);
    return;
  }

  while((_dirent = readdir(directory)))
  {
    std::string file_name = input_dir_ + "/" + std::string(_dirent->d_name);

    //ROS_INFO_STREAM("Processing " << file_name);

    if (file_name.length() <= 5)
      continue;

    if (static_cast<int>(file_name[file_name.length()-5]-'0') != 0)
      continue;

    //read the RGB image
    if (!readImage(file_name, &cameras_[0].image))
      continue;

    //read the depth image
    std::string file_name_depth(file_name);
    file_name_depth[file_name_depth.length()-5] = '1';
    if (!readImage(file_name_depth, &cameras_[1].image))
      continue;

    if (!processing_)
      continue;

    //check if the pattern found
    bool chessboard_found(true);
    for (int i=0; i<cameras_.size(); ++i)
    {
      if (!processImage(cameras_[i].image, i))
      {
        chessboard_found = false;
        continue;
      }
    }

    if (!chessboard_found)
      continue;

    //add an image pair
    if (pose_estimator_.getNumberOfImagePairs() <= buf_size_)
    {
      if (pose_estimator_.addImagePair(cameras_[0].image, cameras_[1].image))
      {
        poseProcess();
        ROS_INFO_STREAM("Image pair: " << pose_estimator_.getNumberOfImagePairs() << "\n");
      }
    }
    else
      ROS_ERROR_STREAM("Maximum number of image pairs " << buf_size_ << " achieved");
  }
  closedir(directory);
}
