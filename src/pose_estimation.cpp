#include "pose_estimation.hpp"

//Include basic headers
#include <iostream>
#include <ctime>
#include <cstring>

#define HAS_OPENCV3 0

PoseEstimation::PoseEstimation():
  pattern_square_size_(0.025),
  pattern_size_(cv::Size(9, 6))
{
}

void PoseEstimation::setPatternSize(const int &w,
                                    const int &h,
                                    const float &d)
{
  pattern_size_.width = w;
  pattern_size_.height = h;
  pattern_square_size_ = d;
}

bool PoseEstimation::checkCheckerboardPoints(cv::Mat &image)
{
  std::vector<cv::Point2f> imagePoints;
  // WE NEED TO GET GRAYSCALE IMAGE
  bool found_checker_cl = cv::findChessboardCorners(image,
                                                    pattern_size_,
                                                    imagePoints,
                                                    CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS + cv::CALIB_CB_FAST_CHECK);

  //draw points
  if (image.channels() == 1)
    cv::cvtColor(image, image, CV_GRAY2RGB);
  cv::drawChessboardCorners(image, pattern_size_, imagePoints, found_checker_cl);

  if (found_checker_cl == true && imagePoints.size() == pattern_size_.area())
  {
    return true;
  }
  return false;
}

bool getCheckerboardPoints(cv::Mat &image,
                           std::vector<cv::Point2f>& imagePoints,
                           cv::Size &pattern_size)
{
  // WE NEED TO GET GRAYSCALE IMAGE
  bool found_checker_cl = cv::findChessboardCorners(image,
                                                    pattern_size,
                                                    imagePoints,
                                                    CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS + cv::CALIB_CB_FAST_CHECK);

  if (found_checker_cl)
  {
    /// Set the neeed parameters to find the refined corners
    cv::Size winSize = cv::Size(5, 5);
    cv::Size zeroZone = cv::Size(-1, -1);
    cv::TermCriteria criteria = cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 40, 0.001);

    /// Calculate the refined corner locations
    cornerSubPix(image, imagePoints, winSize, zeroZone, criteria);

    // Whole pattern is recovered
    if (imagePoints.size() == pattern_size.area())
    {
      return true;
    }
    return false;
  }
  return false;
}

void PoseEstimation::init(const int &n_cameras)
{
  world_points.clear();
  image_points.clear();
  image_points.reserve(n_cameras);
  image_points.resize(n_cameras);
  image_pairs.clear();
  image_pairs.reserve(n_cameras);
  image_pairs.resize(n_cameras);

  for (int i = 0; i < n_cameras; ++i)
  {
    image_points[i] = std::vector<std::vector<cv::Point2f> >();
    image_pairs[i] = std::vector<cv::Mat>();
  }
}

bool PoseEstimation::addImagePair(cv::Mat &img_c1,
                                  cv::Mat &img_c2)
{
  //std::cout << "Add image pair" << std::endl;
  std::vector<cv::Point2f> image_points_1;
  std::vector<cv::Point2f> image_points_2;
  bool found_checker_cl = getCheckerboardPoints(img_c1, image_points_1, pattern_size_);
  bool found_checker_c2 = getCheckerboardPoints(img_c2, image_points_2, pattern_size_);

  if (found_checker_cl && found_checker_c2)
  {
    //std::cout << "Two checkerboards found" << std::endl;
    // Create real world coordinates
    world_points.push_back(std::vector<cv::Point3f>());
    for (int i = 0; i < pattern_size_.height; ++i)
    {
      for (int j = 0; j < pattern_size_.width; ++j)
      {
        world_points[world_points.size()-1].push_back(
              cv::Point3f(float(i)*pattern_square_size_, float(j)*pattern_square_size_, 1.0f));
      }
    }

    image_points[0].push_back(image_points_1);
    image_points[1].push_back(image_points_2);

    if (image_pairs.size() >= 2)
    {
      image_pairs[0].push_back(img_c1.clone());
      image_pairs[1].push_back(img_c2.clone());
    }
    return true;
  }
  return false;
}

float PoseEstimation::estimatePose(CameraCalibration& C1,
                                   CameraCalibration& C2,
                                   Pose& pose)
{
  if (getNumberOfImagePairs() > 0)
  {
#if HAS_OPENCV3
    pose.rms = cv::stereoCalibrate(world_points,
                                   image_points[0],
                                   image_points[1],
                                   C1.A, C1.d, C2.A, C2.d,
                                   image_pairs[0][0].size(),
                                   pose.R, pose.t, pose.E, pose.F,
                                   cv::CALIB_FIX_INTRINSIC,
                                   cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 100, DBL_EPSILON));
#else
    pose.rms = cv::stereoCalibrate(world_points,
                                   image_points[0],
                                   image_points[1],
                                   C1.A, C1.d, C2.A, C2.d,
                                   image_pairs[0][0].size(),
                                   pose.R, pose.t, pose.E, pose.F,
                                   cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 100, DBL_EPSILON),
                                   cv::CALIB_FIX_INTRINSIC);
#endif

    return pose.rms;
  }
  else
    return std::numeric_limits<float>::max();
}

int PoseEstimation::getNumberOfImagePairs()
{
  if (image_points.size() > 0)
    return PoseEstimation::image_points[0].size();
  else
    return 0;
}

std::vector<std::vector<cv::Mat> > PoseEstimation::getImagePairs()
{
  return image_pairs;
}
