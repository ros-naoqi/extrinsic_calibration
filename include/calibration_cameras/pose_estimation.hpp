#ifndef POSEESTIMATION_HPP
#define POSEESTIMATION_HPP

#include <opencv2/opencv.hpp>
#include "opencv2/calib3d/calib3d.hpp"

struct cam_calib{
	cv::Mat A;
	cv::Mat d;
};
struct Pose{
	cv::Mat R;
	cv::Mat t;
	cv::Mat E;
	cv::Mat F;
	float rms;
};

class PoseEstimation
{
public:
  PoseEstimation();
  void init(const int &n_cam);

  void setPatternSize(const int &w,
                      const int &h,
                      const float &dim);

  bool checkCheckerboardPoints(cv::Mat &image);

  bool addImagePair(cv::Mat &img_1,
                    cv::Mat &img_2);

  float estimatePose(cam_calib& C1,
                     cam_calib& C2,
                     Pose &pose);

  int getNumberOfImagePairs();

  std::vector<std::vector<cv::Mat> > getImagePairs();

protected:
  //pattern square size
  float pattern_square_size_;

  //pattern size
  cv::Size pattern_size_;

  //coordinates of points in the world space
  std::vector<std::vector<cv::Point3f> > world_points;

  //coordinates of points in the image space
  std::vector<std::vector<std::vector<cv::Point2f> > > image_points;

  //images for calibration
  std::vector<std::vector<cv::Mat> > image_pairs;
};

#endif // POSEESTIMATION_HPP
