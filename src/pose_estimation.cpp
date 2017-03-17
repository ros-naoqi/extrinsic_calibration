#include "calibration_cameras/pose_estimation.hpp"

//Include basic headers
#include <iostream>
#include <ctime>
#include <cstring>

PoseEstimation::PoseEstimation():
  pattern_square_size(0.025),
  pattern_size(cv::Size(9, 6))
{
}

void PoseEstimation::setPatternSize(const int &w, const int &h, const float &d)
{
  pattern_size.width = w;
	pattern_size.height = h;
	pattern_square_size = d;
}

bool PoseEstimation::checkCheckerboardPoints(cv::Mat &image)
{
	std::vector<cv::Point2f> imagePoints;
	// WE NEED TO GET GRAYSCALE IMAGE
	bool found_checker_cl = cv::findChessboardCorners(image, pattern_size, imagePoints, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS + cv::CALIB_CB_FAST_CHECK);

  //draw points
  if (image.channels() == 1)
    cv::cvtColor(image, image, CV_GRAY2RGB);
  cv::drawChessboardCorners(image, pattern_size, imagePoints, found_checker_cl);

  if (found_checker_cl == true && imagePoints.size() == pattern_size.area())
	{
		return true;
	}
	return false;
}

bool getCheckerboardPoints(cv::Mat &image, std::vector<cv::Point2f>& imagePoints, cv::Size &pattern_size)
{
	// WE NEED TO GET GRAYSCALE IMAGE
	bool found_checker_cl = cv::findChessboardCorners(image, pattern_size, imagePoints, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS + cv::CALIB_CB_FAST_CHECK);

  if (found_checker_cl)
	{
		/// Set the neeed parameters to find the refined corners
		cv::Size winSize = cv::Size(5, 5);
		cv::Size zeroZone = cv::Size(-1, -1);
		cv::TermCriteria criteria = cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 40, 0.001);

		/// Calculate the refined corner locations
		cornerSubPix(image, imagePoints, winSize, zeroZone, criteria);

		// Whole pattern is recovered
		if (imagePoints.size() == pattern_size.area()){
			return true;
		}
		return false;
	}
	return false;
}

void PoseEstimation::initEstimation(const int &n_cameras)
{
	worldCheckerboardPoints.clear();
	imageCheckerboradPoints.clear();
  imageCheckerboradPoints.reserve(n_cameras);
  imageCheckerboradPoints.resize(n_cameras);
	imagesCalibration.clear();
  imagesCalibration.reserve(n_cameras);
  imagesCalibration.resize(n_cameras);

	for (int i = 0; i < n_cameras; ++i){
		imageCheckerboradPoints[i] = std::vector<std::vector<cv::Point2f> >();
		imagesCalibration[i] = std::vector<cv::Mat>();
	}
}

bool PoseEstimation::addImagePair(cv::Mat &img_c1, cv::Mat &img_c2)
{
  //std::cout << "Add image pair" << std::endl;
	std::vector<cv::Point2f> imagePoints_1;
	std::vector<cv::Point2f> imagePoints_2;
//std::cout << "87" << img_c1.channels() << " " << img_c1.type() << std::endl;
  bool found_checker_cl = getCheckerboardPoints(img_c1, imagePoints_1, pattern_size);
//std::cout << "89" << img_c2.channels() << " " << img_c2.type() << std::endl;
  bool found_checker_c2 = getCheckerboardPoints(img_c2, imagePoints_2, pattern_size);
//std::cout << "91" << std::endl;
	if (found_checker_cl && found_checker_c2)
	{
    //std::cout << "Two checkerboards found" << std::endl;
		// Create real world coordinates
		worldCheckerboardPoints.push_back(std::vector<cv::Point3f>());
    for (int i = 0; i < pattern_size.height; ++i){
      for (int j = 0; j < pattern_size.width; ++j){
        worldCheckerboardPoints[worldCheckerboardPoints.size()-1].push_back(
              cv::Point3f(float(i)*pattern_square_size, float(j)*pattern_square_size, 1.0f));
			}
		}

		imageCheckerboradPoints[0].push_back(imagePoints_1);
		imageCheckerboradPoints[1].push_back(imagePoints_2);

    if (imagesCalibration.size() >= 2)
    {
      imagesCalibration[0].push_back(img_c1.clone());
      imagesCalibration[1].push_back(img_c2.clone());
      /*imagesCalibration[0].resize(imagesCalibration[0].size()+1);
      imagesCalibration[0].back() = img_c1;
      imagesCalibration[1].resize(imagesCalibration[1].size()+1);
      imagesCalibration[1].back() = img_c2;*/
    }
		return true;
	}
	return false;
}

float PoseEstimation::estimatePose(cam_calib& C1,cam_calib& C2,Pose& pose)
{
  if (getNumberOfImagePairs() > 0)
    pose.rms = cv::stereoCalibrate(worldCheckerboardPoints,
                                   imageCheckerboradPoints[0],
                                   imageCheckerboradPoints[1],
                                   C1.A, C1.d, C2.A, C2.d,
                                   imagesCalibration[0][0].size(),
                                   pose.R, pose.t, pose.E, pose.F,
                                   cv::CALIB_FIX_INTRINSIC,
                                   cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 1e-6));
	
	return pose.rms;

}

int PoseEstimation::getNumberOfImagePairs()
{
  if (imageCheckerboradPoints.size() > 0)
    return PoseEstimation::imageCheckerboradPoints[0].size();
  else
    return 0;
}

std::vector<std::vector<cv::Mat> > PoseEstimation::getImagePairs()
{
  return imagesCalibration;
}
