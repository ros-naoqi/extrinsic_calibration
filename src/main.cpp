#include "calibration_cameras/calibrator.hpp"

#include <opencv2/opencv.hpp>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "calibration_cameras");

  Calibrator calib;

  cv::namedWindow("image_rgb", cv::WINDOW_AUTOSIZE);
  cv::namedWindow("image_ir", cv::WINDOW_AUTOSIZE);
  cv::waitKey(1);

  ros::Rate rate(10);

  std::cout << std::endl << "Please, ensure that you see an image "
            << "in -image_rgb- window and press a key "
            << "to choose an operating mode:"
            << std::endl
            << "* press -s- key to save an image pair: "
            << "place your calibration pattern in front of the robot "
            << "and save an image pair when the pattern is detected; "
            << "you should save it at different positions, orientations, scales "
            << "(at least 20 pairs or more)"
            << std::endl
            << "* press -c- key to process the recorded data"
            << std::endl;

  uchar key_pressed;
  while(ros::ok())
  {
    ros::spinOnce();

    if ((key_pressed = cv::waitKey(10)) != -1)
    {
      /*if (key_pressed == 'r')
      {
        calib.saveImagePairs();
      }
      else*/ if (key_pressed == 's')
      {
        calib.saveImagePair();
      }
      else if (key_pressed == 'c')
      {
        calib.readAndProcessImages();
        ros::shutdown();
      }
    }
    rate.sleep();
  }

  ros::shutdown();
  return 0;
}
