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
            << "* for data recording: place your calibration pattern "
            << "in front of the camera "
            << "and when the pattern is detected, press -s- key"
            << std::endl
            << "* for data processing: when you finish with recording, "
            << "press -c- key to process the recorded data"
            << std::endl;

  while(ros::ok())
  {
    ros::spinOnce();

    uchar key_pressed;
    if ((key_pressed = cv::waitKey(10)) != -1)
    {
      if (key_pressed == 'r')
      {
        calib.saveImagePairs();
      }
      else if (key_pressed == 's')
      {
        calib.saveImagePair();
      }
      else if (key_pressed == 'c')
      {
        calib.readAndProcessImages();
      }
    }
    rate.sleep();
  }

  ros::shutdown();  
  return 0;
}
