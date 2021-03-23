#include <iostream>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "sensor_msgs/Image.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/features2d.hpp>
#include <cmath>
#include "std_msgs/Float32.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PointStamped.h"
#include <ros/ros.h>

class CheckDetection
{
  public:

  std::vector<cv::KeyPoint> keypoints;
  cv::SimpleBlobDetector::Params params;
  cv::Mat orignial_image,tmp,im_with_keypoints;



 // const std::string OPENCV_WINDOW = "Image window";

  CheckDetection();
  ~CheckDetection();

 void check(cv::Mat);
};