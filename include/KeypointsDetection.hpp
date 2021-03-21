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
#include "medianFilter.hpp"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PointStamped.h"

class KeypointsDetection
{
  public:
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  std::vector<cv::KeyPoint> keypoints;
  cv::SimpleBlobDetector::Params params;
  float threshold;
  cv::Point2d center_point;
  std::vector<cv::Point2f> list_of_positions;
  ros::Publisher pixel_center_location;
  medianFilter* filter=new medianFilter();
  geometry_msgs::PointStamped pixel_pos;
  cv::Mat blurred,im_with_keypoints;


  const std::string OPENCV_WINDOW = "Image window";

  KeypointsDetection(ros::NodeHandle &);
  ~KeypointsDetection();

 void findCenter(cv::Mat*, ros::Time);
};