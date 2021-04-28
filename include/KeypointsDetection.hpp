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
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "HEAR_math/ButterFilter_2nd.hpp"


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
  ros::Publisher pixel_center_location, pixel_center_location_filtered;
  //medianFilter* filter=new medianFilter();
  ButterFilter_2nd* filter_x=new ButterFilter_2nd(ButterFilter_2nd::BF_settings::FS100FC30);
  ButterFilter_2nd* filter_y=new ButterFilter_2nd(ButterFilter_2nd::BF_settings::FS100FC30);
  geometry_msgs::PointStamped pixel_pos;
  cv::Mat blurred,im_with_keypoints;
  void saveImage(cv::Mat&);
  std::string Path = "/home/osama/noDetectionFrames/frame";
  int imageIndex = 0;


  const std::string OPENCV_WINDOW = "Image window";

  KeypointsDetection(ros::NodeHandle &);
  ~KeypointsDetection();

 void findCenter(cv::Mat*, std_msgs::Header);
};