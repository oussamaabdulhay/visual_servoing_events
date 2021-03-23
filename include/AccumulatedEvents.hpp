#include <iostream>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "sensor_msgs/Image.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <dvs_msgs/EventArray.h>
#include <dvs_msgs/Event.h>
#include "KeypointsDetection.hpp"


class AccumulatedEvents
{
  public:
  ros::NodeHandle nh_;
  ros::Subscriber sub;
  image_transport::ImageTransport it_;
  image_transport::Publisher pub;
  cv::Mat EventsImage;
  KeypointsDetection* keypoints_locater=new KeypointsDetection(nh_);
  sensor_msgs::Image img_msg;
  int i = 0;
  ros::Time packet_avg_time;
 
  const std::string OPENCV_WINDOW = "Image window";

  AccumulatedEvents(ros::NodeHandle &);
  ~AccumulatedEvents();

  void Events(dvs_msgs::EventArray msg);
};