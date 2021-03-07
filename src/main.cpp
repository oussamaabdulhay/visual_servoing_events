#include "AccumulatedEvents.hpp"
#include "KeypointsDetection.hpp"
#include "geometry_msgs/PointStamped.h"
#include "sensor_msgs/Image.h"
#include <ros/ros.h>
#include <opencv2/core/types.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "sensor_msgs/Image.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/features2d.hpp>

int main(int argc, char** argv)
{


ros::init(argc, argv, "visual_servoing_events_node");
ros::NodeHandle main_nodehandle;

AccumulatedEvents* visualisation=new AccumulatedEvents(main_nodehandle);

KeypointsDetection* keypoints_locater=new KeypointsDetection(main_nodehandle);

cv::Mat* eventsImage = &(visualisation->EventsImage);




  ros::Rate r(200);
  while (ros::ok())
  {
    r.sleep();
    keypoints_locater->findCenter(eventsImage);
    ros::spinOnce();
  }
  return 0;
}
