#include "AccumulatedEvents.hpp"
#include "KeypointsDetection.hpp"
#include <ros/ros.h>
#include <opencv2/core/types.hpp>



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
