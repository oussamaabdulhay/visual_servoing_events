#include "AccumulatedEvents.hpp"
#include "vs_events.hpp"
#include <ros/ros.h>
#include <opencv2/core/types.hpp>




int main(int argc, char** argv)
{


ros::init(argc, argv, "visual_servoing_events_node");
ros::NodeHandle main_nodehandle;

AccumulatedEvents* visualisation=new AccumulatedEvents(main_nodehandle);
vs_events* localisation=new vs_events(main_nodehandle);

// ros::Rate r(200);
// while (ros::ok())
// {
//   r.sleep();
//   ros::spinOnce();
// }
ros::spin();

return 0;

}
