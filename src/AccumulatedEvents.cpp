#include "AccumulatedEvents.hpp"

AccumulatedEvents::AccumulatedEvents(ros::NodeHandle &t_nh):it_(nh_)
{

    nh_ = t_nh;
    sub = nh_.subscribe("/dvs/events", 1, &AccumulatedEvents::Events,this);
    pub=it_.advertise("/EventsImage",1);
    
    cv::namedWindow(OPENCV_WINDOW);
}
AccumulatedEvents::~AccumulatedEvents()
{
    cv::destroyWindow(OPENCV_WINDOW);
}

void AccumulatedEvents::Events(dvs_msgs::EventArray msg)
{
    EventsImage = cv::Mat::zeros(260, 346, CV_8UC1);
    
    cv_bridge::CvImage img_bridge;
    
    dvs_msgs::Event first_event = msg.events.front();
    dvs_msgs::Event last_event = msg.events.back();
    ros::Time start = first_event.ts;
    ros::Time end = last_event.ts;
     
    ros::Time packet_avg_time = start + (end - start) *0.5; 
    for (int i = 0; i < msg.events.size(); i++)
    {
        cv::circle(EventsImage, cv::Point(msg.events[i].x, msg.events[i].y), 0.5, cv::Scalar(255, 0, 0), -1, 8);
    }
    std_msgs::Header header;
    header.seq = i++;
    header.stamp= packet_avg_time;
    img_bridge=cv_bridge::CvImage(header, sensor_msgs::image_encodings::MONO8, EventsImage);
    img_bridge.toImageMsg(img_msg);
    pub.publish(img_msg);
   
    cv::imshow("EventsImage", EventsImage);
    cv::waitKey(1);   
}
