#include "KeypointsDetection.hpp"

KeypointsDetection::KeypointsDetection(ros::NodeHandle &t_nh):it_(nh_)
{

    nh_ = t_nh;
    pixel_center_location = nh_.advertise<geometry_msgs::PointStamped>("/center_position", 1);

    cv::namedWindow(OPENCV_WINDOW);

    params.filterByArea = true;
    params.minArea = 10;
    params.maxArea = 30;

    // Filter by Circularity
    params.filterByCircularity = false;
    params.minCircularity = 0.5;

    // Filter by Convexity
    params.filterByConvexity = false;
    params.minConvexity = 0.5;

    // Filter by Inertia
    params.filterByInertia = false;
    params.minInertiaRatio = 0.6;

    threshold = 10000.0;
    
    f_c=616.5;  //TODO: Change to events camera fc
    // p_drone_camera.x = -0.038;
    // p_drone_camera.y = 0.1308;
    // p_drone_camera.z = -0.1137;
}

KeypointsDetection::~KeypointsDetection()
{
    cv::destroyWindow(OPENCV_WINDOW);
}

void KeypointsDetection::findCenter(cv::Mat* EventsImage, ros::Time EventImageTime)
{   
    if(EventsImage->empty())
    {
        
    }
    else
    {
    cv::medianBlur(*EventsImage, blurred, 3);
    cv::bitwise_not(blurred, blurred);
    cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);
    detector->detect(blurred, keypoints);
    cv::drawKeypoints(blurred, keypoints, im_with_keypoints, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    std::cout<<keypoints.size()<<std::endl;

    if (keypoints.size() == 0)
    {
    std::cout << "EMPTY KEYPOINTS\n";
    }

    else
    {
    float std_dev;
    list_of_positions.push_back(keypoints[0].pt);

    if (list_of_positions.size() == 3)
    {
      std_dev = filter->getStdDev(list_of_positions);
      if (std_dev < threshold)
      {
        center_point.x = list_of_positions.back().x;
        center_point.y = list_of_positions.back().y;
        
        pixel_pos.point.x = center_point.x-173;
        pixel_pos.point.y = center_point.y-130;
        pixel_pos.header.stamp = EventImageTime;

        geometry_msgs::Point pixel_pub;
        pixel_pub.x=pixel_pos.point.x;
        pixel_pub.y=pixel_pos.point.y;
        
        pixel_center_location.publish(pixel_pos);
      }
      else
       {
        std::cout << "standard dev too high\n";
        center_point = filter->getMedian(list_of_positions, center_point);

        pixel_pos.point.x = center_point.x-173;
        pixel_pos.point.y = center_point.y-130;
        pixel_pos.header.stamp = EventImageTime;

      
        pixel_center_location.publish(pixel_pos);
       }
    list_of_positions.erase(list_of_positions.begin());
    }
  }
    cv::imshow("im_with_keypoints", im_with_keypoints); 
    cv::waitKey(1);    
}
}