#include "KeypointsDetection.hpp"

KeypointsDetection::KeypointsDetection(ros::NodeHandle &t_nh):it_(nh_)
{

    nh_ = t_nh;
    pixel_center_location = nh_.advertise<geometry_msgs::PointStamped>("/center_position", 10);
    pixel_center_location_filtered = nh_.advertise<geometry_msgs::PointStamped>("/center_position_filtered", 10);


    params.filterByArea = true;
    params.minArea = 400;
    params.maxArea = 2000;

    // Filter by Circularity
    params.filterByCircularity = true;
    params.minCircularity = 0.5;

    // Filter by Convexity
    params.filterByConvexity = false;
    params.minConvexity = 0.1;

    // Filter by Inertia
    params.filterByInertia = false;
    params.minInertiaRatio = 0.1;

    threshold = 10000;
    
}

KeypointsDetection::~KeypointsDetection()
{
}

void KeypointsDetection::findCenter(cv::Mat* EventsImage, std_msgs::Header EventImageHeader)
{   
    if(EventsImage->empty())
    {
        
    }
    else
    {
    cv::GaussianBlur(*EventsImage, blurred, cv::Size(5,5), 0, 0);
    cv::bitwise_not(blurred, blurred);
    cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);
    detector->detect(blurred, keypoints);
    cv::drawKeypoints(blurred, keypoints, im_with_keypoints, cv::Scalar(0,0,255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    std::cout<<keypoints.size()<<std::endl;
    //std::cout<<keypoints[0].size<<std::endl;
    

    if (keypoints.size() == 0)
    {
    std::cout << "EMPTY KEYPOINTS\n";
    //saveImage(im_with_keypoints);
    }

    else
    {
        center_point.x = keypoints[0].pt.x;
        center_point.y = keypoints[0].pt.y;
        
        pixel_pos.point.x = center_point.x-174.06;
        pixel_pos.point.y = center_point.y-133.044;
        pixel_pos.point.z = 0;
        pixel_pos.header = EventImageHeader;
    
        pixel_center_location.publish(pixel_pos);

        geometry_msgs::PointStamped pixel_pos_filtered;
        pixel_pos_filtered.point.x = filter_x->runTask(pixel_pos.point.x);
        pixel_pos_filtered.point.y = filter_y->runTask(pixel_pos.point.y);
        pixel_pos_filtered.point.z = 0;

        pixel_pos_filtered.header = EventImageHeader;
        
        pixel_center_location_filtered.publish(pixel_pos_filtered);
  }
    
  
    cv::imshow("im_with_keypoints", im_with_keypoints); 
    cv::waitKey(1);  

}
}

// void KeypointsDetection::saveImage(cv::Mat &img )
// {
//   Path=Path+std::to_string(imageIndex)+ ".jpg";
//   cv::imwrite(Path,img);
//   Path="/home/osama/noDetectionFrames/frame";
//   imageIndex++;
// }