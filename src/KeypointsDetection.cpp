#include "KeypointsDetection.hpp"

KeypointsDetection::KeypointsDetection(ros::NodeHandle &t_nh):it_(nh_)
{

    nh_ = t_nh;
    pixel_center_location = nh_.advertise<geometry_msgs::PointStamped>("/center_position", 10);
    pixel_center_location_filtered = nh_.advertise<geometry_msgs::PointStamped>("/center_position_filtered", 10);
    //diff_unfiltered_pub = nh_.advertise<geometry_msgs::PointStamped>("/diff_unfiltered", 10);
    //diff_filtered_pub = nh_.advertise<geometry_msgs::PointStamped>("/diff_filtered", 10);


    params.filterByArea = true;
    params.minArea = 400;
    params.maxArea = 2000;

    // Filter by Circularity
    params.filterByCircularity = true;
    params.minCircularity = 0.6;

    // Filter by Convexity
    params.filterByConvexity = false;
    params.minConvexity = 0.2;

    // Filter by Inertia
    params.filterByInertia = false;
    params.minInertiaRatio = 0.2;

    threshold = 10000;
    _dt = 1/.100;
    
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
        
        pixel_pos.point.x = center_point.x-172.62;
        pixel_pos.point.y = center_point.y-126.6;
        pixel_pos.point.z = 0;
        pixel_pos.header = EventImageHeader;
    
        pixel_center_location.publish(pixel_pos);

        // float diff_x = (pixel_pos.point.x - pixel_pos_prev.point.x) / _dt;
        // float diff_y = (pixel_pos.point.y - pixel_pos_prev.point.y) / _dt;
        // geometry_msgs::PointStamped diff_unfiltered;
        // diff_unfiltered.point.x = diff_x;
        // diff_unfiltered.point.y = diff_y;
        // diff_unfiltered.header = EventImageHeader;

        // diff_unfiltered_pub.publish(diff_unfiltered);

        geometry_msgs::PointStamped pixel_pos_filtered;
        pixel_pos_filtered.point.x = filter_x->runTask(pixel_pos.point.x);
        pixel_pos_filtered.point.y = filter_y->runTask(pixel_pos.point.y);
        pixel_pos_filtered.point.z = 0;

        pixel_pos_filtered.header = EventImageHeader;
        
        pixel_center_location_filtered.publish(pixel_pos_filtered);

        // float diff_x_fil = (pixel_pos_filtered.point.x - pixel_pos_filtered_prev.point.x) / _dt;
        // float diff_y_fil = (pixel_pos_filtered.point.y - pixel_pos_filtered_prev.point.y) / _dt;
        // geometry_msgs::PointStamped diff_filtered;
        // diff_filtered.point.x = diff_x_fil;
        // diff_filtered.point.y = diff_y_fil;
        // diff_filtered.header = EventImageHeader;
        

        // diff_filtered_pub.publish(diff_filtered);

        // pixel_pos_prev.point.x = pixel_pos.point.x;
        // pixel_pos_prev.point.y = pixel_pos.point.y;
        // pixel_pos_prev.point.z = 0;

        // pixel_pos_filtered_prev.point.x = pixel_pos_filtered.point.x;
        // pixel_pos_filtered_prev.point.y = pixel_pos_filtered.point.y;
        // pixel_pos_filtered_prev.point.z = 0;
  }
    
  
    // cv::imshow("im_with_keypoints", im_with_keypoints); 
    // cv::waitKey(1);  

}
}

// void KeypointsDetection::saveImage(cv::Mat &img )
// {
//   Path=Path+std::to_string(imageIndex)+ ".jpg";
//   cv::imwrite(Path,img);
//   Path="/home/osama/noDetectionFrames/frame";
//   imageIndex++;
// }