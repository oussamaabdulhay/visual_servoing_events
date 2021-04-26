#include "KeypointsDetection.hpp"

KeypointsDetection::KeypointsDetection(ros::NodeHandle &t_nh):it_(nh_)
{

    nh_ = t_nh;
    pixel_center_location = nh_.advertise<geometry_msgs::PointStamped>("/center_position", 10);


    params.filterByArea = true;
    params.minArea = 400;
    params.maxArea = 1500;

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
    //blurred = *EventsImage;
    cv::GaussianBlur(*EventsImage, blurred, cv::Size(5,5), 0, 0);
    //cv::medianBlur(*EventsImage, blurred,3);
    // cv::threshold(blurred, blurred, 0,255,cv::THRESH_BINARY);
    // cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE ,cv::Size(9,9) );
    // cv::morphologyEx(blurred, blurred, 3 , element);
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
    //float std_dev;
    //list_of_positions.push_back(keypoints[0].pt);

    //if (list_of_positions.size() == 3)
    //{
      //std_dev = filter->getStdDev(list_of_positions);
      //if (std_dev < threshold)
      //{
        center_point.x = keypoints[0].pt.x;
        center_point.y = keypoints[0].pt.y;
        
        pixel_pos.point.x = center_point.x-175.35;
        pixel_pos.point.y = center_point.y-133.17;
        pixel_pos.point.z = 0;
        pixel_pos.header = EventImageHeader;

        // pixel_pos.point.x = std::trunc(pixel_pos.point.x);
        // pixel_pos.point.y = std::trunc(pixel_pos.point.y);

     
        pixel_center_location.publish(pixel_pos);
      //}
    //   else
    //    {
    //     std::cout << "standard dev too high\n";
    //     center_point = filter->getMedian(list_of_positions, center_point);

    //     pixel_pos.point.x = center_point.x-105.8;
    //     pixel_pos.point.y = center_point.y-92.3;
    //     pixel_pos.point.z = 0;
    //     pixel_pos.header = EventImageHeader;
    //    }
    // list_of_positions.erase(list_of_positions.begin());
    // }
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