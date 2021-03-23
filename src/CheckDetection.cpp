#include "CheckDetection.hpp"

CheckDetection::CheckDetection()
{

    //cv::namedWindow(OPENCV_WINDOW);

    params.filterByArea = true;
    params.minArea = 700;
    params.maxArea = 2000;

    // Filter by Circularity
    params.filterByCircularity = false;
    params.minCircularity = 0.2;

    // Filter by Convexity
    params.filterByConvexity = false;
    params.minConvexity = 0.2;

    // Filter by Inertia
    params.filterByInertia = false;
    params.minInertiaRatio = 0.2;

    
}

CheckDetection::~CheckDetection()
{
    //cv::destroyWindow(OPENCV_WINDOW);
}

void CheckDetection::check(cv::Mat img)
{   
    orignial_image = img;
    if(orignial_image.empty())
    {
        std::cout<<"EMPTY IMAGE";
    }
    else
    {
    tmp = orignial_image;
    cv::GaussianBlur(orignial_image, tmp, cv::Size(5, 5), 0, 0);
    //cv::bitwise_not(tmp, tmp);
    cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);
    detector->detect(tmp, keypoints);
    cv::drawKeypoints(tmp, keypoints, im_with_keypoints, cv::Scalar(0,0,255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    cv::imshow("im_with_keypoints", im_with_keypoints); 
    cv::waitKey(5000);  
    //std::cout<<keypoints[0].size<<std::endl;
}
}

int main1(int argc, char** argv)
{
  std::string Path = "/home/osama/noDetectionFrames/frame39.jpg";
  cv::Mat img = cv::imread(Path, cv::IMREAD_GRAYSCALE);
  ros::init(argc, argv, "CheckDetection_node");
  ros::NodeHandle main_nodehandle;
  
  CheckDetection* Detection=new CheckDetection();
  Detection->check(img);  
  
  while (ros::ok())
  {
    ros::spinOnce();
     
  }
  
  
  return 0;
}