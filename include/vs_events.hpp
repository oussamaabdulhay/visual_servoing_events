#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/QuaternionStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Point.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <cmath>
#include "geometry_msgs/Point.h"
#include <sstream>
#include <iostream>
#include "medianFilter.hpp"
#include "geometry_msgs/Point.h"
#include <sys/stat.h>
#include "RotationMatrix3by3.hpp"
#include <math.h>
#include <eigen3/Eigen/Dense>
#include <ros/ros.h>


class vs_events
{
  public:
    ros::NodeHandle nh_;
    typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseStamped,geometry_msgs::QuaternionStamped,geometry_msgs::PoseStamped> sync_poilicy;
    message_filters::Synchronizer<sync_poilicy> *sync;
    message_filters::Subscriber<geometry_msgs::PoseStamped> *image1_sub;
    message_filters::Subscriber<geometry_msgs::QuaternionStamped> *attitude_sub;
    message_filters::Subscriber<geometry_msgs::PoseStamped> *pose_sub;
    ros::Publisher drone_pose, drone_position_with_offset, all_pose;
    float  f_c, depth;
    Vector3D<float> drone_orientation,p_drone_camera,pixel_pos;
    Vector3D<float> pos_data;

    void ImageProcess(const geometry_msgs::PoseStampedConstPtr& ,const geometry_msgs::QuaternionStampedConstPtr&, const geometry_msgs::PoseStampedConstPtr&);    
    void rotate_camera_vector(Vector3D<float> );
    void update_rotation_matrices(Vector3D<float>);
    Vector3D<float> rotate_offset();
    Vector3D<float> get_object_location(Vector3D<float>);
    void quatToEuler(const geometry_msgs::QuaternionStampedConstPtr&);
    void getDepthYaw(const geometry_msgs::PoseStampedConstPtr&);

        
    vs_events(ros::NodeHandle&);
    ~vs_events();

};