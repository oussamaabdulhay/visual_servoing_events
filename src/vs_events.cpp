#include "vs_events.hpp"


vs_events::vs_events(ros::NodeHandle &main_nodehandle)
{   
    
      
    nh_=main_nodehandle;
    point_sub = new message_filters::Subscriber<geometry_msgs::PointStamped>(nh_, "/center_position", 100); //2 
    attitude_sub = new message_filters::Subscriber<geometry_msgs::QuaternionStamped>(nh_, "/filter/quaternion", 100); //8
    pose_sub = new message_filters::Subscriber<geometry_msgs::PoseStamped>(nh_, "/Robot_1/pose", 100); //8
    sync = new message_filters::Synchronizer<sync_poilicy>(sync_poilicy(10),*point_sub, *attitude_sub, *pose_sub); //8
    sync->registerCallback(boost::bind(&vs_events::ImageProcess, this, _1, _2, _3)); 

    drone_pose = nh_.advertise<geometry_msgs::PointStamped>("/vs_position", 1);
    drone_position_with_offset = nh_.advertise<geometry_msgs::Point>("/drone_position_with_offset", 1);
    all_pose = nh_.advertise<geometry_msgs::Pose>("/pose", 1);


    f_c=351.54;
    p_drone_camera.x = 0.122; //-0.038;
    p_drone_camera.y = 0.02; //0.1308;
    p_drone_camera.z = -0.111; //-0.1137;


}

vs_events::~vs_events()
{
}

void vs_events::ImageProcess(const geometry_msgs::PointStampedConstPtr& msg, const geometry_msgs::QuaternionStampedConstPtr& attitude, const geometry_msgs::PoseStampedConstPtr& pose)
{
    //std::cout<<"IN\n";
    quatToEuler(attitude);
    getDepthYaw(pose);
    geometry_msgs::Pose check;
    check.position.x = pos_data.x;
    check.position.y = pos_data.y;
    check.position.z = pos_data.z;

    check.orientation.x = drone_orientation.x;
    check.orientation.y = drone_orientation.y;
    check.orientation.z = drone_orientation.z;

    all_pose.publish(check);

    pixel_pos.x = msg->point.x;
    pixel_pos.y = msg->point.y;
    rotate_camera_vector(pixel_pos);
}


void vs_events::rotate_camera_vector(Vector3D<float> pixel_pos)
{

    Vector3D<float> camera_pixel_vector;
    camera_pixel_vector.x = f_c;
    camera_pixel_vector.y = -1 * pixel_pos.x;
    camera_pixel_vector.z = pixel_pos.y;

    RotationMatrix3by3 R_d_i;
    Eigen::Matrix<float, 3, 3> R_d_to_i_temp;
    R_d_to_i_temp = R_d_i.Update(drone_orientation); //Create the rotation matrices
    R_d_to_i_temp.transposeInPlace(); //drone to inertial
    
    Vector3D<float> t_results;
    t_results.x = camera_pixel_vector.x * R_d_to_i_temp(0, 0) + camera_pixel_vector.y * R_d_to_i_temp(0, 1) + camera_pixel_vector.z * R_d_to_i_temp(0, 2);
    t_results.y = camera_pixel_vector.x * R_d_to_i_temp(1, 0) + camera_pixel_vector.y * R_d_to_i_temp(1, 1) + camera_pixel_vector.z * R_d_to_i_temp(1, 2);
    t_results.z = camera_pixel_vector.x * R_d_to_i_temp(2, 0) + camera_pixel_vector.y * R_d_to_i_temp(2, 1) + camera_pixel_vector.z * R_d_to_i_temp(2, 2);

    get_object_location(t_results);

}



Vector3D<float> vs_events::rotate_offset()    
{
    Eigen::Matrix<float, 3, 3> R_body_to_inertial_temp(3, 3);
    RotationMatrix3by3 R_b_i;

    R_body_to_inertial_temp = R_b_i.Update(drone_orientation); //Create the rotation matrices
    R_body_to_inertial_temp.transposeInPlace(); //drone to inertial

    Vector3D<float> t_results;
    t_results.x = p_drone_camera.x * R_body_to_inertial_temp(0, 0) + p_drone_camera.y * R_body_to_inertial_temp(0, 1) + p_drone_camera.z * R_body_to_inertial_temp(0, 2);
    t_results.y = p_drone_camera.x * R_body_to_inertial_temp(1, 0) + p_drone_camera.y * R_body_to_inertial_temp(1, 1) + p_drone_camera.z * R_body_to_inertial_temp(1, 2);
    t_results.z = p_drone_camera.x * R_body_to_inertial_temp(2, 0) + p_drone_camera.y * R_body_to_inertial_temp(2, 1) + p_drone_camera.z * R_body_to_inertial_temp(2, 2);

    return t_results;
}


Vector3D<float> vs_events::get_object_location(Vector3D<float> rotated_pixel_vector)    
{

    Vector3D<float> object_location;

    object_location.x = (rotated_pixel_vector.x * depth) / rotated_pixel_vector.y;
    object_location.y = depth;
    object_location.z = (rotated_pixel_vector.z * depth) / rotated_pixel_vector.y;

    geometry_msgs::PointStamped object_pos;
    object_pos.point.x = -1 * object_location.x;
    object_pos.point.y = -1 * object_location.y;
    object_pos.point.z = object_location.z;

    drone_pose.publish(object_pos);

    Vector3D<float> rotated_offset = rotate_offset();

    // std::cout<<rotated_offset.x<<std::endl;
    // std::cout<<rotated_offset.y<<std::endl;
    // std::cout<<rotated_offset.z<<std::endl;

    geometry_msgs::Point object_pos_with_offset;
    object_pos_with_offset.x = (object_location.x - rotated_offset.x) * -1; 
    object_pos_with_offset.y = (object_location.y - rotated_offset.y) * -1;
    object_pos_with_offset.z = (object_location.z + rotated_offset.z);


    
    
    drone_position_with_offset.publish(object_pos_with_offset);

    return object_location;
}

 void vs_events::quatToEuler(const geometry_msgs::QuaternionStampedConstPtr& msg_attitude)
 {

    geometry_msgs::Quaternion att_data;

    att_data.x = msg_attitude->quaternion.x;
    att_data.y = msg_attitude->quaternion.y;
    att_data.z = msg_attitude->quaternion.z;
    att_data.w = msg_attitude->quaternion.w;


    Vector3D<float> _euler;

    double sinr_cosp = +2.0 * ( att_data.w * att_data.x + att_data.y * att_data.z);
    double cosr_cosp = +1.0 - 2.0 * (att_data.x * att_data.x + att_data.y * att_data.y);
    _euler.x = atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = +2.0 * ( att_data.w * att_data.y - att_data.z * att_data.x);
    if (fabs(sinp) >= 1)
        _euler.y = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        _euler.y = asin(sinp);

    Vector3D<double> orientation_euler;
    orientation_euler.x = _euler.x;
    orientation_euler.y = _euler.y;
    orientation_euler.z = _euler.z;

    drone_orientation.x = orientation_euler.x;
    drone_orientation.y = orientation_euler.y;
 }

 void vs_events::getDepthYaw(const geometry_msgs::PoseStampedConstPtr& msg_pose)
 {
    
    pos_data.x = -1 * msg_pose->pose.position.y;
    pos_data.y = msg_pose->pose.position.x;
    pos_data.z = msg_pose->pose.position.z;
    geometry_msgs::Quaternion att_data;
    att_data.x = msg_pose->pose.orientation.x;
    att_data.y = msg_pose->pose.orientation.y;
    att_data.z = msg_pose->pose.orientation.z;
    att_data.w = msg_pose->pose.orientation.w;

    depth = 3.15 - pos_data.y; //3.15 location of object plane
    Vector3D<float> _euler;
    // yaw (z-axis rotation)
    double siny_cosp = +2.0 * (att_data.w * att_data.z + att_data.x * att_data.y);
    double cosy_cosp = +1.0 - 2.0 * (att_data.y * att_data.y + att_data.z * att_data.z);  
    _euler.z = atan2(siny_cosp, cosy_cosp);

    _euler.z -= -90.*(M_PI/180.);
    drone_orientation.z = _euler.z;
}