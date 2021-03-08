#include "RotationMatrix3by3.hpp"


RotationMatrix3by3::RotationMatrix3by3(/* args */){
}

RotationMatrix3by3::~RotationMatrix3by3(){
}


Eigen::Matrix<float, 3, 3> RotationMatrix3by3::Update(Vector3D<float> t_input)
{ 

    float sr = sin(t_input.x);
    float cr = cos(t_input.x);

    float sp = sin(t_input.y);
    float cp = cos(t_input.y);

    float sy = sin(t_input.z);
    float cy = cos(t_input.z);
    
    Matrix3by3::v1.x =cy * cp;
    Matrix3by3::v1.y =sy * cp;
    Matrix3by3::v1.z =sp * -1;

    Matrix3by3::v2.x =cy * sp * sr - sy * cr;
    Matrix3by3::v2.y =sy * sp * sr + cy * cr;
    Matrix3by3::v2.z =cp * sr;

    Matrix3by3::v3.x =cy * sp * cr + sy * sr;
    Matrix3by3::v3.y =sy * sp * cr - cy * sr;
    Matrix3by3::v3.z =cp * cr;

    this->Transpose();
    /* 
    
    Original Rotation Matrix for ENU coordinate system. 
    Matrix3by3::v1.x =cp * cy;
    Matrix3by3::v1.y =cp * sy;
    Matrix3by3::v1.z =sp * -1;
    Matrix3by3::v2.x =sr * sp * cy - cr * sy;
    Matrix3by3::v2.y =sr * sp * sy + cr * cy;
    Matrix3by3::v2.z =cp * sr;
    Matrix3by3::v3.x =cr * sp * cy + sr * sy;
    Matrix3by3::v3.y =cr * sp * sy - sr * cy;
    Matrix3by3::v3.z =cp * cr;
    */

   _m << v1.x, v2.x, v3.x,
         v1.y, v2.y, v3.y,
         v1.z, v2.z, v3.z;

    return _m;
}