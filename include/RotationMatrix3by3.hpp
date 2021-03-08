#pragma once
#include <eigen3/Eigen/Dense>
#include "Matrix3by3.hpp"
#include <math.h>

class RotationMatrix3by3 : public Matrix3by3 {
    private:
        Eigen::Matrix<float, 3, 3> _m;
    public:
        RotationMatrix3by3();
        ~RotationMatrix3by3();
        Eigen::Matrix<float, 3, 3> Update(Vector3D<float>);
};
