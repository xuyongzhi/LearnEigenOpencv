#ifndef EIGEN_CV_GEOMETRY_FUNS_H
#define EIGEN_CV_GEOMETRY_FUNS_H

#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <iostream>

using namespace Eigen;
using namespace cv;
using namespace std;

void Test_RotateVector();
void TestCoordinate();
void MyEuler_ZXY( const Eigen::Matrix3f& R_n2b,Eigen::Vector3f& euler_n2b );
void R2Euler( const Eigen::Matrix3f& R_n2b, const Eigen::Vector3i& eulerOrder ,Eigen::Vector3f& euler_n2b);
void Euler2R_sameOrder(const Vector3f& euler_n2b,const Vector3i& eulerOrder_n2b, Matrix3f& R_n2b);
void Euler2R(const Vector3f& euler_n2b,const Vector3i& eulerOrder_n2b, Matrix3f& R_b2n);



#endif EIGEN_CV_GEOMETRY_FUNS_H
