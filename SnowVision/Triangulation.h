#pragma once
#include "SnowVision.h"
#include "BasicClasses.h"

void ImagePoints2TriangulatedPoints(std::vector<Eigen::Vector3d>& TriangulatedPoints, const std::vector<std::vector<Eigen::Vector2d>>& ImagePoints, const CameraGroup Cameras, const double distance_tol);

void Split_Camera_Rt(Eigen::MatrixXd& output_R, Eigen::MatrixXd& output_t, Eigen::MatrixXd Rt);

void Get_H(Eigen::MatrixXd& output, Eigen::Vector3d f_main, Eigen::Vector3d f_sub, Eigen::MatrixXd R_sub);

void Get_S_Star(Eigen::MatrixXd& output, const Eigen::Vector3d& f_main, const Eigen::Vector3d& f_sub, const Eigen::MatrixXd& Rt);

void Get_W_star(Eigen::MatrixXd& output, Eigen::MatrixXd s_star, Eigen::Vector3d f_main, Eigen::MatrixXd f_sub);

void Get_W_sub_W_main(Eigen::MatrixXd& output1, Eigen::MatrixXd& output2, Eigen::MatrixXd w_star);

void W_TO_World_Coordination(Eigen::MatrixXd& output, Eigen::MatrixXd w_sub, Eigen::MatrixXd Rt_sub);

void Get_F_Group(std::vector<std::vector<Eigen::Vector3d>>& output, std::vector<std::vector<Eigen::Vector2d>> Points, CameraGroup Cameras);

