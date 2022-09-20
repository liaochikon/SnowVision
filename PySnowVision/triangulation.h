#pragma once
#include "pysnowvision.h"

class Camera
{
public:
    Eigen::MatrixXd K;
    Eigen::MatrixXd K_Inverse;
    Eigen::MatrixXd Rt;
    Eigen::MatrixXd R;
    Eigen::MatrixXd t;
    Eigen::MatrixXd D;
    Camera() :K(3, 3), K_Inverse(3, 3), Rt(3, 4), R(3, 3), t(3, 1), D(1, 5) {}
};

class CameraGroup
{
public:
    int CameraCount = 0;
    std::vector<Camera> CameraInfo;
    std::vector<std::vector<Eigen::Vector2d>> CameraPoints;
    std::vector<Eigen::Vector3d> TriangulatedPoints;
    std::vector<double> TriangulatedScore;


    CameraGroup(std::string filepath);

    void Triangulation(const double distance_tol);
    void AddCameraPoints(const std::vector<Eigen::Vector2d>& camerapoints);
    void ClearCameraPoints();
    void ClearTriangulatedPoints();

    void TriangulationCondense(const double distance_tol);
    void TriangulationWorkspaceFilter(const std::vector<double> workspace);
};