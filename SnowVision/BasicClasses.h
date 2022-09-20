#pragma once
#include "SnowVision.h"
#include "FileProcessor.h"

class Camera
{
public:
    Eigen::MatrixXd K;
    Eigen::MatrixXd K_Inverse;
    Eigen::MatrixXd Rt;
    Eigen::MatrixXd D;
    cv::Mat K_Mat;
    cv::Mat D_Mat;
    Camera() :K(3, 3), K_Inverse(3, 3), Rt(3, 4), D(1, 5) {}
};

class CameraGroup
{
public:
    int CameraCount;
    std::vector<Camera> CameraInfo;
    CameraGroup(std::string filepath)
    {
        std::vector<std::string> lines = ReadLine(txt2String(filepath));
        CameraCount = std::stoi(Split(lines[0], "=")[1]);
        for (int i = 0; i < CameraCount; i++)
        {
            Camera cam;
            int index = 1 + (i * 4);
            Read_2DArray(cam.K, lines[index + 1]);
            cam.K_Inverse = cam.K.inverse();
            Read_2DArray(cam.Rt, lines[index + 2]);
            Read_2DArray(cam.D, lines[index + 3]);
            cam.K_Mat = (cv::Mat_<double>(3, 3) << cam.K(0, 0), cam.K(0, 1), cam.K(0, 2),
                                                   cam.K(1, 0), cam.K(1, 1), cam.K(1, 2),
                                                   cam.K(2, 0), cam.K(2, 1), cam.K(2, 2));
            cam.D_Mat = (cv::Mat_<double>(5, 1) << cam.D(0, 0), cam.D(0, 1), cam.D(0, 2), cam.D(0, 3), cam.D(0, 4));
            CameraInfo.push_back(cam);
        }
    }
};

class ImagePoints
{
public:
    std::vector<Eigen::Vector2d> ImagePointsForEachCamera;
    void Clear()
    {
        ImagePointsForEachCamera.clear();
    }
    int Point_Count()
    {
        int pointcount = 0;
        for (int i = 0; i < ImagePointsForEachCamera.size(); i++)
        {
            pointcount += ImagePointsForEachCamera[i].size();
        }
        return pointcount;
    }
};

class PointGroup
{
public:
    std::vector<ImagePoints> PointsGroup;
};