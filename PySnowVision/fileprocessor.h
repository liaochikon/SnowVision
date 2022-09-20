#pragma once
#include "pysnowvision.h"

std::vector<std::string> Split(std::string fulltext, std::string split_key);

std::vector<std::string> ReadLine(std::string fulltext);

std::string txt2String(std::string filepath);

void Read_2DArray(Eigen::MatrixXd& mat, std::string line);

int Load_Images(std::vector<cv::VideoCapture>& cap_array, std::string imagepath, int cameracount);

int Load_Videos(std::vector<cv::VideoCapture>& cap_array, std::string videopath, int cameracount);