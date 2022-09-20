#include "SnowVision.h"

void Find_Point(std::vector<Eigen::Vector2d>& imagepoints, cv::Mat processed_image)
{
	std::vector<std::vector<cv::Point>> contours;
	cv::findContours(processed_image, contours, cv::noArray(), cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
	if (!contours.size())
		return;
	for (int i = 0; i < contours.size(); i++)
	{
		cv::Moments M = cv::moments(contours[i]);
		double area = cv::contourArea(contours[i]);
		if (area > 0)
			imagepoints.push_back(Eigen::Vector2d(M.m10 / M.m00, M.m01 / M.m00));
	}
}