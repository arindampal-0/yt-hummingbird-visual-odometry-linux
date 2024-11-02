#pragma once

#include <string>
#include <vector>

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"


void getCalibrationData(std::string imagesetPath, double& focal,
                        cv::Point2d& pp);

void featureTracking(cv::Mat img1, cv::Mat img2,
                     std::vector<cv::Point2f>& points1,
                     std::vector<cv::Point2f>& points2,
                     std::vector<uchar>& status);

void featureDetection(cv::Mat img, std::vector<cv::Point2f>& points);