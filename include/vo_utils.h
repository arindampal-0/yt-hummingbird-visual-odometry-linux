#pragma once

#include <vector>
#include <opencv2/core.hpp>


void getCalibrationData(std::string datasetPath, double& focal,
                        cv::Point2d& pp);

void featureTracking(cv::Mat img1, cv::Mat img2,
                     std::vector<cv::Point2f>& points1,
                     std::vector<cv::Point2f>& points2,
                     std::vector<uchar>& status);

void featureDetection(cv::Mat img, std::vector<cv::Point2f>& points);