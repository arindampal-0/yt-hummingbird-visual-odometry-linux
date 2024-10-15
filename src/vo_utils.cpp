#include "vo_utils.h"
#include "opencv2/core/types.hpp"

#include <opencv2/video/tracking.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <filesystem>
#include <fstream>
#include <iterator>
#include <sstream>
#include <string>

/* Load these values from Kitty's calibration files */
void getCalibrationData(std::string datasetPath, double& focal, cv::Point2d& pp)
{
    std::filesystem::path calibrationFilePath = datasetPath;
    calibrationFilePath /= "calib.txt";

    std::ifstream myFile(calibrationFilePath);

    if (myFile.is_open())
    {
        std::string line;
        while (getline(myFile, line))
        {
            std::istringstream iss(line);
            std::vector<std::string> results(
                (std::istream_iterator<std::string>(iss)),
                std::istream_iterator<std::string>());

            focal = std::stod(results[1]);
            pp.x = std::stod(results[3]);
            pp.y = std::stod(results[7]);

            break;
        }

        myFile.close();
    }
}

void featureTracking(cv::Mat img1, cv::Mat img2,
                     std::vector<cv::Point2f>& points1,
                     std::vector<cv::Point2f>& points2,
                     std::vector<uchar>& status)
{
    std::vector<float> err;
    cv::Size windowSize = cv::Size(21, 21);
    cv::TermCriteria termCriteria = cv::TermCriteria(
        cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01);

    cv::calcOpticalFlowPyrLK(img1, img2, points1, points2, status, err,
                             windowSize, 3, termCriteria, 0, 0.001);

    /* Getting rid of points for which the KLT tracking failed
        or those who have gone outside the frame. */
    int indexCorrection = 0;
    for (int i = 0; i < status.size(); ++i)
    {
        cv::Point2f pt = points2.at(i - indexCorrection);
        if ((status.at(i) == 0) || (pt.x < 0) || (pt.y < 0))
        {
            if ((pt.x < 0) || (pt.y < 0))
            {
                status.at(i) = 0;
            }

            points1.erase(points1.begin() + (i - indexCorrection));
            points1.erase(points2.begin() + (i - indexCorrection));
            indexCorrection++;
        }
    }
}

void featureDetection(cv::Mat img, std::vector<cv::Point2f>& points) 
{
    /* Uses FAST for feature detection */
    std::vector<cv::KeyPoint> keypoints;
    int fastThreshold = 20;
    bool nonMaxSuppression = true;
    cv::FAST(img, keypoints, fastThreshold, nonMaxSuppression);
    cv::KeyPoint::convert(keypoints, points, std::vector<int>());
}