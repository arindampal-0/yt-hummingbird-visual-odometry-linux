#include "vo.h"

#include <ctime>
#include <opencv2/core.hpp>
#include "opencv2/calib3d.hpp"
#include "opencv2/core/types.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>

#include "vo_utils.h"

#define MAX_FRAME 1000
#define MIN_NUM_FEAT 2000

double getAbsoluteScale(std::string datasetPosesPath, int frameId)
{

    std::ifstream file(datasetPosesPath);

    if (file.is_open())
    {
        std::string line;
        int i = 0;
        double x = 0, y = 0, z = 0;
        double xPrev, yPrev, zPrev;

        while ((std::getline(file, line)) && (i <= frameId))
        {
            xPrev = x;
            yPrev = y;
            zPrev = z;

            std::istringstream in(line);
            for (int j = 0; j < 12; ++j)
            {
                in >> z;
                if (j == 7)
                    y = z;

                if (j == 3)
                    x = z;
            }

            i++;
        }

        file.close();

        return std::sqrt((x - xPrev) * (x - xPrev) + (y - yPrev) * (y - yPrev) +
                         (z - zPrev) * (z - zPrev));
    }

    std::cerr << "Unable to open file\n";
    return 0;
}

int VisualOdometry::run(std::string datasetPath)
{
    cv::Mat img1, img2;
    cv::Mat Rf, tf;

    double scale = 1.00;
    char filename1[200];
    char filename2[200];
    sprintf(filename1,
            (std::filesystem::path(datasetPath) / "image_0/%06d.png").c_str(),
            0);
    sprintf(filename2,
            (std::filesystem::path(datasetPath) / "image_0/%06d.png").c_str(),
            1);

    char text[100];
    int fontFace = cv::FONT_HERSHEY_PLAIN;
    double fontScale = 1;
    int thickness = 1;
    cv::Point textOrg(10, 50);

    /* read the first two fromes from the dataset for initial setup */
    cv::Mat colorImg1 = cv::imread(filename1);
    cv::Mat colorImg2 = cv::imread(filename2);

    if (!colorImg1.data || !colorImg2.data)
    {
        std::cerr << "Error reading image!\n";
        return -1;
    }

    /* Convert to gray scale images */
    cv::cvtColor(colorImg1, img1, cv::COLOR_BGR2GRAY);
    cv::cvtColor(colorImg2, img2, cv::COLOR_BGR2GRAY);

    /* Feature detection, tracking */
    /* Vector to store feature points coordinates */
    std::vector<cv::Point2f> points1, points2;
    featureDetection(img1, points1);
    std::vector<uchar> status;
    featureTracking(img1, img2, points1, points2, status);

    double focal;
    cv::Point2d pp;
    getCalibrationData(datasetPath, focal, pp);

    /* Recovering the pose and the essential matrix */
    cv::Mat E, R, t, mask;
    E = cv::findEssentialMat(points2, points1, focal, pp, cv::RANSAC, 0.999, 1.0, mask);
    cv::recoverPose(E, points2, points1, R, t, focal, pp, mask);

    cv::Mat prevImage = img2;
    cv::Mat currImage;
    std::vector<cv::Point2f> prevFeatures = points2;
    std::vector<cv::Point2f> currFeatures;

    char filename[100];
    Rf = R.clone();
    tf = R.clone();

    clock_t begin = clock();

    cv::namedWindow("Road facing Egomotion camera", cv::WINDOW_AUTOSIZE);
    cv::namedWindow("Trajectory", cv::WINDOW_AUTOSIZE);

    cv::Mat traj = cv::Mat::zeros(600, 600, CV_8UC3);

    std::filesystem::path datasetPosesPath = std::filesystem::path(datasetPath) / "data_odometry_poses/dataset/poses/00.txt";

    for (int frameNum = 2; frameNum < MAX_FRAME; ++frameNum)
    {
        sprintf(filename, (std::filesystem::path(datasetPath) / "image_0/%06d.png").c_str(), frameNum);
        cv::Mat currColorImg = cv::imread(filename);
        cv::cvtColor(currColorImg, currImage, cv::COLOR_BGR2GRAY);

        std::vector<uchar> status;
        featureTracking(prevImage, currImage, prevFeatures, currFeatures, status);

        E = cv::findEssentialMat(currFeatures, prevFeatures, focal, pp, cv::RANSAC, 0.999, 1.0, mask);
        cv::recoverPose(E, currFeatures, prevFeatures, R, t, focal, pp, mask);
        cv::Mat prevPts(2, prevFeatures.size(), CV_64F), currPts(2, currFeatures.size(), CV_64F);

        for (size_t i = 0; i < prevFeatures.size(); ++i)
        {
            prevPts.at<double>(0, i) = prevFeatures.at(i).x;
            prevPts.at<double>(1, i) = prevFeatures.at(i).y;

            currPts.at<double>(0, i) = currFeatures.at(i).x;
            currPts.at<double>(1, i) = currFeatures.at(i).y;
        }

        scale = getAbsoluteScale(datasetPosesPath, frameNum);

        if ((scale > 0.1) && (t.at<double>(2) > t.at<double>(0)) && (t.at<double>(2) > t.at<double>(1)))
        {
            /* t_final = t_prev + scale * (R_prev * t_current)*/
            tf = tf + scale * (Rf * t);
            /* R_final = R_current * R_prev */
            Rf = R * Rf;
        }

        /* a redetection is triggered in case the number of features 
            being tracked go below a particular threshold */
        if (prevFeatures.size() < MIN_NUM_FEAT)
        {
            featureDetection(prevImage, prevFeatures);
            featureTracking(prevImage, currImage, prevFeatures, currFeatures, status);
        }

        prevImage = currImage.clone();
        prevFeatures = currFeatures;

        /* offset for easier visuzalization */
        int x = int(tf.at<double>(0)) + 300;
        /* -1 inversion and offset for easier visualization */
        int y = int(-1 * tf.at<double>(2)) + 500;
        cv::circle(traj, cv::Point(x, y), 1, CV_RGB(255, 0, 0), 2);

        cv::rectangle(traj, cv::Point(10, 30), cv::Point(550, 50), CV_RGB(0, 0, 0), cv::FILLED);
        sprintf(text, "Coordinates: x = %02fm y = %02fm z = %02fm", tf.at<double>(0), tf.at<double>(1), tf.at<double>(2));
        putText(traj, text, textOrg, fontFace, fontScale, cv::Scalar::all(255), thickness, 8);

        cv::imshow("Road facing Egomotion camera", currColorImg);
        cv::imshow("Trajectory", traj);

        cv::waitKey(1);
    }

    return 0;
}
