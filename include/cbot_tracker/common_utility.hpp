#ifndef COMMON_UTILITY_H
#define COMMON_UTILITY_H

#include <iostream>
#include <string>
#include <cmath>
#include <vector>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>

//#include <Eigen/Dense>
//#include <Eigen/Geometry>

namespace cbot_tracker
{

void drawArrow(cv::Mat& img, 
               cv::Point2d pStart, 
               double radian, 
               int len, 
               int arrow_len, 
               int arrow_degree, 
               cv::Scalar color, 
               int thickness);

void part_warpPerspective(cv::Mat &img_src, cv::Mat &img_dst, cv::Rect2d bb, cv::Mat &H_to_src);

cv::Rect2d enlargeRect(const cv::Mat& img, const cv::Rect2d& R, double scale);

void compute_homography(std::string detector_method, 
                        cv::Mat& H_to_bg, 
                        cv::Mat& H_to_original, 
                        std::vector<cv::KeyPoint>& keypoints_bg, 
                        std::vector<cv::KeyPoint>& keypoints_original, 
                        cv::Mat& descriptors_bg, 
                        cv::Mat& descriptors_original);

} // namespace cbot_tracker

#endif