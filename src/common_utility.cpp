#include "cbot_tracker/common_utility.hpp"
#include <omp.h>

const double PI = 3.1415926;

namespace cbot_tracker{

cv::Rect2d enlargeRect(const cv::Mat& img, const cv::Rect2d& R, double scale){
    cv::Rect2d R2(R.x, R.y, scale * R.width, scale * R.height);

    R2.x -= (scale  - 1) / 2.0 * R.width;
    R2.y -= (scale - 1) / 2.0 * R.height;

    R2 &= cv::Rect2d(0, 0, img.cols, img.rows); // rectangle intersection

    return R2;
}


void part_warpPerspective(cv::Mat &img_src, cv::Mat &img_dst, cv::Rect2d bb, cv::Mat &H_to_src)
{
    //使用双线性插值，反投影计算透视投影后dst(bb)的像素值
    int n = ((int)bb.height) * ((int)bb.width);
    std::vector< std::vector<cv::Point2f> > pt_dst(n);
    std::vector< std::vector<cv::Point2f> > pt_src(n);
    
    // accept only char type matrices
    CV_Assert(img_dst.depth() == CV_8U);

    cv::Mat_<cv::Vec3b> I_src = img_src; //彩色图，三通道
    cv::Mat_<cv::Vec3b> I_dst  = img_dst;

    int dst_x, dst_y;
    double src_x, src_y;

    #pragma omp parallel for
    for( int r = 0; r < (int)bb.height; ++r) // rows
    {   
        for ( int c = 0; c < (int)bb.width; ++c){ //cols;

            dst_x = (int)bb.x + c;
            dst_y = (int)bb.y + r;
            pt_dst[r*( (int)bb.width )+c].push_back(cv::Point2f(dst_x, dst_y));
            cv::perspectiveTransform(pt_dst[r*((int)bb.width)+c], pt_src[r*((int)bb.width)+c], H_to_src);
            src_x = pt_src[r*((int)bb.width)+c][0].x; 
            src_y = pt_src[r*((int)bb.width)+c][0].y;
     
            if(src_x < 2 || src_x + 3 > img_src.cols || src_y < 2 || src_y + 3 > img_src.rows)
                continue;
            double f00 = (int(src_x)+1-src_x)*(int(src_y)+1-src_y);
            double f10 = (src_x - int(src_x))*(int(src_y) + 1 - src_y);
            double f01 = (int(src_x)+1- src_x)*(src_y - int(src_y));
            double f11 = (src_x - int(src_x))*(src_y - int(src_y));
            int src_y_i = int(src_y);
            int src_x_i = int(src_x);
         
            for(int i = 0; i < img_src.channels(); i++){
                I_dst(dst_y, dst_x)[i] = I_src(src_y_i, src_x_i)[i] * f00 + I_src(src_y_i, src_x_i+1)[i] * f10 + I_src(src_y_i+1, src_x_i)[i] * f01 + I_src(src_y_i+1, src_x_i+1)[i] * f11;
            } 
        }
    }
}

void drawArrow(cv::Mat& img, cv::Point2d pStart, double radian, int len, int arrow_len, int arrow_degree, cv::Scalar color, int thickness)
{       
    cv::Point2d arrow;    
    //计算 θ 角（最简单的一种情况在下面图示中已经展示，关键在于 atan2 函数，详情见下面)

    cv::Point2d pEnd = cv::Point2d(pStart.x + len * cos(radian), pStart.y + len * sin(radian));
    cv::line(img, pStart, pEnd, color, thickness);   
    //计算箭角边的另一端的端点位置（上面的还是下面的要看箭头的指向，也就是pStart和pEnd的位置） 
    arrow.x = pEnd.x + arrow_len * cos(PI/2 + radian - PI * arrow_degree / 180);     
    arrow.y = pEnd.y + arrow_len * sin(PI/2 + radian - PI * arrow_degree / 180);  
    cv::line(img, pEnd, arrow, color, thickness);   
    arrow.x = pEnd.x + arrow_len * cos(PI/2 + radian + PI * arrow_degree / 180);     
    arrow.y = pEnd.y + arrow_len * sin(PI/2 + radian + PI * arrow_degree / 180);    
    cv::line(img, pEnd, arrow, color, thickness);
}

void compute_homography(std::string detector_method, cv::Mat& H_to_bg, cv::Mat& H_to_original, std::vector<cv::KeyPoint>& keypoints_bg, std::vector<cv::KeyPoint>& keypoints_original, cv::Mat& descriptors_bg, cv::Mat& descriptors_original){

    //H_to_bg
    std::vector<cv::Point2f> matched_bg, matched_original;

    if(detector_method == "ORB"){
        //----------  for orb -----------------
        //Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("FlannBased");
        cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");

        //Matching descriptor vectors using FLANN matcher
        std::vector< std::vector<cv::DMatch> > matches;
        matcher->knnMatch(descriptors_bg, descriptors_original, matches, 2);
        /*
        void cv::DescriptorMatcher::knnMatch    (   InputArray      queryDescriptors,
                                                    InputArray      trainDescriptors, 
                                                )
        */

        for(unsigned i = 0; i < matches.size(); i++) {
            if(matches[i][0].distance < 0.95 * matches[i][1].distance) {
                  matched_bg.push_back(keypoints_bg[matches[i][0].queryIdx].pt); //Point2f cv::KeyPoint::pt
                  matched_original.push_back(keypoints_original[matches[i][0].trainIdx].pt);
            }
        }
    }

    else if(detector_method == "SURF"){
        cv::FlannBasedMatcher matcher;
        std::vector< cv::DMatch > matches;
        matcher.match( descriptors_bg, descriptors_original, matches );

        double max_dist = 0; double min_dist = 100;
        //-- Quick calculation of max and min distances between keypoints
        for( int i = 0; i < descriptors_original.rows; i++ )
        { 
            double dist = matches[i].distance;
            if( dist < min_dist ) min_dist = dist;
            if( dist > max_dist ) max_dist = dist;
        }

        //-- choose only "good" matches (i.e. whose distance is less than 3*min_dist )
        std::vector< cv::DMatch > good_matches;

        for( int i = 0; i < descriptors_original.rows; i++ )
        { 
            if( matches[i].distance <= 6*min_dist ){
                good_matches.push_back( matches[i]); 
            }
        }
        
        for(unsigned i = 0; i < good_matches.size(); i++) {
            matched_bg.push_back(keypoints_bg[good_matches[i].queryIdx].pt); //Point2f cv::KeyPoint::pt
            matched_original.push_back(keypoints_original[good_matches[i].trainIdx].pt);        
        }   
    }
    /*
        Mat cv::findHomography  (   InputArray      srcPoints,
                                    InputArray      dstPoints, 
                                )
    */
    H_to_original = cv::findHomography(matched_bg, matched_original, cv::RANSAC ); // perspective transformation H from matched_bg to matched_original
    H_to_bg = cv::findHomography(matched_original, matched_bg, cv::RANSAC );
    //coupute H which transform from object --> track
}

} // namespace cbot_tracker