#include <opencv2/opencv.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>
#include <iostream>
#include <sstream>

#include "BOOSTING/trackerAdaBoosting.hpp"
#include "BOOSTING/roiSelector.hpp"
#include "BOOSTING/fourCornerSelector.hpp"
#include "cbot_tracker/cameraWrapper.hpp" // for usb camera and video
#include "cbot_tracker/common_utility.hpp"

#include <time.h>
#include <stdio.h>
#include <boost/asio.hpp>
#include <boost/bind.hpp>

using namespace std; 
using namespace cv;

/*
static const char* keys =
{   "{help h usage ?    | | print usage message}"
    "{useCamera         | | choose Camera or video}"
    "{d                 | | Camera device number}"
    "{v                 | | video name        }"
    "{r                 |5| ros rate}"};

*/
static void help()
{
    std::cout << "parameters: \n"
                 "{help h usage ?    | | print usage message}\n"
                 "{useCamera         | | choose Camera or video}\n"
                 "{d                 | | Camera device number}\n"
                 "{v                 | | video name        }\n"
                 "{r                 |5| ros rate}\n\n";

    std::cout << "\n use example: ./visual_localization -useCamera=true -d=1 -r=5\n"
                 "\n use example: ./visual_localization -useCamera=false -v=xx.avi -r=5\n"
                 << std::endl;

    std::cout << "\n\nHot keys: \n"
       "\tq - quit the program\n"
       "\tp - pause/start video\n";
}


bool updateROI(cv::Mat &img, cv::Ptr<BOOSTING::Tracker> &tracker, cv::Rect2d &roi);


bool refresh_H = false;
bool tracker_initialized = false;

// sort corners as left_top, right_top, left_bottom, right_bottom
void cornersSort(std::vector<cv::Point2f>& corners){
    cv::Point2f center;
    center.x = (corners[0].x + corners[1].x + corners[2].x + corners[3].x) / 4.0;
    center.y = (corners[0].y + corners[1].y + corners[2].y + corners[3].y) / 4.0;

    std::vector<cv::Point2f> corners_ordered; // lt, rt, lb, rb
    corners_ordered.resize(4);
    for(int i = 0; i < 4; ++i){
        if(corners[i].x < center.x && corners[i].y < center.y)
            corners_ordered[0] = corners[i];
        else if(corners[i].x < center.x && corners[i].y > center.y)
            corners_ordered[2] = corners[i];
        else if(corners[i].x > center.x && corners[i].y < center.y)
            corners_ordered[1] = corners[i];
        else if(corners[i].x > center.x && corners[i].y > center.y)
            corners_ordered[3] = corners[i];
    }
    corners = corners_ordered;
}

int main( int argc, char** argv ){

    if(argc < 2){
        help();
        return -1;
    }
    cv::FileStorage fs(argv[1], cv::FileStorage::READ);
    if(!fs.isOpened()){
        help();
        return -1;
    }

    boost::asio::io_service ioService;
    std::string serialPortName;
    fs["serialPortName"] >> serialPortName;
    serialPortName = "/dev/" + serialPortName;
    boost::asio::serial_port sPort(ioService, serialPortName);
    // set post's parameter
    sPort.set_option( boost::asio::serial_port::baud_rate(9600) );
    sPort.set_option( boost::asio::serial_port::flow_control(boost::asio::serial_port::flow_control::none) );
    sPort.set_option( boost::asio::serial_port::parity(boost::asio::serial_port::parity::odd) );
    sPort.set_option( boost::asio::serial_port::stop_bits(boost::asio::serial_port::stop_bits::one) );
    sPort.set_option( boost::asio::serial_port::character_size(8) );

    std::string intrinsic_file;
    cv::Mat cameraMatrix, distCoeffs;
    int rectify = 1;
    fs["rectify"] >> rectify;

    if(rectify) // when read image, need to rectify the image
    {
        fs["cameraMatrix"] >> cameraMatrix;
        fs["distCoeffs"] >> distCoeffs;
    }

    cameraWrapper::cameraWrapper cam;
    std::string camera_type;
    fs["cameraType"] >> camera_type;
    if(camera_type == "usb"){
        int device = -1;
        fs["cameraDevice"] >> device;
        if(!cam.usbcam_init(device, cameraMatrix, distCoeffs)){
            std::cout << "ERROR! Unable to open usb camera: " << device << std::endl;
            return -1;
        }
    }
    else if(camera_type == "video"){
        std::string video_file;
        fs["cameraFile"] >> video_file;
        if(!cam.videocam_init(video_file, cameraMatrix, distCoeffs)){
            std::cout << "ERROR! Unable to open Video: " << video_file << std::endl;
            return -1;
        }
    }
    else if(camera_type == "ip"){
        std::string ip;
        fs["ip"] >> ip;
        if(!cam.ipcam_init(ip, cameraMatrix, distCoeffs)){
            std::cout << "ERROR! Unable to open ip camera: " << ip << std::endl;
            return -1;
        }
    }

    // instantiates the specific Tracker
    cv::Ptr<BOOSTING::Tracker> tracker = BOOSTING::TrackerBoosting::create();
    if(tracker == NULL){
        std::cout << "\nError in the instantiation of the tracker" << std::endl;
        return -1;
    }

    cv::Mat image;
    if(rectify)
    {
        if(!cam.getNextRectifiedImage(image))
        {
            std::cout << "ERROR! failed to read image" << std::endl;
            return -1;
        }
    }
    else // do not rectify the image
    {
        if(!cam.getNextImage(image))
        {
            std::cout << "ERROR! failed to read image" << std::endl;
            return -1;
        }
    }

    cv::namedWindow("FourCornerSelector", 0);
    std::vector<cv::Point2f> corners_ori = BOOSTING::selectFourCorner("FourCornerSelector", image);
    cv::destroyWindow("FourCornerSelector");
    cornersSort(corners_ori);

    std::vector<cv::Point2f> cornersPerspectived;
    cornersPerspectived.resize(4);
    cv::FileNode cornersNode = fs["cornersPerspectived"];
    cv::FileNodeIterator it = cornersNode.begin(), it_end = cornersNode.end();
    for(int idx = 0; it != it_end; ++it, ++idx){
        cornersPerspectived[idx].x = (float)(*it)["x"];
        cornersPerspectived[idx].y = (float)(*it)["y"];
    }
    cornersSort(cornersPerspectived);

    for(int i = 0; i < corners_ori.size(); ++i)
        std::cout << "corners_ori_" << i << ": x = " << corners_ori[i].x << " , y = " << corners_ori[i].y << " \n";
    for(int i = 0; i < cornersPerspectived.size(); ++i)
        std::cout << "cornersPerspectived_" << i << ": x = " << cornersPerspectived[i].x << " , y = " << cornersPerspectived[i].y << " \n";
    
    cv::Mat H_to_bg, H_to_frame;
    H_to_bg = cv::getPerspectiveTransform(corners_ori, cornersPerspectived);
    H_to_frame = cv::getPerspectiveTransform(cornersPerspectived, corners_ori);

    cv::Mat img_bg, img_bg_display;
    cv::warpPerspective(image, img_bg, H_to_bg, cv::Size(cornersPerspectived[3].x+20, cornersPerspectived[3].y+20));
    img_bg.copyTo(img_bg_display);
    
    /*
    cv::Ptr<cv::xfeatures2d::SURF> feature_detector = cv::xfeatures2d::SURF::create(400);
    std::string detector_method = "SURF";

    std::vector<cv::KeyPoint> kps_frame;
    cv::Mat descriptors_frame;
    feature_detector->detectAndCompute(image, cv::Mat(), kps_frame, descriptors_frame); // Detects keypoints and computes the descriptors 
    
    std::vector<cv::KeyPoint> kps_bg;
    cv::Mat descriptors_bg;
    feature_detector->detectAndCompute(img_bg_display, cv::Mat(), kps_bg, descriptors_bg);

    cbot_tracker::compute_homography(detector_method, H_to_frame, H_to_bg, kps_frame, kps_bg, descriptors_frame, descriptors_bg);
    cbot_tracker::part_warpPerspective(image, img_bg_display, cv::Rect2d(0, 0, img_bg.cols, img_bg.rows), H_to_frame);
    */

    cv::namedWindow("selectROI", 0);
    cv::Rect2d roi = BOOSTING::selectROI("selectROI", img_bg_display);
    cv::Rect2d bb_for_perspective =  cbot_tracker::enlargeRect(img_bg_display, roi, 3);
    cv::destroyWindow("selectROI");

    bool pause_tracker = false;
    cv::namedWindow("visual_localization", 0);

    while(1)
    {
        clock_t t = clock();
        if(!tracker_initialized){   // reinitialize the tracker
            if(!tracker->init(img_bg_display, roi))
            {
                std::cout << "\n Could not initialize the tracker... \n";
                return -1;
            }
            tracker_initialized = true;
        }
    
        bool hasNewImage = false;
        if(rectify)
            hasNewImage = cam.getNextRectifiedImage(image);
        else
            hasNewImage = cam.getNextImage(image);

        /*
        if(refresh_H){  // refresh the H_to_bg
            kps_frame.clear();
            feature_detector->detectAndCompute(image, cv::Mat(), kps_frame, descriptors_frame);
            cbot_tracker::compute_homography(detector_method, H_to_frame, H_to_bg, kps_frame, kps_bg, descriptors_frame, descriptors_bg);
        }
        */

        if(hasNewImage)
        {
            if(!pause_tracker)
            {
                
                bb_for_perspective = cbot_tracker::enlargeRect(img_bg_display, roi, 3);
                // perspective the image to the img_bg_display's ROI image: 
                img_bg.copyTo(img_bg_display);
                cbot_tracker::part_warpPerspective(image, img_bg_display, bb_for_perspective, H_to_frame);
                cv::rectangle(img_bg_display, bb_for_perspective, cv::Scalar(0, 0, 255), 1, 1);
                
                if(updateROI(img_bg_display, tracker, roi))
                {
                    double x = roi.x + roi.width / 2;
                    double y = roi.y + roi.height / 2;
                    double theta = 0;

                    char buffers[25];
                    sprintf(buffers, "x%7.1f", double(x));
                    sprintf(buffers + 8, "y%7.1f", double(y));
                    sprintf(buffers + 16, "t%7.1f", double(theta));
                    buffers[24] = '\n';

                    boost::asio::write(sPort, boost::asio::buffer(buffers) );
                }  
                
            }
            //createHeader(header);
            //publishImage(header, img_bg_display, imagePub);
            
            cv::imshow("visual_localization", img_bg_display);
        }
        t = clock() - t;
        printf("\n One frame consumes %.1f ms!\n", ((float)t)*1000/CLOCKS_PER_SEC);

        char c = (char)cv::waitKey(2);
        if(c == 'q')
            break;
        if(c == 'p')
            pause_tracker = !pause_tracker;

    }
    cam.releaseCamera();
    cv::destroyAllWindows();
    return 0;
}

/*  update localizatin roi
*/
bool updateROI(cv::Mat &img, cv::Ptr<BOOSTING::Tracker> &tracker, cv::Rect2d &roi)
{
    if(tracker->update(img, roi))
    {
        cv::rectangle(img, roi, cv::Scalar(255, 0, 0), 2, 1);
        return true;
    }
    else
        return false;
}

