#ifndef CAMERAWRAPPER_DEFINE_HPP
#define CAMERAWRAPPER_DEFINE_HPP

#include <opencv2/opencv.hpp>
#include <iostream>

#include "camera_hc.hpp"      
#include "log.h"
#include "comdef.h"
#include <unistd.h> // usleep

namespace cameraWrapper
{
class ipCamera
{
private:
    int _cameraPORT = 8000;
    std::string _cameraIP;
    cv::Mat _cameraMatrix;
    cv::Mat _distCoeffs;
    CAMERA_HC *_camera;

public:
    ipCamera(){}

    // initialization
    bool init(const std::string& cameraIP, const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs){
        _cameraIP = cameraIP;
        _cameraMatrix = cameraMatrix.clone();
        _distCoeffs = distCoeffs.clone();
        _camera = new CAMERA_HC();
        if (0 != _camera->createIPC(_cameraIP.c_str(), _cameraPORT, 0))
        {
            printf("Create IPC failed: %s:%d\n",_cameraIP.c_str(), _cameraPORT);
            return false;
        }
        // Wait 1s to init camera configuration
        usleep(1e6);

        return true;
    }

    bool getNextImage(cv::Mat &img){
        if(0 != _camera->getImage(img)){
            printf("ipCamera: get data failed !\n");
            return false;
        }
        return true;
    }

    bool getNextRectifiedImage(cv::Mat &img){
        if(0 != _camera->getImage(img))
            return false;
        cv::Mat tmp = img.clone();
        cv::undistort(tmp, img, _cameraMatrix, _distCoeffs);
        return true;
    }

    void rectifyImage(cv::Mat &img){
        cv::Mat originImg = img.clone();
        cv::undistort(originImg, img, _cameraMatrix, _distCoeffs);
    }

    void releaseCamera(){
        delete _camera;
    }

};

class usbCamera
{
private:
    int _device;    // camera device number
    cv::VideoCapture _cap;
    cv::Mat _cameraMatrix;
    cv::Mat _distCoeffs;
    
public:
    //cv::Mat _image; 

    usbCamera(){
        _device = -1;
    }

    /* initialization */
    bool init(int device, cv::Mat cameraMatrix, cv::Mat distCoeffs){
        _device = device;
        _cameraMatrix = cameraMatrix;
        _distCoeffs = distCoeffs;
        _cap.open(_device);
        _cap.set(cv::CAP_PROP_FRAME_WIDTH, 960);
        _cap.set(cv::CAP_PROP_FRAME_HEIGHT, 720);
        if(!_cap.isOpened())
            return false;
        return true;
    }

    bool getNextImage(cv::Mat &img){
        _cap.read(img);
        if(img.empty())
            return false;
        return true;
    }

    bool getNextRectifiedImage(cv::Mat &img){
        _cap.read(img);
        if(img.empty())
            return false;
        cv::Mat tmp = img.clone();
        cv::undistort(tmp, img, _cameraMatrix, _distCoeffs);
        return true;
    }

    void rectifyImage(cv::Mat &img){
        cv::Mat originImg = img.clone();
        cv::undistort(originImg, img, _cameraMatrix, _distCoeffs);
    }

    void releaseCamera(){
        _cap.release();
    }
};

class videoCap
{
private:
    std::string _vfile;    // camera device number
    cv::VideoCapture _cap;
    cv::Mat _cameraMatrix;
    cv::Mat _distCoeffs;
    
public:
    //cv::Mat _image; 

    videoCap(){
        _vfile = "";
    }

    /* initialization */
    bool init(const std::string vfile, cv::Mat cameraMatrix, cv::Mat distCoeffs){
        _vfile = vfile;
        _cameraMatrix = cameraMatrix;
        _distCoeffs = distCoeffs;
        _cap.open(_vfile);
        _cap.set(cv::CAP_PROP_FRAME_WIDTH, 960);
        _cap.set(cv::CAP_PROP_FRAME_HEIGHT, 720);
        if(!_cap.isOpened())
            return false;
        return true;
    }

    bool getNextImage(cv::Mat &img){
        _cap.read(img);
        if(img.empty())
            return false;
        return true;
    }
    
    bool getNextRectifiedImage(cv::Mat &img){
        _cap.read(img);
        if(img.empty())
            return false;
        cv::Mat tmp = img.clone();
        cv::undistort(tmp, img, _cameraMatrix, _distCoeffs);
        return true;
    }

    void rectifyImage(cv::Mat &img){
        cv::Mat originImg = img.clone();
        cv::undistort(originImg, img, _cameraMatrix, _distCoeffs);
    }

    void releaseCamera(){
        _cap.release();
    }
};

// wraper of ipcamera, usbcamera and videocamera
class cameraWrapper
{
private:
    std::string _cameraType;
    ipCamera* _ipc;
    usbCamera* _usbc;
    videoCap* _videoc;

    
public:
    //cv::Mat _image; 

    cameraWrapper(){}

    bool ipcam_init(const std::string ip, cv::Mat cameraMatrix = cv::Mat(), cv::Mat distCoeffs = cv::Mat()){
        _cameraType = "ip";
        _ipc = new ipCamera;
        return _ipc->init(ip, cameraMatrix, distCoeffs);
    }

    bool usbcam_init(const int device, cv::Mat cameraMatrix = cv::Mat(), cv::Mat distCoeffs = cv::Mat()){
        _cameraType = "usb";
        _usbc = new usbCamera;
        return _usbc->init(device, cameraMatrix, distCoeffs);
    }

    bool videocam_init(const std::string vfile, cv::Mat cameraMatrix = cv::Mat(), cv::Mat distCoeffs = cv::Mat()){
        _cameraType = "video";
        _videoc = new videoCap;
        return _videoc->init(vfile, cameraMatrix, distCoeffs);
    }

    bool getNextImage(cv::Mat &img){
        if(_cameraType == "ip")
            return _ipc->getNextImage(img);
        else if(_cameraType == "usb")
            return _usbc->getNextImage(img);
        else if(_cameraType == "video")
            return _videoc->getNextImage(img);
        else
            return false;
    }
    
    bool getNextRectifiedImage(cv::Mat &img){
        if(_cameraType == "ip")
            return _ipc->getNextRectifiedImage(img);
        else if(_cameraType == "usb")
            return _usbc->getNextRectifiedImage(img);
        else if(_cameraType == "video")
            return _videoc->getNextRectifiedImage(img);
        else
            return false;
    }

    void rectifyImage(cv::Mat &img){
        if(_cameraType == "ip")
            _ipc->rectifyImage(img);
        else if(_cameraType == "usb")
            _usbc->rectifyImage(img);
        else if(_cameraType == "video")
            _videoc->rectifyImage(img);
    }

    void releaseCamera(){
        if(_cameraType == "ip"){
            _ipc->releaseCamera();
            delete _ipc;
        }
        else if(_cameraType == "usb"){
            _usbc->releaseCamera();
            delete _usbc;
        }
        else if(_cameraType == "video"){
            _videoc->releaseCamera();
            delete _videoc;
        }
    }
};


};

#endif 