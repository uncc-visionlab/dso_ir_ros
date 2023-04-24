/**
 * This file is part of DSO.
 * 
 * Copyright 2016 Technical University of Munich and Intel.
 * Developed by Jakob Engel <engelj at in dot tum dot de>,
 * for more information see <http://vision.in.tum.de/dso>.
 * If you use this code, please cite the respective publications as
 * listed on the above website.
 *
 * DSO is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * DSO is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with DSO. If not, see <http://www.gnu.org/licenses/>.
 */


#include <locale.h>
#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>

#include "IOWrapper/Pangolin/PangolinDSOViewer.h"
#include "IOWrapper/OutputWrapper/SampleOutputWrapper.h"

#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/PoseStamped.h>
#include "cv_bridge/cv_bridge.h"

#include <dso_ir_ros/dso_ir_ros.hpp>

#include "util/settings.h"
#include "FullSystem/FullSystem.h"
#include "util/Undistort.h"

using namespace dso;

FullSystem* fullSystem;
Undistort* undistorter;
bool useSampleOutput = false;

namespace stdpatch {
#define NUMIDCHARS 3

    template < typename T > std::string to_string(const T& n) {
        std::ostringstream stm;
        stm << std::setw(NUMIDCHARS) << std::setfill('0') << n;
        return stm.str();
    }
}

ROS_dso_ir::~ROS_dso_ir() {
    delete undistorter;
    delete fullSystem;
}

void ROS_dso_ir::rgbImageCallback(const sensor_msgs::ImageConstPtr rgb_msg) {
    static int frame_id = 0;
    frame_time = rgb_msg->header.stamp;
    std::string keyframe_frameid_str("frame_");
    keyframe_frameid_str.append(stdpatch::to_string(frame_id++));
    frame_id_str = keyframe_frameid_str;

    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(rgb_msg, sensor_msgs::image_encodings::MONO8);
    cv::Mat gray_img = cv_ptr->image;
    assert(gray_img.type() == CV_8U);
    assert(gray_img.channels() == 1);

    if (setting_fullResetRequested) {
        std::vector<IOWrap::Output3DWrapper*> wraps = fullSystem->outputWrapper;
        delete fullSystem;
        for (IOWrap::Output3DWrapper* ow : wraps) ow->reset();
        fullSystem = new FullSystem();
        fullSystem->linearizeOperation = false;
        fullSystem->outputWrapper = wraps;
        if (undistorter->photometricUndist != 0)
            fullSystem->setGammaFunction(undistorter->photometricUndist->getG());
        setting_fullResetRequested = false;
    }

    MinimalImageB minImg((int) gray_img.cols, (int) gray_img.rows, (unsigned char*) gray_img.data);
    ImageAndExposure* undistImg = undistorter->undistort<unsigned char>(&minImg, 1, 0, 1.0f);
    undistImg->timestamp = rgb_msg->header.stamp.toSec(); // relay the timestamp to dso
    fullSystem->addActiveFrame(undistImg, frameID);
    frameID++;
    delete undistImg;
}

void ROS_dso_ir::initializeSubscribersAndPublishers() {
    //    int queue_size = 10;
    imgSub = nh.subscribe("/usb_cam/image_raw", 1, &ROS_dso_ir::rgbImageCallback, this);

    undistorter = Undistort::getUndistorterForFile(camera_calibration_file_str,
            camera_gamma_file_str, camera_vignette_file_str);

    //    image_transport::ImageTransport it_depth(*nodeptr);
    //message_filters::Subscriber<sensor_msgs::CameraInfo>
    //        sub_rgbImage(*nodeptr, "rgb/image_raw", 1);
    //    image_transport::ImageTransport it_rgb(*nodeptr);
    // rgb uses normal ros transport hints.
    //    image_transport::TransportHints hints("raw", ros::TransportHints(), *nodeptr);
    //image_transport::SubscriberFilter sub_rgbImage;
    //    sub_rgbImage.subscribe(it_rgb, "input_image", 1, hints);

    //    message_filters::Subscriber<sensor_msgs::CameraInfo>
    //            sub_rgbCameraInfo;
    //    sub_rgbCameraInfo.subscribe(*nodeptr, "camera_info", 1);
    //    ros::NodeHandle nh;
    //    ros::Subscriber imgSub = nh.subscribe("image", 1, &vidCb);

    // option 1
    //    message_filters::TimeSynchronizer
    //            <sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo>
    //            syncTime(sub_depthImage, sub_rgbImage, sub_rgbCameraInfo, queue_size);
    //    syncTime.registerCallback(boost::bind(&Feature3DEngine::rgbdImageCallback,
    //            engineptr, _1, _2, _3));
    // option 2
    //    typedef message_filters::sync_policies::ExactTime
    //            <sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo> MyExactSyncPolicy;
    // ExactTime takes a queue size as its constructor argument, hence MySyncPolicy(queue_size)
    //    message_filters::Synchronizer<MyExactSyncPolicy> syncExact(MyExactSyncPolicy(queue_size),
    //            sub_depthImage, sub_rgbImage, sub_rgbCameraInfo);
    // option 3
    //    syncApprox.registerCallback(boost::bind(&ROS_RgbdSurfaceTracker::rgbdImageCallback,
    //            this, _1, _2, _3));

    //    pubPose_w_cov = nodeptr->advertise<geometry_msgs::PoseWithCovarianceStamped>("pose_w_cov", 1000);
    //    pubOdom_w_cov = nodeptr->advertise<geometry_msgs::PoseWithCovarianceStamped>("odom_w_cov", 1000);
}

int main(int argc, char **argv) {
    /**
     * The ros::init() function needs to see argc and argv so that it can perform
     * any ROS arguments and name remapping that were provided at the command line.
     * For programmatic remappings you can use a different version of init() which takes
     * remappings directly, but for most command-line programs, passing argc and argv is
     * the easiest way to do it.  The third argument to init() is the name of the node.
     *
     * You must call one of the versions of ros::init() before using any other
     * part of the ROS system.
     */
    ros::init(argc, argv, "dso_ir_ros");
    /**
     * NodeHandle is the main access point to communications with the ROS system.
     * The first NodeHandle constructed will fully initialize this node, and the last
     * NodeHandle destructed will close down the node.
     */
    ROS_dso_ir dso_ir;
    dso_ir.initializeSubscribersAndPublishers();

    setting_desiredImmatureDensity = 1000;
    setting_desiredPointDensity = 1200;
    setting_minFrames = 5;
    setting_maxFrames = 7;
    setting_maxOptIterations = 4;
    setting_minOptIterations = 1;
    setting_logStuff = false;
    setting_kfGlobalWeight = 1.3;

    printf("MODE WITH CALIBRATION, but without exposure times!\n");
    setting_photometricCalibration = 2;
    setting_affineOptModeA = 0;
    setting_affineOptModeB = 0;
    std::string calib;
    std::string vignetteFile;
    std::string gammaFile;
    std::string saveFile = "";
    calib = dso_ir.camera_calibration_file_str;
    gammaFile = dso_ir.camera_gamma_file_str;
    vignetteFile = dso_ir.camera_vignette_file_str;
    std::cout << "HERE 0 calib=" << calib << " gamma=" << gammaFile << " vignette=" << vignetteFile << std::endl;
    undistorter = Undistort::getUndistorterForFile(calib, gammaFile, vignetteFile);

    std::cout << "HERE 1" << std::endl;
    setGlobalCalib(
            (int) undistorter->getSize()[0],
            (int) undistorter->getSize()[1],
            undistorter->getK().cast<float>());

    std::cout << "HERE 2" << std::endl;
    fullSystem = new FullSystem();
    fullSystem->linearizeOperation = false;

    if (!disableAllDisplay)
        fullSystem->outputWrapper.push_back(new IOWrap::PangolinDSOViewer(
            (int) undistorter->getSize()[0],
            (int) undistorter->getSize()[1]));

    if (useSampleOutput)
        fullSystem->outputWrapper.push_back(new IOWrap::SampleOutputWrapper());

    if (undistorter->photometricUndist != 0)
        fullSystem->setGammaFunction(undistorter->photometricUndist->getG());

    std::cout << "HERE 3" << std::endl;
    //    ros::NodeHandle nh;
    //    ros::Subscriber imgSub = nh.subscribe("image", 1, &vidCb);

    ros::spin();
    fullSystem->printResult(saveFile);
    for (IOWrap::Output3DWrapper* ow : fullSystem->outputWrapper) {
        ow->join();
        std::cout << "HERE 4" << std::endl;
        delete ow;
    }

    delete undistorter;
    delete fullSystem;

    return EXIT_SUCCESS;
}

//void parseArgument(char* arg) {
//    int option;
//    char buf[1000];
//
//    if (1 == sscanf(arg, "sampleoutput=%d", &option)) {
//        if (option == 1) {
//            useSampleOutput = true;
//            printf("USING SAMPLE OUTPUT WRAPPER!\n");
//        }
//        return;
//    }
//
//    if (1 == sscanf(arg, "quiet=%d", &option)) {
//        if (option == 1) {
//            setting_debugout_runquiet = true;
//            printf("QUIET MODE, I'll shut up!\n");
//        }
//        return;
//    }
//
//
//    if (1 == sscanf(arg, "nolog=%d", &option)) {
//        if (option == 1) {
//            setting_logStuff = false;
//            printf("DISABLE LOGGING!\n");
//        }
//        return;
//    }
//
//    if (1 == sscanf(arg, "nogui=%d", &option)) {
//        if (option == 1) {
//            disableAllDisplay = true;
//            printf("NO GUI!\n");
//        }
//        return;
//    }
//    if (1 == sscanf(arg, "nomt=%d", &option)) {
//        if (option == 1) {
//            multiThreading = false;
//            printf("NO MultiThreading!\n");
//        }
//        return;
//    }
//    if (1 == sscanf(arg, "calib=%s", buf)) {
//        calib = buf;
//        printf("loading calibration from %s!\n", calib.c_str());
//        return;
//    }
//    if (1 == sscanf(arg, "vignette=%s", buf)) {
//        vignetteFile = buf;
//        printf("loading vignette from %s!\n", vignetteFile.c_str());
//        return;
//    }
//
//    if (1 == sscanf(arg, "gamma=%s", buf)) {
//        gammaFile = buf;
//        printf("loading gammaCalib from %s!\n", gammaFile.c_str());
//        return;
//    }
//
//    if (1 == sscanf(arg, "savefile=%s", buf)) {
//        saveFile = buf;
//        printf("saving to %s on finish!\n", saveFile.c_str());
//        return;
//    }
//    printf("could not parse argument \"%s\"!!\n", arg);
//}

//void vidCb(const sensor_msgs::ImageConstPtr img) {
//    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
//    assert(cv_ptr->image.type() == CV_8U);
//    assert(cv_ptr->image.channels() == 1);
//
//
//    if (setting_fullResetRequested) {
//        std::vector<IOWrap::Output3DWrapper*> wraps = fullSystem->outputWrapper;
//        delete fullSystem;
//        for (IOWrap::Output3DWrapper* ow : wraps) ow->reset();
//        fullSystem = new FullSystem();
//        fullSystem->linearizeOperation = false;
//        fullSystem->outputWrapper = wraps;
//        if (undistorter->photometricUndist != 0)
//            fullSystem->setGammaFunction(undistorter->photometricUndist->getG());
//        setting_fullResetRequested = false;
//    }
//
//    MinimalImageB minImg((int) cv_ptr->image.cols, (int) cv_ptr->image.rows, (unsigned char*) cv_ptr->image.data);
//    ImageAndExposure* undistImg = undistorter->undistort<unsigned char>(&minImg, 1, 0, 1.0f);
//    undistImg->timestamp = img->header.stamp.toSec(); // relay the timestamp to dso
//    fullSystem->addActiveFrame(undistImg, frameID);
//    frameID++;
//    delete undistImg;
//
//}

//int main2(int argc, char** argv) {
//    ros::init(argc, argv, "dso_ir_ros");
//
//    for (int i = 1; i < argc; i++) {
//        parseArgument(argv[i]);
//    }
//
//    setting_desiredImmatureDensity = 1000;
//    setting_desiredPointDensity = 1200;
//    setting_minFrames = 5;
//    setting_maxFrames = 7;
//    setting_maxOptIterations = 4;
//    setting_minOptIterations = 1;
//    setting_logStuff = false;
//    setting_kfGlobalWeight = 1.3;
//
//    printf("MODE WITH CALIBRATION, but without exposure times!\n");
//    setting_photometricCalibration = 2;
//    setting_affineOptModeA = 0;
//    setting_affineOptModeB = 0;
//
//    std::cout << "HERE 0 calib=" << calib << " gamma=" << gammaFile << " vignette=" << vignetteFile << std::endl;
//    undistorter = Undistort::getUndistorterForFile(calib, gammaFile, vignetteFile);
//
//    std::cout << "HERE 1" << std::endl;
//    setGlobalCalib(
//            (int) undistorter->getSize()[0],
//            (int) undistorter->getSize()[1],
//            undistorter->getK().cast<float>());
//
//    std::cout << "HERE 2" << std::endl;
//    fullSystem = new FullSystem();
//    fullSystem->linearizeOperation = false;
//
//    if (!disableAllDisplay)
//        fullSystem->outputWrapper.push_back(new IOWrap::PangolinDSOViewer(
//            (int) undistorter->getSize()[0],
//            (int) undistorter->getSize()[1]));
//
//    if (useSampleOutput)
//        fullSystem->outputWrapper.push_back(new IOWrap::SampleOutputWrapper());
//
//    if (undistorter->photometricUndist != 0)
//        fullSystem->setGammaFunction(undistorter->photometricUndist->getG());
//
//    std::cout << "HERE 3" << std::endl;
//    ros::NodeHandle nh;
//    ros::Subscriber imgSub = nh.subscribe("image", 1, &vidCb);
//
//    ros::spin();
//    fullSystem->printResult(saveFile);
//    for (IOWrap::Output3DWrapper* ow : fullSystem->outputWrapper) {
//        ow->join();
//        std::cout << "HERE" << std::endl;
//        delete ow;
//    }
//
//    delete undistorter;
//    delete fullSystem;
//
//    return 0;
//}

