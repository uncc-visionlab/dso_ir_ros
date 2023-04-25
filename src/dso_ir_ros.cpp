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
#include <cv_bridge/cv_bridge.h>

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
    imgSub = nh.subscribe("/camera/image_raw", 1, &ROS_dso_ir::rgbImageCallback, this);

    undistorter = Undistort::getUndistorterForFile(camera_calibration_file_str,
            camera_gamma_file_str, camera_vignette_file_str);

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
    std::cout << "Input files are calib=" << calib << " gamma=" << gammaFile << " vignette=" << vignetteFile << std::endl;
    undistorter = Undistort::getUndistorterForFile(calib, gammaFile, vignetteFile);

    setGlobalCalib(
            (int) undistorter->getSize()[0],
            (int) undistorter->getSize()[1],
            undistorter->getK().cast<float>());

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

    ros::spin();
    fullSystem->printResult(saveFile);
    for (IOWrap::Output3DWrapper* ow : fullSystem->outputWrapper) {
        ow->join();
        delete ow;
    }

    delete undistorter;
    delete fullSystem;

    return EXIT_SUCCESS;
}