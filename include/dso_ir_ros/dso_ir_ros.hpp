/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   dso_ir_ros.hpp
 * Author: arwillis
 *
 * Created on April 24, 2023 8:45 AM
 */

#ifndef ROS_RGBD_SURFACE_TRACKER_HPP
#define ROS_RGBD_SURFACE_TRACKER_HPP

#ifdef __cplusplus

// ROS includes
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
//#include <sensor_msgs/PointCloud2.h>
//#include <sensor_msgs/image_encodings.h>
//#include <geometry_msgs/TransformStamped.h>
//#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <image_transport/image_transport.h>
//#include <image_geometry/pinhole_camera_model.h>

// TF includes
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

class ROS_dso_ir {
public:
    typedef boost::shared_ptr<ROS_dso_ir> Ptr;

    std::string camera_calibration_file_str;
    std::string camera_vignette_file_str;
    std::string camera_gamma_file_str;

    ROS_dso_ir() :
    nodeptr(new ros::NodeHandle),
    nh("~"), frameID(0) {
        nh.param<std::string>("map_frame", map_frame_id_str, "optitrack");
        nh.param<std::string>("optical_parent", parent_frame_id_str, "optitrack");
        nh.param<std::string>("optical_frame", rgbd_frame_id_str, "rgbd_frame");
        nh.param<std::string>("calibration_file", camera_calibration_file_str, "");
        nh.param<std::string>("vignette_file", camera_vignette_file_str, "");
        nh.param<std::string>("gamma_file", camera_gamma_file_str, "");

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
        //        r

    };

    virtual ~ROS_dso_ir();

    void initializeSubscribersAndPublishers();
    void rgbImageCallback(const sensor_msgs::ImageConstPtr rgb_msg_in);
    //            const sensor_msgs::CameraInfoConstPtr& info_msg);

    //    void cameraInfoToCVMats(const sensor_msgs::CameraInfoConstPtr &cam_info,
    //            bool useRectifiedParameters, cv::Mat &cameraMatrix,
    //            cv::Mat &distortionCoeffs, cv::Size &imSize);

    //    void publishResults(const sensor_msgs::ImageConstPtr& depth_msg, cv::Mat & rgb_result);

private:
    // -------------------------
    // Disabling default copy constructor and default
    // assignment operator.
    // -------------------------
    ROS_dso_ir(const ROS_dso_ir& yRef);
    ROS_dso_ir& operator=(const ROS_dso_ir& yRef);

    ros::NodeHandle nh;
    ros::NodeHandlePtr nodeptr;
    ros::Time frame_time;
    std::string frame_id_str;

    cv_bridge::CvImageConstPtr cv_rgbimg_ptr;

    std::string map_frame_id_str; // frame id for parent of the RGBD sensor
    std::string parent_frame_id_str; // frame id for parent of the RGBD sensor
    std::string rgbd_frame_id_str; // frame id for RGBD sensor      


    ros::Subscriber imgSub;
    // subscribers to RGBD sensor data
    //    image_transport::SubscriberFilter sub_rgbImage;
    //    ros::Subscriber sub_trigger_ptcloud_pub;

    //    message_filters::Subscriber<sensor_msgs::CameraInfo> sub_rgbCameraInfo;

    //    ros::Publisher pubPose_w_cov;
    //    ros::Publisher pubOdom_w_cov;

    //    tf::TransformBroadcaster tfbroadcaster;
    //
    //    image_transport::ImageTransport it;
    //    image_transport::Publisher image_pub;
    int frameID;
};

#endif /* __cplusplus */
#endif /* ROS_RGBD_SURFACE_TRACKER_H */

