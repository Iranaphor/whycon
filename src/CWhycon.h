#ifndef __CWhycon_H__
#define __CWhycon_H__

#define _LARGEFILE_SOURCE
#define _FILE_OFFSET_BITS 64

#include <stdlib.h>
#include <string>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <SDL2/SDL.h>

// WhyCon libs
#include "CGui.h"
#include "CTimer.h"
#include "CCircleDetect.h"
#include "CTransformation.h"
#include "CNecklace.h"
#include "CRawImage.h"

// ROS2 libraries
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

// Include the custom messages
#include "whycon_ros/msg/marker_array.hpp"
#include "whycon_ros/msg/marker.hpp"

using namespace cv;

class CWhycon : public rclcpp::Node {

    public: 
        CWhycon();      // constructor sets up essential variables
        ~CWhycon();     // destructor
        void init(std::string fPath, std::string calPath);      // creates necessary objects and segment detectors

        void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
        void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);

    private:
        // Various class members...

        // ROS publishers and subscribers
        rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr subInfo;
        image_transport::Subscriber subImg;
        rclcpp::Publisher<whycon_ros::msg::MarkerArray>::SharedPtr markers_pub;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr visual_pub;

        // Other private members...

        // Methods for calibration and processing
        void manualcalibration();
        void autocalibration();
        void processKeys();
};

#endif // __CWhycon_H__
