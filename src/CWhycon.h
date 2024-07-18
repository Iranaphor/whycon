#ifndef __CWhycon_H__
#define __CWhycon_H__

#define _LARGEFILE_SOURCE
#define _FILE_OFFSET_BITS 64

#include <stdlib.h>
#include <string>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <SDL/SDL.h>

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
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

using namespace cv;

class CWhycon : public rclcpp::Node {

    public:

        int imageWidth;         // default camera resolution
        int imageHeight;        // default camera resolution
        float circleDiameter;   // default black circle diameter [m];
        float fieldLength;      // X dimension of the coordinate system
        float fieldWidth;       // Y dimension of the coordinate system

        /*marker detection variables*/
        bool identify;          // whether to identify ID
        int numMarkers;         // num of markers to track
        int numFound;           // num of markers detected in the last step
        int numStatic;          // num of non-moving robots
        int maxMarkers;         // maximum number of markers

        //circle identification
        int idBits;             // num of ID bits
        int idSamples;          // num of samples to identify ID
        int hammingDist;        // hamming distance of ID code

        /*program flow control*/
        bool stop;          // stop and exit ?
        int moveVal;        // how many frames to process ?
        int moveOne;        // how many frames to process now (setting moveOne to 0 or lower freezes the video stream)

        // filtering parameters
        bool useAcuteFilter;
        int maxDetectionDistance;
        int minDetectionsToPublish;

        CWhycon();      // constructor sets up essential variables
        ~CWhycon();     // destructor
        void init(std::string fPath, std::string calPath);      // creates necessary objects and segment detectors

        void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
        void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);

    private:

        /*GUI-related stuff*/
        CGui* gui;              // drawing, events capture
        bool useGui;            // use graphic interface at all?
        int guiScale;           // in case camera resolution exceeds screen one, gui is scaled down
        SDL_Event event;        // store mouse and keyboard events
        int keyNumber;          // number of keys pressed in the last step       
        Uint8 lastKeys[1000];   // keys pressed in the previous step
        Uint8 *keys;            // pressed keys
        bool displayHelp;       // displays some usage hints
        bool drawCoords;        // draws coordinates at the robot's positions
        int runs;               // number of gui updates/detections performed 
        int evalTime;           // time required to detect the patterns
        int screenWidth;        // max GUI width
        int screenHeight;       // max GUI height

        /*variables related to (auto) calibration*/
        const int calibrationSteps = 20;            // how many measurements to average to estimate calibration pattern position (manual calib)
        const int autoCalibrationSteps = 30;        // how many measurements to average to estimate calibration pattern position (automatic calib)  
        const int autoCalibrationPreSteps = 10;     // how many measurements to discard before starting to actually auto-calibrating (automatic calib)  
        int calibNum;                       // number of objects acquired for calibration (5 means calibration finished inactive)
        STrackedObject calib[5];            // array to store calibration patterns positions
        STrackedObject *calibTmp;           // array to store several measurements of a given calibration pattern
        int calibStep;                      // actual calibration step (num of measurements of the actual pattern)
        bool autocalibrate;                 // is the autocalibration in progress ?
        ETransformType lastTransformType;   // pre-calibration transform (used to preserve pre-calibation transform type)
        int wasMarkers;                        // pre-calibration number of robots to track (used to preserve pre-calibation number of robots to track)

        /*marker detection variables*/
        STrackedObject *objectArray;       // object array (detected objects in metric space)
        SSegment *currInnerSegArray;       // inner segment array
        SSegment *currentSegmentArray;     // segment array (detected objects in image space)
        SSegment *lastSegmentArray;        // segment position in the last step (allows for tracking)
        CCircleDetect **detectorArray;     // detector array (each pattern has its own detector)

        CTransformation *trans;                         // allows to transform from image to metric coordinates
        CNecklace *decoder;                             // Necklace code decoder

        rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr subInfo;                // camera info subscriber
        image_transport::Subscriber subImg;     // image raw subscriber
        rclcpp::Publisher<whycon_ros::msg::MarkerArray>::SharedPtr markers_pub;             // publisher of MarkerArray
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr visual_pub;       // publisher of MarkerArray for RVIZ
        CRawImage *image;                       // encapsulation of image raw data

        std::string fontPath;           // path to GUI font
        std::string calibDefPath;       // path to user defined coordinate calibration

        // intrinsic and distortion params from camera_info
        Mat intrinsic = Mat::ones(3,3, CV_32FC1);
        Mat distCoeffs = Mat::ones(1,5, CV_32FC1);

        /*manual calibration can be initiated by pressing 'r' and then clicking circles at four positions (0,0)(fieldLength,0)...*/
        void manualcalibration();

        /*finds four outermost circles and uses them to set-up the coordinate system - [0,0] is left-top, [0,fieldLength] next in clockwise direction*/
        void autocalibration();

        /*process events coming from GUI*/
        void processKeys();
};

#endif
