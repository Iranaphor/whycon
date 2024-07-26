#include "CWhycon.h"

CWhycon::CWhycon() : Node("whycon_node") {
    // Initialize member variables...
}

CWhycon::~CWhycon() {
    // Cleanup...
}

void CWhycon::init(std::string fPath, std::string calPath) {
    this->fontPath = fPath;
    this->calibDefPath = calPath;

    // Declare and get parameters...

    // Allocate arrays and initialize detectors...

    // Initialize GUI and transformation objects...

    // Subscribe to topics and advertise publishers
    subInfo = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "/camera/camera_info", 1, std::bind(&CWhycon::cameraInfoCallback, this, std::placeholders::_1));

    image_transport::ImageTransport it(this->shared_from_this());
    subImg = it.subscribe("/camera/image_raw", 1, [this](const sensor_msgs::msg::Image::SharedPtr msg) {
        this->imageCallback(msg);
    });

    markers_pub = this->create_publisher<whycon_ros::msg::MarkerArray>("/whycon_ros/markers", 10);
    visual_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>("/whycon_ros/visual", 10);
}

void CWhycon::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
    if (msg->k[0] == 0) {
        RCLCPP_ERROR(this->get_logger(), "ERROR: Camera is not calibrated!");
        return;
    } else if (msg->k[0] != intrinsic.at<float>(0, 0) || msg->k[2] != intrinsic.at<float>(0, 2) || msg->k[4] != intrinsic.at<float>(1, 1) || msg->k[5] != intrinsic.at<float>(1, 2)) {
        for (int i = 0; i < 5; i++) distCoeffs.at<float>(i) = msg->d[i];
        int tmpIdx = 0;
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                intrinsic.at<float>(i, j) = msg->k[tmpIdx++];
            }
        }
        trans->updateParams(intrinsic, distCoeffs);
    }
}

void CWhycon::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    // Setup timers...

    // Check if readjusting of camera is needed...

    memcpy(image->data, (void*)&msg->data[0], msg->step * msg->height);

    // Process segments and perform transformations...

    // Publishing information about tags 
    whycon_ros::msg::MarkerArray markerArray;
    markerArray.header = msg->header;

    visualization_msgs::msg::MarkerArray visualArray;

    for (int i = 0; i < numMarkers; i++) {
        if (currentSegmentArray[i].valid) {
            whycon_ros::msg::Marker marker;
            marker.id = currentSegmentArray[i].ID;
            marker.size = currentSegmentArray[i].size;

            // Convert to ROS standard Coordinate System
            marker.position.position.x = -objectArray[i].y;
            marker.position.position.y = -objectArray[i].z;
            marker.position.position.z = objectArray[i].x;

            tf2::Vector3 axis_vector(objectArray[i].pitch, objectArray[i].roll, objectArray[i].yaw);
            tf2::Vector3 up_vector(0.0, 0.0, 1.0);
            tf2::Vector3 marker_pos(marker.position.position.x, marker.position.position.y, marker.position.position.z);
            marker_pos.normalize();

            if (useAcuteFilter) {
                if (marker_pos.dot(axis_vector) > 0 && sqrt(marker.position.position.x * marker.position.position.x + marker.position.position.z * marker.position.position.z) < maxDetectionDistance) {
                    tf2::Vector3 right_vector = axis_vector.cross(up_vector);
                    right_vector.normalize();
                    tf2::Quaternion quat(right_vector, -1.0 * acos(axis_vector.dot(up_vector)));
                    quat.normalize();
                    marker.position.orientation = tf2::toMsg(quat);
                    marker.rotation.x = objectArray[i].pitch;
                    marker.rotation.y = objectArray[i].roll;
                    marker.rotation.z = objectArray[i].yaw;
                    markerArray.markers.push_back(marker);

                    visualization_msgs::msg::Marker visualMarker;
                    visualMarker.header = msg->header;
                    visualMarker.ns = "whycon";
                    visualMarker.id = (identify) ? marker.id : i;
                    visualMarker.type = visualization_msgs::msg::Marker::SPHERE;
                    visualMarker.action = visualization_msgs::msg::Marker::MODIFY;
                    visualMarker.pose = marker.position;
                    visualMarker.scale.x = circleDiameter;  // meters
                    visualMarker.scale.y = circleDiameter;
                    visualMarker.scale.z = 0.01;
                    visualMarker.color.r = 0.0;
                    visualMarker.color.g = 1.0;
                    visualMarker.color.b = 0.0;
                    visualMarker.color.a = 1.0;
                    visualMarker.lifetime = rclcpp::Duration::from_seconds(0.2);  // sec
                    visualArray.markers.push_back(visualMarker);
                }
            } else {
                if (sqrt(marker.position.position.x * marker.position.position.x + marker.position.position.z * marker.position.position.z) < maxDetectionDistance) {
                    tf2::Vector3 right_vector = axis_vector.cross(up_vector);
                    right_vector.normalize();
                    tf2::Quaternion quat(right_vector, -1.0 * acos(axis_vector.dot(up_vector)));
                    quat.normalize();
                    marker.position.orientation = tf2::toMsg(quat);
                    marker.rotation.x = objectArray[i].pitch;
                    marker.rotation.y = objectArray[i].roll;
                    marker.rotation.z = objectArray[i].yaw;
                    markerArray.markers.push_back(marker);

                    visualization_msgs::msg::Marker visualMarker;
                    visualMarker.header = msg->header;
                    visualMarker.ns = "whycon";
                    visualMarker.id = (identify) ? marker.id : i;
                    visualMarker.type = visualization_msgs::msg::Marker::SPHERE;
                    visualMarker.action = visualization_msgs::msg::Marker::MODIFY;
                    visualMarker.pose = marker.position;
                    visualMarker.scale.x = circleDiameter;  // meters
                    visualMarker.scale.y = circleDiameter;
                    visualMarker.scale.z = 0.01;
                    visualMarker.color.r = 0.0;
                    visualMarker.color.g = 1.0;
                    visualMarker.color.b = 0.0;
                    visualMarker.color.a = 1.0;
                    visualMarker.lifetime = rclcpp::Duration::from_seconds(0.2);  // sec
                    visualArray.markers.push_back(visualMarker);
                }
            }
        }
    }

    // Only publish if a certain amount of markers are detected
    if (markerArray.markers.size() >= minDetectionsToPublish) {
        markers_pub->publish(markerArray);
        visual_pub->publish(visualArray);
    }

    // Update GUI if enabled
    if (useGui) {
        gui->drawImage(image);
        gui->drawTimeStats(evalTime, numMarkers);
        gui->displayHelp(displayHelp);
        gui->guideCalibration(calibNum, fieldLength, fieldWidth);
    }

    for (int i = 0; i < numMarkers && useGui && drawCoords; i++) {
        if (currentSegmentArray[i].valid) gui->drawStats(currentSegmentArray[i].minx - 30, currentSegmentArray[i].maxy, objectArray[i], trans->transformType == TRANSFORM_2D);
    }

    if (autocalibrate && numFound == numMarkers) autocalibration();
    if (calibNum < 4) manualcalibration();

    if (useGui) gui->update();
    if (useGui) processKeys();
}

void CWhycon::manualcalibration() {
    if (currentSegmentArray[0].valid) {
        STrackedObject o = objectArray[0];
        moveOne = moveVal;

        // Object found - add to buffer
        if (calibStep < calibrationSteps) calibTmp[calibStep++] = o;

        // Calculate object position if buffer contains enough data
        if (calibStep == calibrationSteps) {
            o.x = o.y = o.z = 0;
            for (int k = 0; k < calibrationSteps; k++) {
                o.x += calibTmp[k].x;
                o.y += calibTmp[k].y;
                o.z += calibTmp[k].z;
            }
            o.x = o.x / calibrationSteps;
            o.y = o.y / calibrationSteps;
            o.z = o.z / calibrationSteps;
            if (calibNum < 4) {
                calib[calibNum++] = o;
            }

            // Last object needed to establish transform
            if (calibNum == 4) {
                trans->calibrate2D(calib, fieldLength, fieldWidth);
                trans->calibrate3D(calib, fieldLength, fieldWidth);
                calibNum++;
                numMarkers = wasMarkers;
                trans->saveCalibration(calibDefPath.c_str());
                trans->transformType = lastTransformType;
                detectorArray[0]->localSearch = false;
            }
            calibStep++;
        }
    }
}

void CWhycon::autocalibration() {
    // Autocalibration logic...
}

void CWhycon::processKeys() {
    // Process mouse events
    while (SDL_PollEvent(&event)) {
        if (event.type == SDL_MOUSEBUTTONDOWN) {
            if (calibNum < 4 && calibStep > calibrationSteps) {
                calibStep = 0;
                trans->transformType = TRANSFORM_NONE;
            }
            if (numMarkers > 0) {
                currentSegmentArray[numMarkers - 1].x = event.motion.x * guiScale;
                currentSegmentArray[numMarkers - 1].y = event.motion.y * guiScale;
                currentSegmentArray[numMarkers - 1].valid = true;
                detectorArray[numMarkers - 1]->localSearch = true;
            }
        }
    }

    // Process keys
    keys = SDL_GetKeyboardState(nullptr);
    bool shiftPressed = keys[SDL_SCANCODE_RSHIFT] || keys[SDL_SCANCODE_LSHIFT];

    // Program control
    if (keys[SDL_SCANCODE_ESCAPE]) stop = true;
    if (keys[SDL_SCANCODE_SPACE] && lastKeys[SDL_SCANCODE_SPACE] == false) {
        moveOne = 100000000;
        moveVal = 10000000;
    }
    if (keys[SDL_SCANCODE_P] && lastKeys[SDL_SCANCODE_P] == false) {
        moveOne = 1;
        moveVal = 0;
    }

    if (keys[SDL_SCANCODE_M] && lastKeys[SDL_SCANCODE_M] == false) printf("SAVE %03f %03f %03f %03f %03f %03f\n", objectArray[0].x, objectArray[0].y, objectArray[0].z, objectArray[0].d, currentSegmentArray[0].m0, currentSegmentArray[0].m1);
    if (keys[SDL_SCANCODE_N] && lastKeys[SDL_SCANCODE_N] == false) printf("SEGM %03f %03f %03f %03f\n", currentSegmentArray[0].x, currentSegmentArray[0].y, currentSegmentArray[0].m0, currentSegmentArray[0].m1);
    if (keys[SDL_SCANCODE_S] && lastKeys[SDL_SCANCODE_S] == false) image->saveBmp();

    // Initiate autocalibration
    if (keys[SDL_SCANCODE_A] && lastKeys[SDL_SCANCODE_A] == false) {
        calibStep = 0;
        lastTransformType = trans->transformType;
        wasMarkers = numMarkers;
        autocalibrate = true;
        trans->transformType = TRANSFORM_NONE;
    }

    // Manual calibration
    if (keys[SDL_SCANCODE_R] && lastKeys[SDL_SCANCODE_R] == false) {
        calibNum = 0;
        wasMarkers = numMarkers;
        numMarkers = 1;
    }

    // Debugging
    if (keys[SDL_SCANCODE_L] && lastKeys[SDL_SCANCODE_L] == false) drawCoords = drawCoords == false;
    if (keys[SDL_SCANCODE_D] && lastKeys[SDL_SCANCODE_D] == false) {
        for (int i = 0; i < numMarkers; i++) {
            detectorArray[i]->draw = detectorArray[i]->draw == false;
            detectorArray[i]->debug = detectorArray[i]->debug == false;
            decoder->debugSegment = decoder->debugSegment == false;
        }
    }

    // Transformations
    if (keys[SDL_SCANCODE_1] && lastKeys[SDL_SCANCODE_1] == false) trans->transformType = TRANSFORM_NONE;
    if (keys[SDL_SCANCODE_2] && lastKeys[SDL_SCANCODE_2] == false) trans->transformType = TRANSFORM_2D;
    if (keys[SDL_SCANCODE_3] && lastKeys[SDL_SCANCODE_3] == false) trans->transformType = TRANSFORM_3D;

    // Display help
    if (keys[SDL_SCANCODE_H] && lastKeys[SDL_SCANCODE_H] == false) displayHelp = displayHelp == false;

    // Adjust number of robots to be searched for
    if (keys[SDL_SCANCODE_KP_PLUS] || keys[SDL_SCANCODE_EQUALS]) numMarkers++;
    if (keys[SDL_SCANCODE_KP_MINUS] || keys[SDL_SCANCODE_MINUS]) numMarkers--;

    // Store the key states
    memcpy(lastKeys, keys, keyNumber);
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    auto whycon = std::make_shared<CWhycon>();
    whycon->init(argv[1], argv[2]);

    rclcpp::spin(whycon);
    rclcpp::shutdown();

    return 0;
}
