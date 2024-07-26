cmake_minimum_required(VERSION 3.5)
project(whycon_ros)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Find SDL2 and SDL2_ttf or set manually if not found
find_package(SDL2 REQUIRED)
find_package(SDL2_ttf QUIET)  # QUIET prevents CMake from failing immediately if the package isn't found

if(NOT SDL2_ttf_FOUND)
  message(STATUS "SDL2_ttf not found with find_package, setting manually")
  set(SDL2_TTF_INCLUDE_DIRS /usr/include/SDL2)  # Adjust if necessary
  find_library(SDL2_TTF_LIBRARIES NAMES SDL2_ttf PATHS /usr/lib/x86_64-linux-gnu)  # Adjust library path as necessary
endif()


# Message generation
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Marker.msg"
  "msg/MarkerArray.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

# Specify additional locations of header files
include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${tf2_INCLUDE_DIRS}
  ${tf2_ros_INCLUDE_DIRS}
  ${tf2_geometry_msgs_INCLUDE_DIRS}
  ${image_transport_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${SDL2_INCLUDE_DIRS}
  ${SDL2_TTF_INCLUDE_DIRS}
  ${whycon_ros_INCLUDE_DIRS}  # Include generated headersg
  /opt/ros/humble/include  # Explicitly add the ROS 2 include directory
)

# Declare C++ libraries
add_library(whycon_detector src/CCircleDetect.cpp)
add_library(whycon_image src/CRawImage.cpp)
add_library(whycon_timer src/CTimer.cpp)
add_library(whycon_sysmath src/sysmat.cpp)
add_library(whycon_transform src/CTransformation.cpp)
add_library(whycon_necklace src/CNecklace.cpp)
add_library(whycon_gui src/CGui.cpp)

# Link libraries
target_link_libraries(whycon_detector whycon_image)
target_link_libraries(whycon_transform whycon_sysmath ${rclcpp_LIBRARIES} ${OpenCV_LIBRARIES})
target_link_libraries(whycon_necklace whycon_detector whycon_transform whycon_image)
target_link_libraries(whycon_gui whycon_image whycon_transform SDL2 SDL2_ttf)

# Declare executables
add_executable(whycon_node src/CWhycon.cpp)
ament_target_dependencies(whycon_node
  rclcpp
  tf2
  tf2_ros
  tf2_geometry_msgs
  image_transport  # This should handle linking and including correctly
  visualization_msgs
  std_msgs
  geometry_msgs
  OpenCV
)


# Declare executables
target_link_libraries(whycon_node
  whycon_timer
  whycon_image
  whycon_detector
  whycon_transform
  whycon_gui
  whycon_necklace
  SDL2
  ${SDL2_TTF_LIBRARIES}  # Use the variable that might have been manually set
  ${rclcpp_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${tf2_LIBRARIES}
  ${tf2_ros_LIBRARIES}
  ${tf2_geometry_msgs_LIBRARIES}
)

# Install targets
install(TARGETS
  whycon_detector
  whycon_image
  whycon_timer
  whycon_sysmath
  whycon_transform
  whycon_necklace
  whycon_gui
  whycon_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install directories
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY etc/
  DESTINATION share/${PROJECT_NAME}/etc
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)


# Export package dependencies
ament_export_dependencies(rclcpp 
                          tf2 tf2_ros tf2_geometry_msgs 
                          image_transport cv_bridge OpenCV 
                          SDL2 SDL2_ttf)
ament_package()
