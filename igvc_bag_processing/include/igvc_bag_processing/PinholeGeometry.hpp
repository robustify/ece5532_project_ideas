#pragma once
// Message types
#include <geometry_msgs/Point32.h>

// TF lookup headers
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

// Image processing and camera geometry headers
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>

namespace igvc_bag_processing {

  class PinholeGeometry {
    public:
      PinholeGeometry(const std::string& _vehicle_frame, const std::string& _camera_frame, const std::string& _lidar_frame);
      bool lookup_static_transforms();
      void set_camera_info(const sensor_msgs::CameraInfo& camera_info);
      cv::Point lidar_to_pixel(const tf2::Vector3& lidar_point);
      geometry_msgs::Point32 pixel_to_footprint(const cv::Point& pixel_point);

    private:

      tf2_ros::TransformListener tf_listener;
      tf2_ros::Buffer tf_buffer;
      tf2::Transform camera_transform; // Coordinate transformation from footprint to camera
      tf2::Transform lidar_transform;  // Coordinate transformation from footprint to lidar
      std::string vehicle_frame;
      std::string camera_frame;
      std::string lidar_frame;
      bool looked_up_transforms;

      image_geometry::PinholeCameraModel pinhole_model;

      bool geometry_ok();
  };

}