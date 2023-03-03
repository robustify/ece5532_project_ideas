#include <igvc_bag_processing/PinholeGeometry.hpp>

namespace igvc_bag_processing {

  PinholeGeometry::PinholeGeometry(const std::string& _vehicle_frame, const std::string& _camera_frame, const std::string& _lidar_frame)
  : tf_listener(tf_buffer),
    vehicle_frame(_vehicle_frame),
    camera_frame(_camera_frame),
    lidar_frame(_lidar_frame),
    looked_up_transforms(false)
  {
  }

  bool PinholeGeometry::lookup_static_transforms()
  {
    geometry_msgs::TransformStamped transform_lookup;
    try {
      transform_lookup = tf_buffer.lookupTransform(vehicle_frame, camera_frame, ros::Time(0));
      tf2::convert(transform_lookup.transform, camera_transform);
      transform_lookup = tf_buffer.lookupTransform(vehicle_frame, lidar_frame, ros::Time(0));
      tf2::convert(transform_lookup.transform, lidar_transform);
    } catch (tf2::TransformException& ex) {
      ROS_ERROR_STREAM(ex.what());
      return false;
    }
    looked_up_transforms = true;
    return true;
  }

  void PinholeGeometry::set_camera_info(const sensor_msgs::CameraInfo& camera_info) {
    pinhole_model.fromCameraInfo(camera_info);
  }

  cv::Point PinholeGeometry::lidar_to_pixel(const tf2::Vector3& lidar_point)
  {
    if (!geometry_ok()) {
      return cv::Point();
    }

    auto lidar_camera_point = (camera_transform.inverse() * lidar_transform) * lidar_point;
    return (cv::Point)pinhole_model.project3dToPixel(cv::Point3d(lidar_camera_point.x(), lidar_camera_point.y(), lidar_camera_point.z()));
  }

  geometry_msgs::Point32 PinholeGeometry::pixel_to_footprint(const cv::Point& pixel_point)
  {
    if (!geometry_ok()) {
      return geometry_msgs::Point32();
    }

    // Convert the input pixel coordinates into a 3d ray, where x and y are projected to the point where z is equal to 1.0
    cv::Point3d cam_frame_ray = pinhole_model.projectPixelTo3dRay((cv::Point2d)pixel_point);

    // Represent camera frame ray in footprint frame
    tf2::Vector3 footprint_frame_ray = camera_transform.getBasis() * tf2::Vector3(cam_frame_ray.x, cam_frame_ray.y, cam_frame_ray.z);

    // Using the concept of similar triangles, scale the unit vector such that the end is on the ground plane.
    double s = -camera_transform.getOrigin().z() / footprint_frame_ray.z();
    tf2::Vector3 ground_plane_ray = s * footprint_frame_ray;

    // Then add camera position offset to obtain the final coordinates in footprint frame
    tf2::Vector3 vehicle_frame_point = ground_plane_ray + camera_transform.getOrigin();

    // Fill output point with the result of the projection
    geometry_msgs::Point32 output;
    output.x = vehicle_frame_point.x();
    output.y = vehicle_frame_point.y();
    output.z = vehicle_frame_point.z();
    return output;
  }

  bool PinholeGeometry::geometry_ok()
  {
    bool is_ok = true;
    if (!looked_up_transforms) {
      ROS_ERROR("Static TF lookups have not been done");
      is_ok = false;
    }
    if (!pinhole_model.initialized()) {
      ROS_ERROR("Pinhole camera model not initialized with CameraInfo message");
      is_ok = false;
    }
    return is_ok;
  }

}