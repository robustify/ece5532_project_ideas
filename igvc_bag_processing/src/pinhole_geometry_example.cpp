#include <ros/ros.h>
#include <igvc_bag_processing/PinholeGeometry.hpp>

std::shared_ptr<igvc_bag_processing::PinholeGeometry> pinhole_geometry;

void recvCameraInfo(const sensor_msgs::CameraInfoConstPtr& msg)
{
  // The CameraInfo message from the bag must be sent to the PinholeGeometry class for it to work
  pinhole_geometry->set_camera_info(*msg);
}

void timerCallback(const ros::TimerEvent& event)
{
  // This timer is just for illustrative purposes... In your code, use the lidar_to_pixel and pixel_to_footprint functions wherever you need to use them

  // Projecting a point that could come from a LaserScan into the image and finding the pixel it hits
  tf2::Vector3 example_lidar_point(1.5, 0.0, 0.0);
  cv::Point output_pixel = pinhole_geometry->lidar_to_pixel(example_lidar_point);
  ROS_INFO_STREAM("Lidar point: " << example_lidar_point.x() << ", " << example_lidar_point.y() << ", " << example_lidar_point.z());
  ROS_INFO_STREAM("Corresponding pixel: " << output_pixel.x << ", " << output_pixel.y);

  // Projecting a pixel of interest into the vehicle frame, assuming the detected pixel represents a point on the ground.
  cv::Point example_pixel_point(400, 300);
  geometry_msgs::Point32 output_footprint_point = pinhole_geometry->pixel_to_footprint(example_pixel_point);
  ROS_INFO_STREAM("Image pixel: " << example_pixel_point.x << ", " << example_pixel_point.y);
  ROS_INFO_STREAM("Corresponding point in footprint frame: " << output_footprint_point.x << ", " << output_footprint_point.y << ", " << output_footprint_point.z << "\n\n\n");
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pinhole_geometry_example");
  ros::NodeHandle n;
  ros::NodeHandle pn("~");

  std::string vehicle_frame;
  std::string camera_frame;
  std::string lidar_frame;
  pn.param("vehicle_frame", vehicle_frame, std::string("base_footprint"));
  pn.param("camera_frame", camera_frame, std::string("stereo_left"));
  pn.param("lidar_frame", lidar_frame, std::string("laser"));

  // Initialize class pointer after initializing ROS, passing the TF frame IDs to the constructor
  pinhole_geometry = std::make_shared<igvc_bag_processing::PinholeGeometry>(vehicle_frame, camera_frame, lidar_frame);

  // The PinholeGeometry class requires TF frame transforms between footprint, camera, and lidar.
  // This loop attempts to look up the transforms once per second until it succeeds or the node is stopped
  ROS_INFO("Waiting for TF lookups to complete");
  while (ros::ok() && !pinhole_geometry->lookup_static_transforms()) {
    ros::Duration(1.0).sleep();
  }
  ROS_INFO("Done looking up TF transforms");

  // Set up a subscriber to the CameraInfo topic from the bag
  ros::Subscriber sub_camera_info = n.subscribe("camera_info", 1, recvCameraInfo);

  ros::Timer timer = n.createTimer(ros::Duration(1.0), timerCallback);

  ros::spin();
}