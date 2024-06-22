#ifndef CAMERA_CONTROLLER_PLUGIN_HH_
#define CAMERA_CONTROLLER_PLUGIN_HH_

#include <gazebo/common/Plugin.hh>
#include <gazebo/sensors/CameraSensor.hh>
#include <gazebo/rendering/Camera.hh>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

namespace gazebo
{
  class CameraPlugin : public SensorPlugin
  {
  public:
    void Load(sensors::SensorPtr _sensor, sdf::ElementPtr /*_sdf*/) override;

  private:
    void OnCameraUpdate();

    sensors::CameraSensorPtr cameraSensor;
    event::ConnectionPtr updateConnection;
    std::unique_ptr<ros::NodeHandle> nh;
    ros::Publisher imagePub;
  };
}

#endif // GAZEBO_CAMERA_PLUGIN_HH_