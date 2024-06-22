#include <gazebo/common/Plugin.hh>
#include <gazebo/sensors/Sensor.hh>
#include <gazebo/sensors/CameraSensor.hh>
#include <gazebo/rendering/Camera.hh>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

namespace gazebo
{
  class CameraPlugin : public SensorPlugin
  {
  public:
    void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf) override
    {
      // Get the camera sensor
      this->cameraSensor = std::dynamic_pointer_cast<sensors::CameraSensor>(_sensor);

      if (!this->cameraSensor)
      {
        gzerr << "CameraPlugin requires a CameraSensor.\n";
        return;
      }

      // Read parameters from URDF
      if (_sdf->HasElement("alwaysOn"))
      {
        this->alwaysOn = _sdf->Get<bool>("alwaysOn");
      }

      if (_sdf->HasElement("updateRate"))
      {
        this->updateRate = _sdf->Get<double>("updateRate");
      }

      if (_sdf->HasElement("cameraName"))
      {
        this->cameraName = _sdf->Get<std::string>("cameraName");
      }

      if (_sdf->HasElement("frameName"))
      {
        this->frameName = _sdf->Get<std::string>("frameName");
      }

      if (_sdf->HasElement("topicName"))
      {
        this->topicName = _sdf->Get<std::string>("topicName");
      }

      if (_sdf->HasElement("cameraInfoTopicName"))
      {
        this->cameraInfoTopicName = _sdf->Get<std::string>("cameraInfoTopicName");
      }

      // Connect to the camera sensor's update event
      this->updateConnection = this->cameraSensor->ConnectUpdated(
          std::bind(&CameraPlugin::OnCameraUpdate, this));

      // Initialize ROS
      if (!ros::isInitialized())
      {
        int argc = 0;
        char **argv = nullptr;
        ros::init(argc, argv, "camera_plugin");
      }
      this->nh.reset(new ros::NodeHandle("camera_plugin"));

      // Create publishers for the image and camera info topics
      this->imagePub = this->nh->advertise<sensor_msgs::Image>(this->topicName, 1);
      this->infoPub = this->nh->advertise<sensor_msgs::CameraInfo>(this->cameraInfoTopicName, 1);
    }

    // Function called whenever the camera sensor is updated
    void OnCameraUpdate()
    {
      // Get the camera image
      const auto image = this->cameraSensor->ImageData();

      // Publish the image to a ROS topic
      sensor_msgs::Image ros_image;
      ros_image.header.stamp = ros::Time::now();
      ros_image.header.frame_id = this->frameName;
      ros_image.width = image->Width();
      ros_image.height = image->Height();
      ros_image.encoding = "rgb8"; // Adjust the encoding based on your needs
      ros_image.step = ros_image.width * 3; // 3 channels (RGB)
      ros_image.data.resize(ros_image.height * ros_image.step);
      memcpy(&ros_image.data[0], image->Data(), ros_image.data.size());

      imagePub.publish(ros_image);

      // Publish camera info
      sensor_msgs::CameraInfo cam_info_msg;
      cam_info_msg.header = ros_image.header;
      cam_info_msg.height = ros_image.height;
      cam_info_msg.width = ros_image.width;
      // Fill other camera info parameters here

      infoPub.publish(cam_info_msg);
    }

  private:
    sensors::CameraSensorPtr cameraSensor;
    event::ConnectionPtr updateConnection;
    std::unique_ptr<ros::NodeHandle> nh;
    ros::Publisher imagePub;
    ros::Publisher infoPub;
    bool alwaysOn;
    double updateRate;
    std::string cameraName = "camera";
    std::string frameName = "camera_link";
    std::string topicName = "camera/image_raw";
    std::string cameraInfoTopicName = "camera/camera_info";
  };

  // Register this plugin with the simulator
  GZ_REGISTER_SENSOR_PLUGIN(CameraPlugin)
}
