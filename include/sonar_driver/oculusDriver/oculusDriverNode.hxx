// File: oculusDriverNode.hxx
#include <stdint.h>
#include <vector>
#include <functional>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/fluid_pressure.hpp"
#include "sensor_msgs/msg/temperature.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "sonar_driver_interfaces/msg/sonar_configuration.hpp"

#ifndef OCULUSDRIVERNODE_HXX
#define OCULUSDRIVERNODE_HXX

class OculusDriverNode : public rclcpp::Node
{
public:
    static OculusDriverNode *getInstace();
    OculusDriverNode(const char *nodeName);
    ~OculusDriverNode();
    void addConfigurationListener(std::function<void(sonar_driver_interfaces::msg::SonarConfiguration::SharedPtr)> callback);

    std_msgs::msg::Header *commonHeader;
    sensor_msgs::msg::Image *sonarImage;
    sensor_msgs::msg::FluidPressure *sonarPressure;
    sensor_msgs::msg::Temperature *sonarTemperature;
    geometry_msgs::msg::Vector3Stamped *sonarOrientation;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr imagePublisher;
    rclcpp::Publisher<sensor_msgs::msg::FluidPressure>::SharedPtr pressurePublisher;
    rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temperaturePublisher;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr orientationPublisher;
    rclcpp::Publisher<sonar_driver_interfaces::msg::SonarConfiguration>::SharedPtr configurationPublisher;
    rclcpp::Subscription<sonar_driver_interfaces::msg::SonarConfiguration>::SharedPtr configurationSubscriber;

protected:
    inline static OculusDriverNode *instance = nullptr;

    std::vector<std::function<void(sonar_driver_interfaces::msg::SonarConfiguration::SharedPtr)>> *configurationListeners;
    void configurationCallback(sonar_driver_interfaces::msg::SonarConfiguration::SharedPtr msg);
};

#endif