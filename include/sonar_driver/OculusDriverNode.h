
#pragma once

#include <stdint.h>
#include <vector>
#include <functional>

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/header.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/fluid_pressure.hpp"
#include "sensor_msgs/msg/temperature.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"

#include <sonar_driver_interfaces/msg/sonar_configuration.hpp>
#include <sonar_driver_interfaces/msg/sonar_configuration_change.hpp>
#include <sonar_driver_interfaces/msg/sonar_bearings.hpp>   


#include <sonar_driver/sonardevices/Sonar.h>
#include <sonar_driver/sonardevices/OculusSonar.h>

class OculusDriverNode : public rclcpp::Node
{
public:
    OculusDriverNode(const std::string& nodeName);
    
    std::unique_ptr<OculusSonar> sonar_;


    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_img;
    rclcpp::Publisher<sensor_msgs::msg::FluidPressure>::SharedPtr pub_pressure;
    rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr pub_temperature;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr pub_orientation;
    rclcpp::Publisher<sonar_driver_interfaces::msg::SonarConfiguration>::SharedPtr pub_configuration;
    rclcpp::Publisher<sonar_driver_interfaces::msg::SonarBearings>::SharedPtr pub_bearings;

    rclcpp::Subscription<sonar_driver_interfaces::msg::SonarConfigurationChange>::SharedPtr sub_reconfiguration;

    void cb_simplePingResult(std::unique_ptr<SonarImage>& image);

    bool publishIt = false;

protected:

    void updateCommonHeader();

    void publishImage(std::unique_ptr<SonarImage>& image);
    void publishCurrentConfig();

    void publishAdditionalInformation1(OculusSonarImage &image);
    void publishAdditionalInformation2(OculusSonarImage2 &image);
    void publishPressure(double pressure);
    void publishTemperature(double temperature);

    void cb_reconfiguration(const sonar_driver_interfaces::msg::SonarConfigurationChange::SharedPtr msg);

    std_msgs::msg::Header commonHeader_;
    sensor_msgs::msg::Image msg_img_;
    sensor_msgs::msg::Image* msg_img_raw;


};

