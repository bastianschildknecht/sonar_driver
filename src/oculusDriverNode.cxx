#include "oculusDriverNode.hxx"
#include <stdint.h>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/fluid_pressure.hpp"
#include "sensor_msgs/msg/temperature.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"

OculusDriverNode *OculusDriverNode::getInstace()
{
    return OculusDriverNode::instance;
}

OculusDriverNode::OculusDriverNode(const char *nodeName) : rclcpp::Node(nodeName)
{
    // Create messages
    commonHeader = new std_msgs::msg::Header();
    sonarImage = new sensor_msgs::msg::Image();
    sonarImage->height = 1;
    sonarImage->width = 1;
    sonarImage->encoding = "mono8";
    sonarImage->is_bigendian = false;
    sonarImage->step = sonarImage->width;
    sonarImage->data = (uint8_t *)malloc(sizeof(uint8_t));
    imageDataSize = 1;
    sonarPressure = new sensor_msgs::msg::FluidPressure();
    sonarPressure->variance = 0.0;
    sonarTemperature = new sensor_msgs::msg::Temperature();
    sonarTemperature->variance = 0.0;
    sonarOrientation = new geometry_msgs::msg::Vector3Stamped();

    // Initialize publishers
    imagePublisher = this->create_publisher<sensor_msgs::msg::Image>("image", 10);
    pressurePublisher = this->create_publisher<sensor_msgs::msg::FluidPressure>("pressure", 10);
    temperaturePublisher = this->create_publisher<sensor_msgs::msg::Temperature>("temperature", 10);

    // Singleton pattern
    if (OculusDriverNode::instance == nullptr)
    {
        OculusDriverNode::instance = this;
    }
}

OculusDriverNode::~OculusDriverNode()
{
    delete commonHeader;
    commonHeader = nullptr;
    free(sonarImage->data);
    sonarImage->data = nullptr;
    delete sonarImage;
    sonarImage = nullptr;
    delete sonarPressure;
    sonarPressure = nullptr;
    delete sonarTemperature;
    sonarTemperature = nullptr;
    delete sonarOrientation;
    sonarOrientation = nullptr;
}