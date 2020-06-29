#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/fluid_pressure.hpp"
#include "sensor_msgs/msg/temperature.hpp"

#include <cstdlib>
#include <chrono>
#include <string>
#include <thread>
#include <iostream>

#include <sonar_driver/sonardevices/sonardevices.hxx>
#include <sonar_driver/oculusDriver/oculusDriverNode.hxx>

using namespace SonarDevices;

void sonar_image_callback(SonarImage *image)
{
    // Get OculusDriverNode
    OculusDriverNode *oculusNode = OculusDriverNode::getInstace();

    // Get time from system:
    auto now = std::chrono::high_resolution_clock::now();
    auto nanoseconds = now.time_since_epoch();
    int32_t seconds = (int32_t)(nanoseconds.count() / (uint64_t)1e9);
    uint32_t nanos = (uint32_t)(nanoseconds.count() % (uint64_t)1e9);

    // Update header for all messages
    oculusNode->commonHeader->stamp.sec = seconds;
    oculusNode->commonHeader->stamp.nanosec = nanos;
    oculusNode->commonHeader->frame_id = "base_oculus";

    // Create the message from the sonar image
    oculusNode->sonarImage->header = *(oculusNode->commonHeader);
    oculusNode->sonarImage->height = image->imageHeight;
    oculusNode->sonarImage->width = image->imageWidth;
    oculusNode->sonarImage->step = image->imageWidth; // since data of sonar image is uint8

    uint32_t size = oculusNode->sonarImage->height * oculusNode->sonarImage->width;

    // Adapt size of the sonar image if not big enough
    oculusNode->sonarImage->data.resize(size);
    memcpy(&oculusNode->sonarImage->data[0], image->data, size);

    // Publish the message on the OculusDriverNode
    oculusNode->imagePublisher->publish(*(oculusNode->sonarImage));

    // Check image type
    switch (image->imageType)
    {
    case SonarImageType::OculusSonarImageObject:
    {
        OculusSonarImage *osi = (OculusSonarImage *) image;
        // Pressure
        oculusNode->sonarPressure->header = *(oculusNode->commonHeader);
        oculusNode->sonarPressure->fluid_pressure = osi->pressure;
        oculusNode->pressurePublisher->publish(*(oculusNode->sonarPressure));
        // Temperature
        oculusNode->sonarTemperature->header = *(oculusNode->commonHeader);
        oculusNode->sonarTemperature->temperature = osi->temperature;
        oculusNode->temperaturePublisher->publish(*(oculusNode->sonarTemperature));
        break;
    }
    case SonarImageType::OculusSonarImage2Object:
    {
        OculusSonarImage2 *osi2 = (OculusSonarImage2 *) image;
        // Pressure
        oculusNode->sonarPressure->header = *(oculusNode->commonHeader);
        oculusNode->sonarPressure->fluid_pressure = osi2->pressure;
        oculusNode->pressurePublisher->publish(*(oculusNode->sonarPressure));
        // Temperature
        oculusNode->sonarTemperature->header = *(oculusNode->commonHeader);
        oculusNode->sonarTemperature->temperature = osi2->temperature;
        oculusNode->temperaturePublisher->publish(*(oculusNode->sonarTemperature));
        // Orientation 
        oculusNode->sonarOrientation->header = *(oculusNode->commonHeader);
        oculusNode->sonarOrientation->vector.x = osi2->heading;
        oculusNode->sonarOrientation->vector.y = osi2->pitch;
        oculusNode->sonarOrientation->vector.z = osi2->roll;
        oculusNode->orientationPublisher->publish(*(oculusNode->sonarOrientation));
        break;
    }
    default:
        break;
    }
}

void logMessage(const char *msg)
{
    std::cout << msg;
}

int main(int argc, char *argv[])
{
    // Initialize and connect to sonar
    logMessage("Looking for oculus sonar...\n");
    Sonar *sonar = new OculusSonar();
    sonar->findAndConnect();
    if (sonar->getState() != SonarState::Connected)
    {
        logMessage("Could not connect!\n");
        exit(EXIT_FAILURE);
    }
    logMessage("Connected to sonar at ");
    logMessage(sonar->getLocation());
    logMessage("\n");

    // Configure sonar
    logMessage("Configuring sonar...\n");
    sonar->configure(2, 1.0, 0.0, 0.0, 0.0, false, 0.59, 100);
    sonar->setPingRate(40);

    // Initialize ROS Node
    logMessage("Initializing ROS 2 integration...\n");
    rclcpp::init(argc, argv);
    rclcpp::Node *node = new OculusDriverNode("oculus_driver");
    
    logMessage("Registering sonar image callbacks...\n");
    sonar->registerCallback(sonar_image_callback);

    logMessage("Node started!\n");
    while (rclcpp::ok())
    {
        sonar->fire();
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    // Tear down connections and clean up
    logMessage("Tearing down node!\n");
    delete node;
    node = nullptr;
    rclcpp::shutdown();

    sonar->disconnect();
    delete sonar;
    sonar = nullptr;

    exit(EXIT_SUCCESS);
}