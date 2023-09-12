#define _SILENCE_ALL_CXX17_DEPRECATION_WARNINGS

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/fluid_pressure.hpp"
#include "sensor_msgs/msg/temperature.hpp"

#include "sonar_driver_interfaces/msg/sonar_configuration.hpp"
#include "sonar_driver_interfaces/msg/sonar_configuration_change.hpp"

#include <cstdlib>
#include <chrono>
#include <string>
#include <thread>
#include <iostream>

#include <sonar_driver/sonardevices/sonardevices.hxx>
#include <sonar_driver/oculusDriver/oculusDriverNode.hxx>

using namespace SonarDevices;

inline void store_system_time(int32_t &seconds, uint32_t &nanoseconds)
{
    // Get time from system:
    auto now = std::chrono::high_resolution_clock::now();
    auto nanos = now.time_since_epoch();
    seconds = (int32_t)(nanos.count() / (uint64_t)1e9);
    nanoseconds = (uint32_t)(nanos.count() % (uint64_t)1e9);
}

void publish_image(SonarImage *image, int32_t seconds, uint32_t nanos)
{
    // Get OculusDriverNode
    OculusDriverNode *oculusNode = OculusDriverNode::getInstace();

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

void config_callback(sonar_driver_interfaces::msg::SonarConfigurationChange::SharedPtr config, Sonar *sonar)
{
    // Check if not null
    if (config == nullptr)
    {
        return;
    }

    sonar->configure(
        config->fire_mode,
        config->range,
        config->gain,
        config->speed_of_sound,
        config->salinity,
        config->gain_assist,
        config->gamma,
        config->net_speed_limit
    );

    sonar->setPingRate(config->ping_rate);
}

void get_and_publish_config(Sonar *sonar, int32_t seconds, uint32_t nanos)
{
    // Get OculusDriverNode
    OculusDriverNode *oculusNode = OculusDriverNode::getInstace();

    sonar_driver_interfaces::msg::SonarConfiguration configuration;

    // Set the header
    configuration.header.stamp.sec = seconds;
    configuration.header.stamp.nanosec = nanos;
    configuration.header.frame_id = "base_oculus";

    // Fill message with configuration data from sonar
    configuration.fire_mode = sonar->getFireMode();
    configuration.frequency = sonar->getOperatingFrequency();
    configuration.ping_rate = sonar->getPingRate();
    configuration.beam_count = sonar->getBeamCount();
    configuration.beam_separation = sonar->getBeamSeparation();
    configuration.min_range = sonar->getMinimumRange();
    configuration.max_range = sonar->getMaximumRange();
    configuration.current_range = sonar->getCurrentRange();
    configuration.range_resolution = sonar->getRangeResolution();
    configuration.range_count = sonar->getRangeBinCount();
    configuration.horz_fov = sonar->getHorzFOV();
    configuration.vert_fov = sonar->getVertFOV();
    configuration.angular_resolution = sonar->getAngularResolution();
    configuration.gain = sonar->getCurrentGain();
    configuration.gain_assist = sonar->gainAssistEnabled();
    configuration.gamma = sonar->getGamma();
    configuration.speed_of_sound = sonar->getSpeedOfSound();
    configuration.salinity = sonar->getSalinity();
    configuration.temperature = sonar->getTemperature();
    configuration.pressure = sonar->getPressure();
    configuration.net_speed_limit = sonar->getNetworkSpeedLimit();

    // Publish the message on the OculusDriverNode
    oculusNode->configurationPublisher->publish(configuration);
}

void sonar_image_callback(Sonar *sonar, SonarImage *image)
{
    // Get time from system:
    int32_t seconds;
    uint32_t nanos;
    store_system_time(seconds, nanos);

    publish_image(image, seconds, nanos);
    get_and_publish_config(sonar, seconds, nanos);
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
    sonar->configure(2, 5.0, 80.0, 0.0, 0.0, false, 255, 0xff);
    sonar->setPingRate(40);

    // Initialize ROS Node
    logMessage("Initializing ROS 2 integration...\n");
    rclcpp::init(argc, argv);

    // Create OculusDriverNode and a shared pointer to it
    std::shared_ptr<OculusDriverNode> node = std::make_shared<OculusDriverNode>("oculus_driver");
    
    logMessage("Registering sonar image callbacks...\n");
    sonar->registerCallback(sonar_image_callback);

    logMessage("Registering sonar configuration change listeners...\n");
    std::function<void(sonar_driver_interfaces::msg::SonarConfigurationChange::SharedPtr)> listener;
    listener = [sonar](sonar_driver_interfaces::msg::SonarConfigurationChange::SharedPtr config) {
        config_callback(config, sonar);
    };
    node->addConfigurationListener(listener);

    logMessage("Node started!\n");
    while (rclcpp::ok())
    {
        rclcpp::spin_some(node);
        sonar->fire();
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    // Tear down connections and clean up
    logMessage("Tearing down node!\n");

    rclcpp::shutdown();

    sonar->disconnect();
    delete sonar;
    sonar = nullptr;

    exit(EXIT_SUCCESS);
}