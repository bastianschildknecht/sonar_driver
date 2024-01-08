
#include <sonar_driver/OculusDriverNode.h>


OculusDriverNode::OculusDriverNode(const std::string& nodeName) : rclcpp::Node(nodeName){

    // Initialize publishers
    pub_img = this->create_publisher<sensor_msgs::msg::Image>("image", 10);
    pub_pressure = this->create_publisher<sensor_msgs::msg::FluidPressure>("pressure", 10);
    pub_temperature = this->create_publisher<sensor_msgs::msg::Temperature>("temperature", 10);
    pub_orientation = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("orientatiion", 10);
    pub_configuration = this->create_publisher<sonar_driver_interfaces::msg::SonarConfiguration>("configuration", 10);
    pub_bearings = this->create_publisher<sonar_driver_interfaces::msg::SonarBearings>("bearings", 10);

    sub_reconfiguration = this->create_subscription<sonar_driver_interfaces::msg::SonarConfigurationChange>(
        "reconfigure", 10, std::bind(&OculusDriverNode::cb_reconfiguration, this, std::placeholders::_1)
    );

}


void OculusDriverNode::publishImage(SonarImage &image){
    sensor_msgs::msg::Image msg_img;

    msg_img.encoding = "mono8";
    msg_img.is_bigendian = false;

    // Create the message from the sonar image
    msg_img.header = this->commonHeader_;
    msg_img.height = image.imageHeight;
    msg_img.width = image.imageWidth;
    msg_img.step = image.imageWidth; // since data of sonar image is uint8

    uint32_t size = msg_img.height * msg_img.width;

    memcpy(&msg_img.data[0], &image.data, size);

    // msg_img.data = *image.data;

    this->pub_img->publish(msg_img);
}

void OculusDriverNode::publishAdditionalInformation1(OculusSonarImage &image){
    this->publishPressure(image.pressure);
    this->publishTemperature(image.temperature);
}

void OculusDriverNode::publishAdditionalInformation2(OculusSonarImage2 &image){
   
    this->publishPressure(image.pressure);
    
    this->publishTemperature(image.temperature);

    geometry_msgs::msg::Vector3Stamped msg;
    msg.header = this->commonHeader_;
    msg.vector.x = image.heading;
    msg.vector.y = image.pitch;
    msg.vector.z = image.roll;
    this->pub_orientation->publish(msg);
}

void OculusDriverNode::publishPressure(double pressure){
    sensor_msgs::msg::FluidPressure msg;
    msg.header = this->commonHeader_;
    msg.fluid_pressure = pressure;
    this->pub_pressure->publish(msg);
}

void OculusDriverNode::publishTemperature(double temperature){
    sensor_msgs::msg::Temperature msg;
    msg.header = this->commonHeader_;
    msg.temperature = temperature;
    this->pub_temperature->publish(msg);
}


void OculusDriverNode::publishCurrentConfig(){
    sonar_driver_interfaces::msg::SonarConfiguration configuration;

    configuration.header = this->commonHeader_;

    // Fill message with configuration data from sonar
    configuration.fire_mode = sonar_->getFireMode();
    configuration.frequency = sonar_->getOperatingFrequency();
    configuration.ping_rate = sonar_->getPingRate();
    configuration.beam_count = sonar_->getBeamCount();
    configuration.beam_separation = sonar_->getBeamSeparation();
    configuration.min_range = sonar_->getMinimumRange();
    configuration.max_range = sonar_->getMaximumRange();
    configuration.current_range = sonar_->getCurrentRange();
    configuration.range_resolution = sonar_->getRangeResolution();
    configuration.range_count = sonar_->getRangeBinCount();
    configuration.horz_fov = sonar_->getHorzFOV();
    configuration.vert_fov = sonar_->getVertFOV();
    configuration.angular_resolution = sonar_->getAngularResolution();
    configuration.gain = sonar_->getCurrentGain();
    configuration.gain_assist = sonar_->gainAssistEnabled();
    configuration.gamma = sonar_->getGamma();
    configuration.speed_of_sound = sonar_->getSpeedOfSound();
    configuration.salinity = sonar_->getSalinity();
    configuration.temperature = sonar_->getTemperature();
    configuration.pressure = sonar_->getPressure();
    configuration.net_speed_limit = sonar_->getNetworkSpeedLimit();

    // Publish the message on the OculusDriverNode
    pub_configuration->publish(configuration);

    // Publish the bearing table 
    sonar_driver_interfaces::msg::SonarBearings msg_bearings;
    msg_bearings.header = this->commonHeader_;
    msg_bearings.bearings = sonar_->getBearingTable();
    this->pub_bearings->publish(msg_bearings);
    
}


void OculusDriverNode::cb_reconfiguration(const sonar_driver_interfaces::msg::SonarConfigurationChange::SharedPtr msg){
    sonar_->configure(
        msg->fire_mode,
        msg->range,
        msg->gain,
        msg->speed_of_sound,
        msg->salinity,
        msg->gain_assist,
        msg->gamma,
        msg->net_speed_limit
    );

    sonar_->setPingRate(msg->ping_rate);
}

/// @brief Method to execute upon receival of a simplePingResult Message 
/// @param sonar 
/// @param image 
void OculusDriverNode::cb_simplePingResult(Sonar *sonar, SonarImage *image){
    publishImage(*image);
    publishCurrentConfig();
    std::cout << sonar << std::endl;
}


int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    
    std::shared_ptr<OculusDriverNode> node = std::make_shared<OculusDriverNode>("OculusDriverNode");

    // Initialize and connect to sonar
    node->sonar_ = std::make_shared<OculusSonar>();
    node->sonar_->findAndConnect();            // This starts the thread that will process new images
    if (node->sonar_->getState() != SonarState::Connected){
        exit(EXIT_FAILURE);
    }

    // Configure sonar
    node->sonar_->configure(2, 5.0, 80.0, 0.0, 0.0, false, 255, 0xff);
    node->sonar_->setPingRate(40);
    
    SonarCallback callback = [&node](Sonar *sonar, SonarImage *image) -> void {
        node->cb_simplePingResult(sonar, image);
    };

    // Register the callback
    node->sonar_->registerCallback(callback);

    while (rclcpp::ok()){
        rclcpp::spin_some(node);
        node->sonar_->fire();
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }


    rclcpp::shutdown();

    node->sonar_->disconnect();

    exit(EXIT_SUCCESS);
}