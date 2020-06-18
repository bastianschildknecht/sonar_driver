#include <chrono>  
#include <memory> 

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/temperature.hpp"

#include <sonar_driver/sonardevices/sonardevices.hxx>

using namespace std::chrono_literals;

class SonarTemperaturePublisher : public rclcpp::Node
{
    public:
        SonarTemperaturePublisher() : Node("oculus_sonar_temperature_node"), count_()
        {
        publisher_ = this->create_publisher<sensor_msgs::msg::Temperature>("oculus_sonar_temperature", 100);
        // fire twice a second to get updates about the temperature
        timer_ = this->create_wall_timer(
            500ms, std::bind(&SonarTemperaturePublisher::timer_callback, this));
        }
    sensor_msgs::msg::Temperature temperature_msg;
    rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    size_t count_;

    private:
        SonarDevices::Sonar *sonar = new SonarDevices::OculusSonar();
        SonarDevices::SonarImage *img = new SonarDevices::OculusSonarImage(); // 1 or 2?

    void timer_callback()
    {
        //get image from sonar
        /* 
        sonar->fire();
        img = sonar->getLastImage();
        */

        // time form sonar ping time:
        /*
        uint32_t nanoseconds = img->pingStartTime;
        temperature_msg.header.stamp.sec = nanoseconds / 1000000000;
        temperature_msg.header.stamp.nanosec = nanoseconds - (temperature_msg.header.stamp.sec * 1000000000);
        */

        // time from system:
        auto now = std::chrono::high_resolution_clock::now();  
        auto nanoseconds = now.time_since_epoch();
        int32_t seconds = (int32_t) (nanoseconds.count() / 1000000000);
        uint32_t nanos = (uint32_t) (nanoseconds.count() - (seconds * 1000000000));
        this->temperature_msg.header.stamp.sec = seconds;
        this->temperature_msg.header.stamp.nanosec = nanos;
        this->temperature_msg.header.frame_id = this->count_;
        this->count_++;

        // take the pressure and convert it into a ros2 message
        /*
        temperature_msg.temperature = img->pressure;
        temperature_msg.variance = 0;  // 0 for unknown.
        */

        // take a test value for the ros2 message
        this->temperature_msg.temperature = 13.37;
        this->temperature_msg.variance = 0;  // 0 for unknown.

        // publish the msg
        this->publisher_->publish(temperature_msg);
        // to have the message visible change to RCLCPP_INFO 
        // or enable in cli with: ros2 run sonar_driver oculus_pressure_node --ros-args --log-level DEBUG
        RCLCPP_DEBUG(this->get_logger(), "sent message %d, pressure: %d", this->count_, int(this->temperature_msg.temperature));

    }
};



int main(int argc, char * argv[])
{
    std::cout << "In main" << std::endl;
    rclcpp::init(argc, argv);
    std::cout << "initialized rcl" << std::endl;

    SonarTemperaturePublisher pub;

    RCLCPP_INFO(pub.get_logger(), "created publisher successful");

    //connect the sonar
    // TODO: how do we want to give a specific address?
    /*
    pub.sonar->findAndConnect();
    */

    RCLCPP_INFO(pub.get_logger(), "starting publisher loop (spin)");
    auto sharedpub = std::make_shared<SonarTemperaturePublisher>();
    rclcpp::spin(sharedpub);

    RCLCPP_INFO(pub.get_logger(), "finished publishing, shutting down");
    rclcpp::shutdown();

    return 0;

}