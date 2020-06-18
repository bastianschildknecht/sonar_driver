#include <chrono>  
#include <memory> 

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/fluid_pressure.hpp"

#include <sonar_driver/sonardevices/sonardevices.hxx>

using namespace std::chrono_literals;

class SonarPressurePublisher : public rclcpp::Node
{
    public:
        SonarPressurePublisher() : Node("oculus_sonar_pressure_node"), count_()
        {
        publisher_ = this->create_publisher<sensor_msgs::msg::FluidPressure>("oculus_sonar_pressure", 100);
        // fire twice a second to get updates about the pressure
        timer_ = this->create_wall_timer(
            500ms, std::bind(&SonarPressurePublisher::timer_callback, this));
        }
    sensor_msgs::msg::FluidPressure pressure_msg;
    rclcpp::Publisher<sensor_msgs::msg::FluidPressure>::SharedPtr publisher_;
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
        pressure_msg.header.stamp.sec = nanoseconds / 1000000000;
        pressure_msg.header.stamp.nanosec = nanoseconds - (pressure_msg.header.stamp.sec * 1000000000);
        */

        // time from system:
        auto now = std::chrono::high_resolution_clock::now();  
        auto nanoseconds = now.time_since_epoch();
        int32_t seconds = (int32_t) (nanoseconds.count() / 1000000000);
        uint32_t nanos = (uint32_t) (nanoseconds.count() - (seconds * 1000000000));
        this->pressure_msg.header.stamp.sec = seconds;
        this->pressure_msg.header.stamp.nanosec = nanos;
        this->pressure_msg.header.frame_id = this->count_;
        this->count_++;

        // take the pressure and convert it into a ros2 message
        /*
        pressure_msg.fluid_pressure = img->pressure;
        pressure_msg.variance = 0;  // 0 for unknown.
        */

        // take a test value for the ros2 message
        this->pressure_msg.fluid_pressure = 42.0;
        this->pressure_msg.variance = 0;  // 0 for unknown.

        // publish the msg
        this->publisher_->publish(pressure_msg);
        // to have the message visible change to RCLCPP_INFO 
        // or enable in cli with: ros2 run sonar_driver oculus_pressure_node --ros-args --log-level DEBUG
        RCLCPP_DEBUG(this->get_logger(), "sent message %d, pressure: %d", this->count_, int(this->pressure_msg.fluid_pressure));

    }
};



int main(int argc, char * argv[])
{
    std::cout << "In main" << std::endl;
    rclcpp::init(argc, argv);
    std::cout << "initialized rcl" << std::endl;

    SonarPressurePublisher pub;

    RCLCPP_INFO(pub.get_logger(), "created publisher successful");

    //connect the sonar
    // TODO: how do we want to give a specific address?
    /*
    pub.sonar->findAndConnect();
    */

    RCLCPP_INFO(pub.get_logger(), "starting publisher loop (spin)");
    auto sharedpub = std::make_shared<SonarPressurePublisher>();
    rclcpp::spin(sharedpub);

    RCLCPP_INFO(pub.get_logger(), "finished publishing, shutting down");
    rclcpp::shutdown();

    return 0;

}