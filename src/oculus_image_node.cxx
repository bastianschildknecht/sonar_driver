#include <chrono>  
#include <memory> 

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

#include <sonar_driver/sonardevices/sonardevices.hxx>

using namespace std::chrono_literals;

class SonarImagePublisher : public rclcpp::Node
{
    public:
        SonarImagePublisher() : Node("oculus_sonar_image_node"), count_()
        {
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("oculus_sonar_image", 100);
        // fire 50 times a second to get updates about the image
        timer_ = this->create_wall_timer(
            20ms, std::bind(&SonarImagePublisher::image_to_msg, this));
        }
    sensor_msgs::msg::Image msg;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    uint64_t count_;

    private:
        SonarDevices::Sonar *sonar = new SonarDevices::OculusSonar();
        SonarDevices::SonarImage *img = new SonarDevices::SonarImage(); // 0, 1 or 2?

    void image_to_msg()//SonarDevices::SonarImage* img)
    // converting the sonar image to a ros image msg
    {
        // get the image form the sonar
        /*
        sonar->fire();
        img = sonar->getLastImage();
        */
       

        // time form sonar ping time:
        /*
        uint32_t nanoseconds = img->pingStartTime;
        msg.header.stamp.sec = nanoseconds / 1000000000;
        msg.header.stamp.nanosec = nanoseconds - (msg.header.stamp.sec * 1000000000);
        */

        // time from system:
        auto now = std::chrono::high_resolution_clock::now();  
        auto nanoseconds = now.time_since_epoch();
        int32_t seconds = (int32_t) (nanoseconds.count() / 1000000000);
        uint32_t nanos = (uint32_t) (nanoseconds.count() - (seconds * 1000000000)); 

        // set the image header
        msg.header.stamp.sec = seconds;
        msg.header.stamp.nanosec = nanos;
        msg.header.frame_id = this->count_;  // uint64_t  counter
        this->count_++;
        msg.height = img->imageHeight;
        msg.width = img->imageWidth;
        msg.encoding = "mono8";  // taken form sensor_msgs/include/image_enncodings.hpp
        msg.is_bigendian = true;  // no clue what it is, lets try sthg TODO
        msg.step = img->imageWidth;  // since data is uint8

        uint32_t size = msg.height * msg.width;
        memcpy(&msg.data, img->data, size);

        // might be better as Logging Data then just in cout.
        std::cout << "message from image" << std::endl;
        std::cout << "Header time: \t" << msg.header.stamp.sec << 
        " seconds and " << msg.header.stamp.nanosec << " nanos" << std::endl;
        std::cout << "Header id: \t" << msg.header.frame_id << std::endl;
        std::cout << "Image: \t\t" << msg.width << "x" << msg.height<< std::endl;

        this->publisher_->publish(msg);
        std::cout << "published!" << std::endl;
        std::cout << std::endl;
    }
};



int main(int argc, char * argv[])
{
    std::cout << "In main" << std::endl;
    rclcpp::init(argc, argv);
    std::cout << "initialized rcl" << std::endl;

    SonarImagePublisher pub;

    RCLCPP_INFO(pub.get_logger(), "created publisher successful");

    //connect the sonar
    // TODO: how do we want to give a specific address?
    /*
    pub.sonar->findAndConnect();
    */

    RCLCPP_INFO(pub.get_logger(), "starting publisher loop (spin)");
    auto sharedpub = std::make_shared<SonarImagePublisher>();
    rclcpp::spin(sharedpub);

    RCLCPP_INFO(pub.get_logger(), "finished publishing, shutting down");
    rclcpp::shutdown();

    return 0;

}