// Publisher for the Sonar Data in ROS2
// TODO: better explanation
#include <sonar_driver/sonardevices/sonardevices.hxx>

#include <chrono>  // for timer
#include <memory>  // make_shared? i have no idea
#include <string>  // for std::string

#include "rclcpp/rclcpp.hpp"  // Node functionality
// Message types
#include "sensor_msgs/msg/image.hpp"

class SonarImagePublisher : public rclcpp::Node
{
    public:
        SonarImagePublisher() 
        : Node("sonar_image_publisher")
        {
        std::string TOPIC = "/sonar_message";
        //rclcpp::QoS BUFF_SIZE = rclcpp::QoS::;

        publisher = this->create_publisher<sensor_msgs::msg::Image>(TOPIC, 10); 
        RCLCPP_DEBUG(this->get_logger(), "published sonar msg");
        }

        sensor_msgs::msg::Image sonar_message;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher;

};

// sensor_msgs::msg::Image image_to_msg(SonarDevices::SonarImage img);
sensor_msgs::msg::Image image_to_msg(SonarDevices::SonarImage* img)
// converting the sonar image to a ros image msg
{
    // take system time 0 as starting point for timestamp
    auto now = std::chrono::high_resolution_clock::now();  // time in seconds
    // split into seconds and nano seconds
    auto nanoseconds = now.time_since_epoch();
    int32_t seconds = (int32_t) (nanoseconds.count() / 1000000000);
    // take away the 'seconds' part from the overall time and leave only nano part
    uint32_t nanos = (uint32_t) (nanoseconds.count() - (seconds * 1000000000));  

    sensor_msgs::msg::Image msg;
    // TODO: better use ping start time form sonar
    msg.header.stamp.sec = seconds;
    msg.header.stamp.nanosec = nanos;
    msg.header.frame_id = -1;  // uint64_t  counter, maybe lock, all callbacks from same thread
    msg.height = img->imageHeight;
    msg.width = img->imageWidth;
    msg.encoding = "mono8";  // taken form sensor_msgs/include/image_enncodings.hpp
    msg.is_bigendian = true;  // no clue what it is, lets try sthg TODO
    msg.step = img->imageWidth;  // since data is uint8
    
    uint32_t size = msg.height * msg.width;
    memcpy(&msg.data, img->data, size);

    return msg;
}

int main(int argc, char * argv[])
{

    // TODO: shoud argvs be address to sonar or sth like that, that the callback can get executed on it??

    SonarDevices::Sonar *sonar = new SonarDevices::OculusSonar();
    sonar->findAndConnect();

    SonarImagePublisher pub;

    while(rclcpp::ok)
    {
        // pub.sonar_image = SonarDevices::Sonar::registerCallback(image_to_msg); 

        // test image to see whether or not the message sending works
        SonarDevices::SonarImage test_img;
        uint8_t test_data[] = {42};
        test_img.imageType = SonarDevices::SonarImageObject;
        test_img.imageWidth = 1;
        test_img.imageHeight = 1;
        test_img.data = test_data;
        
        pub.sonar_message = image_to_msg(&test_img);

        pub.publisher->publish(pub.sonar_message);  //TODO move publish to callback function
    }

    RCLCPP_INFO(pub.get_logger(), "status is not ok: SIGINT has fired");  // SIG nal  INT errupt?
    rclcpp::shutdown();

    //sonar->disconnect();

    return 0;

}