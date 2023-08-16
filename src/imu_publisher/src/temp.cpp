
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include <boost/asio.hpp>
#include <iostream>
#include <vector>
#include <string>
#include <memory>
#include <boost/algorithm/string.hpp>

using boost::asio::serial_port_base;
using boost::asio::serial_port;
using boost::asio::io_service;

class ImuNode : public rclcpp::Node
{
public:
    ImuNode() : Node("imu_publisher"), port(io)
    {
        imu_data_publisher = this->create_publisher<sensor_msgs::msg::Imu>("imu", 1000);
        
        port.open("/dev/ttyUSB0");
        port.set_option(serial_port_base::baud_rate(921600));
        
        read_timer = this->create_wall_timer(
            std::chrono::milliseconds(1), // Read at 1000Hz
            std::bind(&ImuNode::read_imu_data, this));
        
        publish_timer = this->create_wall_timer(
            std::chrono::milliseconds(5), // Publish at 200Hz
            std::bind(&ImuNode::publish_downsampled_data, this));
    }

private:
    void read_imu_data()
    {
        // Sample serial reading using Boost Asio (assuming using read_until method or similar)
        boost::asio::streambuf response;
        std::istream response_stream(&response);
        boost::asio::read_until(port, response, '\n');
        std::string line;
        std::getline(response_stream, line);

        // Tokenizing and data processing similar to before
        std::vector<std::string> data;
        boost::split(data, line, boost::is_any_of(","));
        
        if (data.size() == 17 && data[0] == "100-0")
        {
            auto imu_msg = std::make_shared<sensor_msgs::msg::Imu>();
            imu_msg->orientation.x = std::stof(data[1]);
            // ... set other fields ...

            imu_buffer.push_back(imu_msg);
            if (imu_buffer.size() > 5) 
            {
                imu_buffer.erase(imu_buffer.begin());
            }
        }
    }

    void publish_downsampled_data()
    {
        if (imu_buffer.size() == 5)
        {
            auto avg_msg = std::make_shared<sensor_msgs::msg::Imu>();
            avg_msg->header.stamp = this->now();
            avg_msg->header.frame_id = "imu_frame";

            for (const auto& msg : imu_buffer)
            {
                avg_msg->orientation.x += msg->orientation.x;
                avg_msg->orientation.x += msg->orientation.x;
                avg_msg->orientation.y += msg->orientation.x;
                avg_msg->orientation.z += msg->orientation.x;
                avg_msg->orientation.w += msg->orientation.x;
                avg_msg->angular_velocity.x += msg->orientation.x;
                avg_msg->angular_velocity.y += msg->orientation.x;
                avg_msg->angular_velocity.z += msg->orientation.x;
                avg_msg->linear_acceleration.x += msg->orientation.x;
                avg_msg->linear_acceleration.y += msg->orientation.x;
                avg_msg->linear_acceleration.z += msg->orientation.x;
            }
            
            avg_msg->orientation.x /= 5;
            avg_msg->orientation.x /= 5;
            avg_msg->orientation.y /= 5;
            avg_msg->orientation.z /= 5;
            avg_msg->orientation.w /= 5;
            avg_msg->angular_velocity.x /= 5;
            avg_msg->angular_velocity.y /= 5;
            avg_msg->angular_velocity.z /= 5;
            avg_msg->linear_acceleration.x /= 5;
            avg_msg->linear_acceleration.y /= 5;
            avg_msg->linear_acceleration.z /= 5;

            imu_data_publisher->publish(*avg_msg);
        }
    }
    
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_data_publisher;
    rclcpp::TimerBase::SharedPtr read_timer, publish_timer;
    io_service io;
    serial_port port;
    std::vector<std::shared_ptr<sensor_msgs::msg::Imu>> imu_buffer;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImuNode>());
    rclcpp::shutdown();
    return 0;
}
