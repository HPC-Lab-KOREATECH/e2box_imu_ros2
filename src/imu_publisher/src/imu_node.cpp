
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include <boost/asio.hpp>
#include <iostream>
#include <vector>
#include <string>
#include <memory>
#include <boost/algorithm/string.hpp>
#include <thread>
#include <mutex>
using boost::asio::serial_port_base;
using boost::asio::serial_port;
using boost::asio::io_service;




class ImuNode : public rclcpp::Node
{
public:
    ImuNode() : Node("imu_publisher"), port(io), exit_thread_(false)
    {
        imu_data_publisher = this->create_publisher<sensor_msgs::msg::Imu>("imu", 1000);
        
        port.open("/dev/ttyUSB0");
        port.set_option(serial_port_base::baud_rate(921600));
         
        publish_timer = this->create_wall_timer(
            std::chrono::milliseconds(1), // Publish at 200Hz
            std::bind(&ImuNode::publish_downsampled_data, this));

        worker_thread_ = std::thread(&ImuNode::read_imu_data, this);
    }

    ~ImuNode()
    {
        exit_thread_ = true;  // 쓰레드에 종료 명령을 보냄
        if (worker_thread_.joinable())
            worker_thread_.join();  // 쓰레드가 종료될 때까지 기다림
    }

private:
    void read_imu_data()
    {
        // Sample serial reading using Boost Asio (assuming using read_until method or similar)
        boost::asio::streambuf response;
        std::istream response_stream(&response);

        while (rclcpp::ok()){
            boost::asio::read_until(port, response, '\n');
            std::string line;
            std::getline(response_stream, line);
            std::lock_guard<std::mutex> lock(mtx);  // 뮤텍스를 통한 동기화
            boost::split(data, line, boost::is_any_of(","));
        }
    }

    void publish_downsampled_data()
    {
        auto avg_msg = std::make_shared<sensor_msgs::msg::Imu>();
        avg_msg->header.stamp = this->now();
        avg_msg->header.frame_id = "imu_frame";
       
       std::lock_guard<std::mutex> lock(mtx);  // 뮤텍스를 통한 동기화
        std::vector<std::string> temp = data;
        avg_msg->orientation.x =  std::stof(temp[3]);
        avg_msg->orientation.y =  std::stof(temp[2]);
        avg_msg->orientation.z =  std::stof(temp[1]);
        avg_msg->orientation.w =  std::stof(temp[4]);
        avg_msg->angular_velocity.x =  std::stof(temp[4]);
        avg_msg->angular_velocity.y =  std::stof(temp[5]);
        avg_msg->angular_velocity.z =  std::stof(temp[6]);

        avg_msg->linear_acceleration.x =  std::stof(temp[8])- 2*(avg_msg->orientation.x*avg_msg->orientation.z-avg_msg->orientation.w*avg_msg->orientation.y);
        avg_msg->linear_acceleration.y =  std::stof(temp[9])- 2*(avg_msg->orientation.w*avg_msg->orientation.x+avg_msg->orientation.y*avg_msg->orientation.z);
        avg_msg->linear_acceleration.z =  std::stof(temp[10])- (avg_msg->orientation.w*avg_msg->orientation.w - avg_msg->orientation.x*avg_msg->orientation.x - avg_msg->orientation.y*avg_msg->orientation.y + avg_msg->orientation.z+avg_msg->orientation.z);

        imu_data_publisher->publish(*avg_msg);
    }
    
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_data_publisher;
    rclcpp::TimerBase::SharedPtr read_timer, publish_timer;
    io_service io;
    serial_port port;
    std::vector<std::shared_ptr<sensor_msgs::msg::Imu>> imu_buffer;
    std::thread worker_thread_;
    bool exit_thread_;
    std::vector<std::string> data = {"0","0","0","0","0","0","0","0","0","0","0","0","0"};
    std::mutex mtx;  // 변수에 접근할 때 사용할 뮤텍스
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImuNode>());
    rclcpp::shutdown();
    return 0;
}
