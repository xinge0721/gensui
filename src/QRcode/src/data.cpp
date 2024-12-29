#include "ros/ros.h"
#include "std_msgs/String.h"
#include <string>
#include <iostream>

// 回调函数，处理接收到的数据
void qrCodeCallback(const std_msgs::String::ConstPtr& msg)
{
    std::string qr_data = msg->data;  // 获取数据
    // ROS_INFO("Received QR Code Data: %s", qr_data.c_str());

    // 解析数据（假设格式为 "speed = 50km/h, angle = 60deg"）
    std::string speed, angle;

    // 查找并提取 speed 和 angle
    size_t speed_pos = qr_data.find("speed = ");
    size_t angle_pos = qr_data.find("angle = ");
    
    if (speed_pos != std::string::npos && angle_pos != std::string::npos)
    {
        // 提取 speed 和 angle 的值
        speed = qr_data.substr(speed_pos + 8, qr_data.find(",", speed_pos) - (speed_pos + 8));  // 获取 speed 部分
        angle = qr_data.substr(angle_pos + 8);  // 获取 angle 部分

        // 去掉多余的引号
        speed.erase(std::remove(speed.begin(), speed.end(), '"'), speed.end());
        angle.erase(std::remove(angle.begin(), angle.end(), '"'), angle.end());

        // 转换为整数
        int speed_int = std::stoi(speed);  // 将 speed 转换为 int
        int angle_int = std::stoi(angle);  // 将 angle 转换为 int

        // 打印解析出的 speed 和 angle
        ROS_INFO("Parsed Data - Speed: %d, Angle: %d", speed_int, angle_int);
    }
    else
    {
        ROS_WARN("Failed to parse speed and angle data.");
    }
}

int main(int argc, char **argv)
{
    // 初始化 ROS 节点
    ros::init(argc, argv, "qrcode_subscriber");
    ros::NodeHandle nh;

    // 订阅 qrcode_data 话题，消息类型是 std_msgs/String
    ros::Subscriber sub = nh.subscribe("qrcode_data", 1000, qrCodeCallback);

    // 循环等待消息
    ros::spin();

    return 0;
}
