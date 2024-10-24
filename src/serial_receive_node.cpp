#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

#define DEVICE_NAME "/dev/ttyUSB0"

rclcpp::Clock ros_clock(RCL_ROS_TIME);

int openSerial(const char *device_name){
    int  fd1 = open(device_name, O_RDWR | O_NOCTTY | O_NONBLOCK);
    
    fcntl(fd1, F_SETFL, 0);
    struct termios conf_tio;
    tcgetattr(fd1, &conf_tio);
    
    speed_t BAUDRATE = B115200;
    cfsetispeed(&conf_tio, BAUDRATE);
    cfsetospeed(&conf_tio, BAUDRATE);

    conf_tio.c_lflag &= ~(ECHO | ICANON);

    conf_tio.c_cc[VMIN] = 0;
    conf_tio.c_cc[VTIME] = 0;

    tcsetattr(fd1, TCSANOW, &conf_tio);
    return fd1;
}

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("serial_receive_node");
    auto serial_pub = node->create_publisher<nav_msgs::msg::Odometry>("serial_odom", 1);

    char device_name[] = DEVICE_NAME;
    char fd1 = openSerial(device_name);
    if(fd1 < 0){
        RCLCPP_ERROR(node->get_logger(), "Serial Failed: could not open %s", device_name);
        printf("Serial Failed \n");
        rclcpp::shutdown();
    }

    rclcpp::WallRate loop_rate(10);
    while(rclcpp::ok()){
        char buf[256] = {0};
        std::string data;
        int flag = 0;

        while(true){
            int recv_data = read(fd1, buf, sizeof(buf));
            if(recv_data > 0){
                flag = 1;
                data += std::string(buf, recv_data);

                float x,y,z,qx,qy,qz,qw;
                int res = sscanf(data.c_str(),"%f%f%f%f%f%f%f",&x,&y,&z,&qx,&qy,&qz,&qw);
                if(res >= 7){
                    auto serial_odom = std::make_unique<nav_msgs::msg::Odometry>();
                    serial_odom->header.stamp = ros_clock.now();
                    serial_odom->header.frame_id = "map";
                    serial_odom->child_frame_id = "odom";
                    serial_odom->pose.pose.position.x = x;
                    serial_odom->pose.pose.position.y = y;
                    serial_odom->pose.pose.position.z = z;
                    serial_odom->pose.pose.orientation.x = qx;
                    serial_odom->pose.pose.orientation.y = qy;
                    serial_odom->pose.pose.orientation.z = qz;
                    serial_odom->pose.pose.orientation.w = qw;
                    std::array<double, 36> pose_covariance = {
                        0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
                        0.0, 0.1, 0.0, 0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0, 0.0, 0.0, 0.1
                    };
                    serial_odom->pose.covariance = pose_covariance;
                    serial_odom->twist.twist.linear.x = 0.0;
                    serial_odom->twist.twist.linear.y = 0.0;
                    serial_odom->twist.twist.linear.z = 0.0;
                    serial_odom->twist.twist.linear.x = 0.0;
                    serial_odom->twist.twist.angular.x = 0.0;
                    serial_odom->twist.twist.angular.y = 0.0;
                    serial_odom->twist.twist.angular.z = 0.0;
                    std::array<double, 36> twist_covariance = {-1}; //non-valid
                    serial_odom->twist.covariance = twist_covariance;
                    serial_pub->publish(std::move(serial_odom));

                    std::cout << "recv: " << x << ", " << y << ", " << z << std::endl; 
                }
            }else{
                flag = 0;
            }
            
            if(flag == 0) break;
        }
        loop_rate.sleep();
    }
    rclcpp::shutdown();
    return 0;
}