#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <vector>
#include <iostream>
#include <math.h>

class laserScanFilter: public rclcpp::Node
{
    public:
        laserScanFilter() : Node("laser_ScanFilter")
    {
        sub1_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&laserScanFilter::laser_callback, this, std::placeholders::_1));  //Subscribe to the image topic turtle bot is publishing to
        pub1_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/laser_filtered", 10);   //Create publisher to publish to topic where processed images will be
    }


    private:
    void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        int interval = 2;
        auto filteredLaser = *msg;
        float maxAngle = 2*M_PI;
        float minAngle = 0;
        filteredLaser.angle_min = minAngle;
        filteredLaser.angle_max = maxAngle;
        filteredLaser.angle_increment = msg->angle_increment*interval;
        std::vector<float> filteredRanges;
        filteredRanges.reserve(msg->ranges.size());
        // laser readings go from 0 to 2PI
        // std::cout<<msg->angle_min<<std::endl;
        // std::cout<<msg->angle_increment<<std::endl;
        // std::cout<<msg->angle_max<<std::endl;
        for(int count = 0; count<msg->ranges.size();count++){
            float currentAngle = msg->angle_min + count*msg->angle_increment;
            std::cout<< currentAngle <<" "<<std::ends; 
            //std::cout<< reading <<" "<<std::ends; //Causes terminal to lag a little
            if(currentAngle <= maxAngle && currentAngle >= minAngle && count%interval == 0){
               filteredRanges.push_back(msg->ranges.at(count));
            }
        }
        filteredLaser.ranges = filteredRanges;
        pub1_->publish(filteredLaser);
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub1_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub1_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<laserScanFilter>());
    rclcpp::shutdown();
    return 0;
}
