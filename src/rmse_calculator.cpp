#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <std_msgs/msg/float64.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>
#include <fstream>
#include <iomanip>

class ErrorCalculator : public rclcpp::Node
{
public:
    ErrorCalculator() : Node("error_calculator_node"), count_(0), sum_squared_error_(0.0)
    {
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&ErrorCalculator::odom_callback, this, std::placeholders::_1));

        amcl_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/amcl_pose", 10, std::bind(&ErrorCalculator::amcl_pose_callback, this, std::placeholders::_1));

        rmse_pub_ = this->create_publisher<std_msgs::msg::Float64>("rmse", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&ErrorCalculator::calculate_rmse, this));

        // Open the file and write the header
        output_file_.open("rmse_data.csv");
        output_file_ << "Time,Odometry X,Odometry Y,Odometry Z,AMCL X,AMCL Y,AMCL Z,RMSE\n";
    }

    ~ErrorCalculator()
    {
        if (output_file_.is_open()) {
            output_file_.close();
        }
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        current_odom_ = *msg;
        odom_received_ = true;
    }

    void amcl_pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        current_amcl_pose_ = *msg;
        amcl_pose_received_ = true;
    }

    void calculate_rmse()
    {
        if (odom_received_ && amcl_pose_received_)
        {
            double dx = current_odom_.pose.pose.position.x - current_amcl_pose_.pose.pose.position.x;
            double dy = current_odom_.pose.pose.position.y - current_amcl_pose_.pose.pose.position.y;
            double dz = current_odom_.pose.pose.position.z - current_amcl_pose_.pose.pose.position.z;

            double squared_error = dx * dx + dy * dy + dz * dz;
            sum_squared_error_ += squared_error;
            count_++;

            double rmse = std::sqrt(sum_squared_error_ / count_);

            RCLCPP_INFO(this->get_logger(), "RMSE: %f", rmse);

            auto rmse_msg = std_msgs::msg::Float64();
            rmse_msg.data = rmse;
            rmse_pub_->publish(rmse_msg);

            // Get the current time
            double current_time = this->get_clock()->now().seconds();

            // Write data to the file
            if (output_file_.is_open())
            {
                output_file_ << std::fixed << std::setprecision(3)
                             << current_time << ","
                             << current_odom_.pose.pose.position.x << ","
                             << current_odom_.pose.pose.position.y << ","
                             << current_odom_.pose.pose.position.z << ","
                             << current_amcl_pose_.pose.pose.position.x << ","
                             << current_amcl_pose_.pose.pose.position.y << ","
                             << current_amcl_pose_.pose.pose.position.z << ","
                             << rmse << "\n";
            }
        }
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_pose_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr rmse_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    nav_msgs::msg::Odometry current_odom_;
    geometry_msgs::msg::PoseWithCovarianceStamped current_amcl_pose_;
    bool odom_received_ = false;
    bool amcl_pose_received_ = false;

    double sum_squared_error_;
    int count_;

    std::ofstream output_file_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ErrorCalculator>());
    rclcpp::shutdown();
    return 0;
}
