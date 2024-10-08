#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class ImageProcessor : public rclcpp::Node
{
public:
    ImageProcessor() : Node("image_processor")
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw", 10, std::bind(&ImageProcessor::image_callback, this, std::placeholders::_1));  //Subscribe to the image topic turtle bot is publishing to
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/image_processed", 10);   //Create publisher to publish to topic where processed images will be
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        // Convert ROS Image message to OpenCV image
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        // Get the dimensions of the image
        int height = cv_ptr->image.rows;
        int width = cv_ptr->image.cols;

        // Calculate the center of the image
        cv::Point center(width / 2, height / 2);

        // Draw a circle at the center of the image
        cv::circle(cv_ptr->image, center, 50, CV_RGB(0, 255, 0), 2);

        // Convert OpenCV image back to ROS Image message
        sensor_msgs::msg::Image::SharedPtr processed_msg = cv_ptr->toImageMsg();

        // Publish the processed image
        publisher_->publish(*processed_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImageProcessor>());
    rclcpp::shutdown();
    return 0;
}
