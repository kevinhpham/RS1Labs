#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <opencv2/opencv.hpp>
#include <vector>

class ScanToImageNode : public rclcpp::Node
{
public:
    ScanToImageNode() : Node("scan_to_image_node"), angle_difference_(0.0), relative_orientaion_(0.0)
    {
        scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&ScanToImageNode::scanCallback, this, std::placeholders::_1));
        cmd_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        RCLCPP_INFO(this->get_logger(), "Scan to Image Node started.");
    }

private:
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // Purpose: This function is a callback that gets triggered whenever a new LaserScan message is received from the /scan topic.

        // Functionality:

        //     Convert LaserScan to Image: The laserScanToMat function is called to convert the LaserScan data into an image (a cv::Mat).
        //     Handle First Image: If it's the first time this function is being called, it captures and displays the first image.
        //     Handle Second Image: If it's the second time this function is called, it captures and displays the second image, then calculates the change in orientation (yaw).
        //     Update and Rotate: For subsequent calls, it updates the images, calculates the yaw change, and logs the relative orientation.
        if(!first_image_captured_)
        {
            first_image_ = laserScanToMat(msg);
            first_image_captured_ = true;
        }

        else if(!second_image_captured_)
        {
            second_image_ = laserScanToMat(msg);
            second_image_captured_ = true;
            calculateYawChange();
            relative_orientaion_ += angle_difference_;
        }

        else
        {
            first_image_ = second_image_;
            second_image_ = laserScanToMat(msg);
            second_image_captured_ = true;
            calculateYawChange();
            relative_orientaion_ += angle_difference_;
        }
        std::cout << "Rotation Angle (degrees): " << angle_difference_ << std::endl;
    }

    cv::Mat laserScanToMat(const sensor_msgs::msg::LaserScan::SharedPtr &scan)
    {

        // Purpose: Converts a LaserScan message into a binary image (cv::Mat), where each pixel represents the presence of an obstacle detected by the laser scanner.

        // Functionality:

        //      Create Image: Initializes a blank image of size 500x500 pixels.
        //      Map Polar to Cartesian: Iterates over the laser scan data, converting polar coordinates (distance and angle) to Cartesian coordinates (x, y) and sets the corresponding pixel in the image to white (255) if within range.
        // Parameters for the image
        int width = 500;
        int height = 500;
        float max_range = scan->range_max;
        float min_range = scan->range_min;

        // Create a blank image (single channel, 8-bit image)
        cv::Mat image = cv::Mat::zeros(height, width, CV_8UC1);

        // Iterate through the laser scan ranges
        for (int i = 0; i < scan->ranges.size(); ++i)
        {
            float range = scan->ranges[i];
            if (range >= min_range && range <= max_range)
            {
                // Calculate the angle of the current laser scan beam
                float angle = scan->angle_min + i * scan->angle_increment;

                // Convert the polar coordinates (range, angle) to Cartesian coordinates (x, y)
                int x = static_cast<int>((range * std::cos(angle)) * (width / 2.0) / max_range + width / 2.0);
                int y = static_cast<int>((range * std::sin(angle)) * (height / 2.0) / max_range + height / 2.0);

                // Ensure x and y are within image bounds
                if (x >= 0 && x < width && y >= 0 && y < height)
                {
                    // Set the corresponding pixel to white (255)
                    image.at<uint8_t>(y, x) = 255;
                }
            }
        }

        return image;
    }

    void calculateYawChange()
    {
        // Purpose: Estimates the change in orientation (yaw angle) of the robot by comparing two images.

        // Functionality:

        //     Feature Matching: Uses feature detection and matching to find corresponding points between the two images.
        std::vector<cv::Point2f> srcPoints, dstPoints;
        detectAndMatchFeatures(first_image_, second_image_, srcPoints, dstPoints);
        //     Estimate Transformation: Computes an affine transformation matrix to determine the rotation between the two images.
        cv::Mat transformationMatrix = cv::estimateAffine2D(srcPoints, dstPoints);

        // Extract rotation components
        double cos_theta = transformationMatrix.at<double>(0, 0);
        double sin_theta = transformationMatrix.at<double>(1, 0);

        // Calculate the rotation angle in radians
        double theta = std::atan2(sin_theta, cos_theta);

        // Convert the angle to degrees
        angle_difference_ = theta * (180.0 / CV_PI);

        // Output the rotation angle
       // std::cout << "Rotation Angle (degrees): " << angle_difference_ << std::endl;
        //     Calculate Angle: Extracts the rotation angle from the transformation matrix and converts it to degrees.
    }

    void detectAndMatchFeatures(const cv::Mat &img1, const cv::Mat &img2,
                                std::vector<cv::Point2f> &srcPoints, std::vector<cv::Point2f> &dstPoints)
    {
        cv::Ptr<cv::ORB> orb = cv::ORB::create();
        std::vector<cv::KeyPoint> keypoints1, keypoints2;
        cv::Mat descriptors1, descriptors2;

        orb->detectAndCompute(img1, cv::noArray(), keypoints1, descriptors1);
        orb->detectAndCompute(img2, cv::noArray(), keypoints2, descriptors2);

        cv::BFMatcher matcher(cv::NORM_HAMMING);
        std::vector<cv::DMatch> matches;
        matcher.match(descriptors1, descriptors2, matches);

        // Sort matches based on distance (lower distance means better match)
        std::sort(matches.begin(), matches.end(), [](const cv::DMatch &a, const cv::DMatch &b)
                  { return a.distance < b.distance; });

        // Determine the number of top matches to keep (30% of total matches)
        size_t numGoodMatches = static_cast<size_t>(matches.size() * 0.15);

        // Keep only the best matches (top 30%)
        std::vector<cv::DMatch> goodMatches(matches.begin(), matches.begin() + numGoodMatches);

        for (const auto &match : matches)
        {
            srcPoints.push_back(keypoints1[match.queryIdx].pt);
            dstPoints.push_back(keypoints2[match.trainIdx].pt);
        }
        // Draw the matches
        cv::Mat img_matches;
        cv::drawMatches(img1, keypoints1, img2, keypoints2, goodMatches, img_matches);

        // Display the matched features
        cv::imshow("Feature Matches", img_matches);
        cv::waitKey(1);
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_publisher_;

    cv::Mat first_image_, second_image_;
    bool first_image_captured_ = false;
    bool second_image_captured_ = false;

    double angle_difference_;
    double relative_orientaion_ = 0.0;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ScanToImageNode>());
    rclcpp::shutdown();
    return 0;
}