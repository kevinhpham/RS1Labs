#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include <opencv2/imgcodecs.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

class ScanToImageNode : public rclcpp::Node
{
public:
    ScanToImageNode() : Node("scan_to_image_node"), angle_difference_(0.0), relative_orientaion_(0.0)
    {
        scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&ScanToImageNode::scanCallback, this, std::placeholders::_1));
        cmd_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        RCLCPP_INFO(this->get_logger(), "Scan to Image Node started.");
        std::string package_share_directory = ament_index_cpp::get_package_share_directory("labs");
        std::string map_file = package_share_directory + "/resources/turtle_map.pgm";
        map_image_ = loadMap(map_file);
    }

private:
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        cv::Rect cropArea(0,0,map_image_.cols,map_image_.rows);
        section_image_ = cropImage(map_image_,cropArea);
        //section_image_ = createEdgeImage(section_image_);
        laser_image_ = laserScanToMat(msg);
        laser_image_ = createEdgeImage(laser_image_);
        cv::rotate(laser_image_,laser_image_,cv::ROTATE_90_COUNTERCLOCKWISE);
        angle_difference_ = calculateYawChange(section_image_,laser_image_);
        std::cout << "Rotation Angle (degrees): " << angle_difference_ << std::endl;
        cv::imshow("Laser Scan Image", laser_image_);
        cv::imshow("Section Image", section_image_);
        cv::waitKey(1);
    }

    cv::Mat laserScanToMat(const sensor_msgs::msg::LaserScan::SharedPtr &scan)
    {

        // Purpose: Converts a LaserScan message into a binary image (cv::Mat), where each pixel represents the presence of an obstacle detected by the laser scanner.

        // Functionality:

        //      Create Image: Initializes a blank image of size 500x500 pixels.
        //      Map Polar to Cartesian: Iterates over the laser scan data, converting polar coordinates (distance and angle) to Cartesian coordinates (x, y) and sets the corresponding pixel in the image to white (255) if within range.
        // Parameters for the image
        int width = 228;
        int height = 210;
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

    double calculateYawChange(cv::Mat image1,cv::Mat image2)
    {
        // Detect and match features between the first and second images
        std::vector<cv::Point2f> srcPoints, dstPoints;
        detectAndMatchFeatures(image1, image2, srcPoints, dstPoints);

        if (srcPoints.size() < 3 || dstPoints.size() < 3) {
            RCLCPP_ERROR(this->get_logger(), "Not enough points for affine transformation.");
            return NULL;
        }

        try {
            cv::Mat transform_matrix = cv::estimateAffinePartial2D(srcPoints, dstPoints);
            if (transform_matrix.empty()) {
                RCLCPP_ERROR(this->get_logger(), "Transformation matrix estimation failed.");
            } else {
                // Extract the rotation angle from the transformation matrix
                angle_difference_ = atan2(transform_matrix.at<double>(1, 0), transform_matrix.at<double>(0, 0));
                angle_difference_ = angle_difference_ * 180.0 / CV_PI;
                RCLCPP_INFO(this->get_logger(), "Estimated yaw angle change: %f degrees", angle_difference_);
            }
        } catch (const cv::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error in estimateAffinePartial2D: %s", e.what());
        }
        return angle_difference_;

    }

    cv::Mat loadMap(const std::string &file_path)
    {
        // Load the PGM file as a grayscale image
        cv::Mat map_image = cv::imread(file_path, cv::IMREAD_GRAYSCALE);
        //Filter image white space
        cv::Mat kernel = (cv::Mat_<uchar>(3, 3) << 0, 0, 0,
                                        0, 1, 0,
                                        0, 0, 0)
                                        ;
        cv::Mat temp_img;
        cv::erode(map_image,temp_img,kernel);
        cv::cvtColor(temp_img, temp_img, cv::COLOR_GRAY2BGR);
        // Apply thresholding
        double thresh = 128;
        cv::threshold(temp_img, temp_img, thresh, 255, cv::THRESH_BINARY);
        cv::bitwise_not(temp_img, temp_img);
        // Resize the image
        cv::Size new_size(228, 220);
        cv::Mat resized_image;
        cv::resize(temp_img, resized_image, new_size);
        if (map_image.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load map image from %s", file_path.c_str());
        } else {
            RCLCPP_INFO(this->get_logger(), "Loaded map image from %s", file_path.c_str());
        }
        return resized_image;
    }

   cv::Mat cropImage(const cv::Mat &map_image, const cv::Rect &region)
    {
        // Ensure the crop region is within the bounds of the image
        if (region.x >= 0 && region.y >= 0 &&
            region.x + region.width <= map_image.cols &&
            region.y + region.height <= map_image.rows)
        {
            // Crop the region of interest from the map image
            cv::Mat cropped_image = map_image(region);
            RCLCPP_INFO(this->get_logger(), "Cropped map region at (%d, %d) with width %d and height %d",
                        region.x, region.y, region.width, region.height);
            return cropped_image;
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Crop region is out of image bounds");
            return cv::Mat();  // Return an empty matrix if out of bounds
        }
    }

    cv::Mat createEdgeImage(const cv::Mat &image)
    {
        // Create an edge image using Canny edge detection
        // Apply Gaussian Blur
        cv::Mat blurredImage, edges, dilatedEdges;
        cv::GaussianBlur(image, blurredImage, cv::Size(5, 5), 1.5);
        cv::Canny(blurredImage, edges, 50, 150);
        // Dilate the edges to merge close edges
        cv::dilate(edges, dilatedEdges, cv::Mat(), cv::Point(-1, -1), 1);
        RCLCPP_INFO(this->get_logger(), "Created edge image");
        return dilatedEdges;
    }

void detectAndMatchFeatures(const cv::Mat& img1, const cv::Mat& img2,
                            std::vector<cv::Point2f>& srcPoints, std::vector<cv::Point2f>& dstPoints) {
    cv::Ptr<cv::ORB> orb = cv::ORB::create();
    std::vector<cv::KeyPoint> keypoints1, keypoints2;
    cv::Mat descriptors1, descriptors2;

    // Detect and compute keypoints and descriptors
    orb->detectAndCompute(img1, cv::noArray(), keypoints1, descriptors1);
    orb->detectAndCompute(img2, cv::noArray(), keypoints2, descriptors2);

    // Match descriptors using BFMatcher with Hamming distance
    cv::BFMatcher matcher(cv::NORM_HAMMING);
    std::vector<cv::DMatch> matches;
    matcher.match(descriptors1, descriptors2, matches);

    // Sort matches based on distance (lower distance means better match)
    std::sort(matches.begin(), matches.end(), [](const cv::DMatch& a, const cv::DMatch& b) {
        return a.distance < b.distance;
    });

    // Determine the number of top matches to keep (15% of total matches)
    size_t numGoodMatches = static_cast<size_t>(matches.size() * 0.15);
    std::vector<cv::DMatch> goodMatches(matches.begin(), matches.begin() + numGoodMatches);

    // Extract the corresponding points from the good matches
    for (const auto& match : goodMatches) {
        srcPoints.push_back(keypoints1[match.queryIdx].pt);
        dstPoints.push_back(keypoints2[match.trainIdx].pt);
    }

    // Perform RANSAC to filter out outliers
    std::vector<uchar> inliersMask(srcPoints.size());
    cv::Mat homography = cv::findHomography(srcPoints, dstPoints, cv::RANSAC, 3, inliersMask);

    // Filter matches based on RANSAC inliers
    std::vector<cv::DMatch> ransacMatches;
    for (size_t i = 0; i < inliersMask.size(); i++) {
        if (inliersMask[i]) {
            ransacMatches.push_back(goodMatches[i]);
        }
    }

    // Draw the RANSAC-filtered matches
    cv::Mat img_matches;
    cv::drawMatches(img1, keypoints1, img2, keypoints2, ransacMatches, img_matches);

    // Display the matched features
    cv::imshow("Feature Matches", img_matches);
    cv::waitKey(1);

    // Update srcPoints and dstPoints with inliers only
    srcPoints.clear();
    dstPoints.clear();
    for (const auto& match : ransacMatches) {
        srcPoints.push_back(keypoints1[match.queryIdx].pt);
        dstPoints.push_back(keypoints2[match.trainIdx].pt);
    }
}


    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_publisher_;

    cv::Mat map_image_, section_image_, laser_image_;
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