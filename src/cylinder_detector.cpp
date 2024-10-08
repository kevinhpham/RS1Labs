#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <math.h>
#include <cmath>
#include <vector>
#include <opencv2/opencv.hpp>

class CylinderDetector : public rclcpp::Node
{
public:
    CylinderDetector() : Node("cylinder_detector"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
    {
        laser_scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&CylinderDetector::scanCallback, this, std::placeholders::_1));
        // odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        //     "/odom", 10, std::bind(&CylinderDetector::odomCallback, this, std::placeholders::_1));
        map_subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", 10, std::bind(&CylinderDetector::map_callback, this, std::placeholders::_1));
        // Publisher to publish the modified map
        map_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("modified_map", 10);
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&CylinderDetector::get_transform, this));
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    geometry_msgs::msg::Pose robot_pose_;
    geometry_msgs::msg::TransformStamped transform_stamped_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscription_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_publisher_;
    nav_msgs::msg::OccupancyGrid modified_map_;
    rclcpp::TimerBase::SharedPtr timer_;

    double circle_x_, circle_y_, circle_diameter_;

    void get_transform()
    {
        // Specify the source and target frames
        std::string source_frame = "base_scan"; // The robot's frame (could also be base_footprint)
        std::string target_frame = "map";       // The map frame

        try
        {
            // Look up the transformation
            transform_stamped_ = tf_buffer_.lookupTransform(
                target_frame,
                source_frame,
                tf2::TimePointZero // Get the latest available transform
            );

            // // Log the translation and rotation
            // RCLCPP_INFO(this->get_logger(), "Translation: (%.2f, %.2f, %.2f)",
            //             transform_stamped_.transform.translation.x,
            //             transform_stamped_.transform.translation.y,
            //             transform_stamped_.transform.translation.z);

            // RCLCPP_INFO(this->get_logger(), "Rotation: (%.2f, %.2f, %.2f, %.2f)",
            //             transform_stamped_.transform.rotation.x,
            //             transform_stamped_.transform.rotation.y,
            //             transform_stamped_.transform.rotation.z,
            //             transform_stamped_.transform.rotation.w);
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "Could not transform '%s' to '%s': %s",
                        target_frame.c_str(), source_frame.c_str(), ex.what());
        }
    }

    void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        // Create a copy of the original map to modify
        if (modified_map_.data.size() != msg->data.size())
        {
            modified_map_ = *msg;
        }
        modified_map_.info = msg->info;
        modified_map_.header = msg->header;
        for (int i = 0; i < msg->data.size(); i++)
        {
            if (modified_map_.data.at(i) >=100)
            {}
            else{
                modified_map_.data.at(i) = msg->data.at(i);
            }
        }

        // Publish the modified map
        map_publisher_->publish(modified_map_);
        RCLCPP_INFO(this->get_logger(), "Published modified occupancy map with a circle.");
    }

    void draw_circle(std::vector<int8_t> &data, int width, int center_x, int center_y, int radius)
    {
        int x = radius;
        int y = 0;
        int decision = 1 - radius;
        set_circle_points(data, width, center_x, center_y, x, y);
        while (y < x)
        {
            y++;
            if (decision < 0)
            {
                decision += 2 * y + 1;
            }
            else
            {
                x--;
                decision += 2 * (y - x) + 1;
                
            }
            set_circle_points(data, width, center_x, center_y, y, x);
        }
    }

    void set_circle_points(std::vector<int8_t> &data, int width, int center_x, int center_y, int x, int y)
    {
        std::vector<std::pair<int, int>> points = {
            {center_x + x, center_y + y}, {center_x - x, center_y + y}, {center_x + x, center_y - y}, {center_x - x, center_y - y}, {center_x + y, center_y + x}, {center_x - y, center_y + x}, {center_x + y, center_y - x}, {center_x - y, center_y - x}};

        for (const auto &[px, py] : points)
        {
            if (px >= 0 && px < width && py >= 0 && py < width)
            {
                data[py * width + px] = 100; // Mark as occupied (100)
                                             // RCLCPP_INFO(this->get_logger(), "Marking occupied cell at (%d, %d)", px, py);
            }
        }
    }

    // // Odometry callback to store robot's position and orientation
    // void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    // {
    //     robot_pose_ = msg->pose.pose; // Store the robot's pose from odometry
    // }

    // Laser scan callback
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
    {
        // Convert laser scan data to an image
        cv::Mat scan_image = laserScanToImage(scan_msg);
        // scan_image = createEdgeImage(scan_image);

        // Detect circles (potential cylinders) in the image
        std::vector<cv::Vec3f> circles = detectCirclesInImage(scan_image);

        // If circles are detected, draw them on the image
        if (!circles.empty())
        {
            for (size_t i = 0; i < circles.size(); ++i)
            {
                float x = circles[i][0];
                float y = circles[i][1];
                float radius = circles[i][2];
                double roboX, roboY;
                convertImagePtToCordinate(x, y, scan_msg->range_max, scan_image.rows, roboX, roboY);

                RCLCPP_INFO(this->get_logger(), "robo: x = %.2f, y = %.2f", roboX, roboY);

                geometry_msgs::msg::Point pt = transformToGlobalFrame(roboX, roboY, transform_stamped_.transform);

                RCLCPP_INFO(this->get_logger(), "Cylinder detected at: x = %.2f, y = %.2f, radius = %.2f", pt.x, pt.y, radius);

                // Draw the circle in the image (circle center and outline)
                cv::circle(scan_image, cv::Point(x, y), static_cast<int>(radius), cv::Scalar(255), 2); // Circle outline
                cv::circle(scan_image, cv::Point(x, y), 2, cv::Scalar(255), 3);                        // Circle center point

                int marked_count = std::count(modified_map_.data.begin(), modified_map_.data.end(), 100);
                // Convert the circle parameters to map coordinates (in cells)
                int width_in_cells = modified_map_.info.width;
                int height_in_cells = modified_map_.info.height;
                double resolution = modified_map_.info.resolution;

                circle_diameter_ = .30; // Increase the diameter to 0.3 meter (for visibility)

                // Convert the center to grid cells

                double center_x_world = -modified_map_.info.origin.position.x  * resolution;
                double center_y_world = -modified_map_.info.origin.position.y  * resolution;

                int x_in_cells = std::round((-modified_map_.info.origin.position.x +pt.x)/resolution);
                int y_in_cells = std::round((-modified_map_.info.origin.position.y +pt.y)/ resolution);

                int radius_in_cells = std::round((circle_diameter_ / 2.0 )/ resolution);

                // Log circle and map parameters for debugging
                RCLCPP_INFO(this->get_logger(), "map origin: (%f, %f) meters", modified_map_.info.origin.position.x,modified_map_.info.origin.position.y );
                RCLCPP_INFO(this->get_logger(), "Map Resolution: %f meters/cell", resolution);
                RCLCPP_INFO(this->get_logger(), "Map width: %d cells", width_in_cells);
                RCLCPP_INFO(this->get_logger(), "Map height: %d cells", height_in_cells);
                RCLCPP_INFO(this->get_logger(), "Circle center: (%f, %f) meters", pt.x, pt.y);
                RCLCPP_INFO(this->get_logger(), "Circle center in grid: (%d, %d)", x_in_cells, y_in_cells);
                RCLCPP_INFO(this->get_logger(), "Circle radius in cells: %d", radius_in_cells);

                // Draw the circle in the map's data
                draw_circle(modified_map_.data, width_in_cells, x_in_cells, y_in_cells, radius_in_cells);

                // Verify map data after modification
                marked_count = std::count(modified_map_.data.begin(), modified_map_.data.end(), 100);
                // RCLCPP_INFO(this->get_logger(), "Number of cells marked as occupied after modification: %d", marked_count);
            }
        }

        // Display the result with detected circles
        displayImage(scan_image);
    }

    // Function to convert quaternion to yaw (theta)
    double quaternionToYaw(const geometry_msgs::msg::Quaternion &q)
    {
        // Extract yaw from quaternion
        double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
        double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
        return std::atan2(siny_cosp, cosy_cosp); // Return yaw in radians
    }

    // Helper function to convert image coordinates to real-world coordinates
    void convertImagePtToCordinate(int img_x, int img_y, double max_range, int image_size, double &real_x, double &real_y)
    {
        // Similar to the previous logic where we map pixel coordinates back to real-world coordinates
        double scale = (2.0 * max_range) / image_size;
        real_x = (img_x - image_size / 2) * scale;
        real_y = (img_y - image_size / 2) * scale;
    }

    // Function to transform from robot frame to map frame using TF
    geometry_msgs::msg::Point transformToGlobalFrame(double x_local, double y_local, geometry_msgs::msg::Pose pose)
    {
        geometry_msgs::msg::Point pt;
        double theta = quaternionToYaw(pose.orientation);
        pt.x = pose.position.x + (x_local * std::cos(theta) - y_local * std::sin(theta));
        pt.y = pose.position.y + (x_local * std::sin(theta) + y_local * std::cos(theta));

        return pt;
    }

    // Function to transform from robot frame to map frame using TF
    geometry_msgs::msg::Point transformToGlobalFrame(double x_local, double y_local, geometry_msgs::msg::Transform pose)
    {
        geometry_msgs::msg::Point pt;
        double theta = quaternionToYaw(pose.rotation);
        pt.x = pose.translation.x + (x_local * std::cos(theta) - y_local * std::sin(theta));
        pt.y = pose.translation.y + (x_local * std::sin(theta) + y_local * std::cos(theta));
        // RCLCPP_INFO(this->get_logger(), "pose : x = %.2f, y = %.2f", pose.translation.x, pose.translation.y);
        return pt;
    }

    // Convert laser scan to a 2D image
    cv::Mat laserScanToImage(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
    {
        int image_size = 500;                     // Size of the image (width and height in pixels)
        double max_range = scan_msg->range_max;   // Max range of the laser scan
        double min_distance_between_points = 0.1; // 5 cm in meters

        // Create a blank image
        cv::Mat image(image_size, image_size, CV_8UC1, cv::Scalar(0)); // Initialize with black (0)

        double angle = scan_msg->angle_min;                 // Start angle of the laser scan
        double angle_increment = scan_msg->angle_increment; // Angle increment between measurements

        cv::Point prev_point(-1, -1); // Previous point, initialized as invalid

        // Loop over each laser scan range
        for (size_t i = 0; i < scan_msg->ranges.size(); ++i)
        {
            double range = scan_msg->ranges[i];

            // Ignore invalid ranges
            if (range < scan_msg->range_min || range > max_range)
            {
                angle += angle_increment;
                continue;
            }

            // Convert polar coordinates (range, angle) to Cartesian coordinates (x, y)
            double x = range * cos(angle);
            double y = range * sin(angle);

            // Map the Cartesian coordinates to the image space (image_size x image_size)
            int img_x = static_cast<int>((x / max_range) * image_size / 2 + image_size / 2);
            int img_y = static_cast<int>((y / max_range) * image_size / 2 + image_size / 2);

            cv::Point current_point(img_x, img_y); // Current laser scan point in image coordinates

            // Mark the current point on the image
            if (img_x >= 0 && img_x < image_size && img_y >= 0 && img_y < image_size)
            {
                image.at<uchar>(img_y, img_x) = 255; // Set the current point to white
            }

            // Draw a line between adjacent points if they are less than 5 cm apart
            if (prev_point.x != -1 && prev_point.y != -1)
            {
                double distance_between_points = sqrt(pow(x - (prev_point.x - image_size / 2) * max_range / (image_size / 2), 2) +
                                                      pow(y - (prev_point.y - image_size / 2) * max_range / (image_size / 2), 2));

                if (distance_between_points < min_distance_between_points)
                {
                    cv::line(image, prev_point, current_point, cv::Scalar(255), 1); // Draw a white line with thickness 1
                }
            }

            // Set the current point as the previous point for the next iteration
            prev_point = current_point;

            // Increment the angle for the next laser scan point
            angle += angle_increment;
        }
        cv::imshow("Laser Scan", image);
        cv::waitKey(1); // Adjust the wait time as necessary

        return image;
    }

    cv::Mat createEdgeImage(const cv::Mat &image)
    {
        // Create an edge image using Canny edge detection
        // Apply Gaussian Blur
        cv::Mat blurredImage, edges, dilated;
        cv::GaussianBlur(image, blurredImage, cv::Size(5, 5), 1.5);
        // cv::dilate(blurredImage, dilated, cv::Mat(), cv::Point(-1, -1), 1);
        cv::Canny(blurredImage, edges, 50, 150);
        // Dilate the edges to merge close edges

        RCLCPP_INFO(this->get_logger(), "Created edge image");
        return edges;
    }

    // Detect circles in the image using Hough Circle Transform
    std::vector<cv::Vec3f> detectCirclesInImage(const cv::Mat &image)
    {
        std::vector<cv::Vec3f> circles;

        // Apply Hough Circle Transform to detect circles in the image
        cv::HoughCircles(
            image,              // Input image (grayscale)
            circles,            // Output vector of detected circles
            cv::HOUGH_GRADIENT, // Detection method (use HOUGH_GRADIENT)
            1,                  // dp (accumulator resolution relative to image resolution)
            image.rows / 30,    // Minimum distance between circle centers
            50,                 // param1: Higher threshold for Canny edge detection
            10,                 // param2: Accumulator threshold for circle detection
            7,                  // Minimum radius (set to 0 if you don't want to specify)
            11                  // Maximum radius (set to 0 if you don't want to specify)
        );
        // Return detected circles
        return circles;
    }

    // Display image with circles
    void displayImage(const cv::Mat &image)
    {
        // Create a named window to display the image
        cv::namedWindow("Laser Scan with Detected Cylinders", cv::WINDOW_AUTOSIZE);

        // Display the image in the window
        cv::imshow("Laser Scan with Detected Cylinders", image);

        // Wait for a key press to close the window
        cv::waitKey(1); // Adjust the wait time as necessary
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CylinderDetector>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
