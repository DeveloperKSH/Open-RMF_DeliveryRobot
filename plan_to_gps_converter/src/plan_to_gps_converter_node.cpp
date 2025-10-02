#include <chrono>
#include <memory>
#include <string>
#include <cmath>
#include <vector>
#include <optional>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <GeographicLib/UTMUPS.hpp>

using namespace std::chrono_literals;

class PlanToGPSConverter : public rclcpp::Node
{
public:
    PlanToGPSConverter()
        : Node("plan_to_gps_converter")
    {
        // Subscribe to the /plan topic
        plan_subscription_ = this->create_subscription<nav_msgs::msg::Path>(
            "/plan", 10, std::bind(&PlanToGPSConverter::plan_callback, this, std::placeholders::_1));

        // Subscribe to the /gps/fix topic
        gps_subscription_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "/gps/fix", 10, std::bind(&PlanToGPSConverter::gps_callback, this, std::placeholders::_1));

        // Publisher for the converted plan
        converted_plan_publisher_ = this->create_publisher<nav_msgs::msg::Path>("/converted_plan", 10);

        // Create a timer to publish the path every 100 milliseconds
        timer_ = this->create_wall_timer(
            500ms, std::bind(&PlanToGPSConverter::publish_converted_plan, this));
    }

private:
    void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
    {
        // Store the GPS data
        current_gps_ = *msg;
    }

    void plan_callback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        if (msg->poses.empty() || !current_gps_)
        {
            RCLCPP_WARN(this->get_logger(), "Plan is empty or GPS data is not yet available");
            return;
        }

        // Store the latest plan
        latest_plan_ = msg;
    }

    void publish_converted_plan()
    {
        if (!latest_plan_ || !current_gps_)
        {
            return;
        }

        const auto &msg = latest_plan_;

        // Assume the first point in the plan corresponds to the current GPS location
        const auto &first_pose = msg->poses.front().pose;

        // Convert the current GPS coordinate to UTM
        double first_pose_easting, first_pose_northing;
        int first_pose_zone;
        gps_to_utm(current_gps_->latitude, current_gps_->longitude, first_pose_easting, first_pose_northing, first_pose_zone);

        // Transform all the poses in the path relative to the first pose
        nav_msgs::msg::Path converted_path;
        converted_path.header = msg->header;

        for (const auto &pose_stamped : msg->poses)
        {
            const auto &pose = pose_stamped.pose;
            double dx = pose.position.x - first_pose.position.x;
            double dy = pose.position.y - first_pose.position.y;

            // Convert the relative pose to UTM coordinates
            double utm_easting = first_pose_easting + dx;
            double utm_northing = first_pose_northing + dy;
            double latitude, longitude;
            utm_to_gps(utm_easting, utm_northing, first_pose_zone, latitude, longitude);

            // Create a new pose with the converted GPS coordinates
            geometry_msgs::msg::PoseStamped gps_pose;
            gps_pose.header = pose_stamped.header;
            gps_pose.pose.position.x = longitude;  // Longitude
            gps_pose.pose.position.y = latitude;   // Latitude
            gps_pose.pose.position.z = current_gps_->altitude; // Altitude

            converted_path.poses.push_back(gps_pose);
        }

        // Publish the converted path
        converted_plan_publisher_->publish(converted_path);
    }

    // void gps_to_utm(double lat, double lon, double &easting, double &northing, int &zone)
    // {
    //     // Simple implementation for conversion, real implementation would need detailed formulas
    //     easting = lon * 111320; // Approximation: 1 degree longitude ~ 111,320 meters
    //     northing = lat * 110540; // Approximation: 1 degree latitude ~ 110,540 meters
    //     zone = static_cast<int>((lon + 180) / 6) + 1; // Simplified UTM zone calculation
    // }

    void gps_to_utm(double lat, double lon, double &easting, double &northing, int &zone) {
        bool northp;
        GeographicLib::UTMUPS::Forward(lat, lon, zone, northp, easting, northing);
    }

    // void utm_to_gps(double easting, double northing, int zone, double &lat, double &lon)
    // {
    //     // Simple implementation for conversion, real implementation would need detailed formulas
    //     lat = northing / 110540; // Approximation
    //     lon = easting / 111320; // Approximation
    // }
    
    void utm_to_gps(double easting, double northing, int zone, double &lat, double &lon) {
        bool northp = true; // Assuming we are in the northern hemisphere
        GeographicLib::UTMUPS::Reverse(zone, northp, easting, northing, lat, lon);
    }

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr plan_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_subscription_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr converted_plan_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::optional<sensor_msgs::msg::NavSatFix> current_gps_;
    nav_msgs::msg::Path::SharedPtr latest_plan_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PlanToGPSConverter>());
    rclcpp::shutdown();
    return 0;
}
