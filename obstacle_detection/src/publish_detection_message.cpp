#include <chrono>
#include <memory>
#include <string>
#include <cmath>
#include <vector>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using namespace std::chrono_literals;

class ObstacleDetectionNode : public rclcpp::Node
{
public:
  ObstacleDetectionNode()
  : Node("obstacle_detection_node"), 
    obstacle_detected_(false), 
    obstacle_threshold_(2.5), 
    obstacle_error_prevent_(0.2), 
    alert_announced_(false), 
    tracking_rotation_(false),
    sum_rotation_angle_(0.0),
    initial_orientation_set_(false),
    initial_yaw_(0.0),
    obstacle_clear_counter_(0),
    alert_timer_active_(false)
  {
    rclcpp::QoS qos_settings(10);
    qos_settings.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    qos_settings.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

    detect_publisher_ = this->create_publisher<std_msgs::msg::String>("obstacle_topic", qos_settings);
    rotation_publisher_ = this->create_publisher<std_msgs::msg::Float64>("rotation_angle_topic", qos_settings);
    correction_publisher_ = this->create_publisher<std_msgs::msg::Float64>("correction_angle_topic", qos_settings);
    spin_publisher_ = this->create_publisher<std_msgs::msg::String>("spin_topic", qos_settings);
    clear_publisher_ = this->create_publisher<std_msgs::msg::String>("alert_topic", qos_settings);

    timer_ = this->create_wall_timer(
      500ms, std::bind(&ObstacleDetectionNode::publish_message, this));

    scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", qos_settings, std::bind(&ObstacleDetectionNode::obstacleCallback, this, std::placeholders::_1));
    
    odometry_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom", qos_settings, std::bind(&ObstacleDetectionNode::odometryCallback, this, std::placeholders::_1));

    alert_subscriber_ = this->create_subscription<std_msgs::msg::String>(
      "alert_topic", qos_settings, std::bind(&ObstacleDetectionNode::alertCallback, this, std::placeholders::_1));
    
    alert_timer_ = this->create_wall_timer(
      3s, std::bind(&ObstacleDetectionNode::check_for_clear_obstacle, this));
    alert_timer_->cancel();
  }

private:
  // 500ms 주기로 토픽을 발행
  void publish_message()
  {
    auto detect_msg = std_msgs::msg::String();

    if (obstacle_detected_) {
      detect_msg.data = "Detected";
    }
    else if (!obstacle_detected_) {
      detect_msg.data = "Undetected"; 
    }
    // 장애물 감지 유무를 'obstacle_topic'으로 발행
    detect_publisher_->publish(detect_msg);
    RCLCPP_INFO(this->get_logger(), "Obstacle: '%s'", detect_msg.data.c_str());

    // 로봇이 회전했을 경우, 로봇의 회전 각도를 'rotation_angle_topic'으로 발행
    if (rotation_completed_) {
      auto rotation_msg = std_msgs::msg::Float64();
      rotation_msg.data = rotation_angle_;
      rotation_publisher_->publish(rotation_msg);
      RCLCPP_INFO(this->get_logger(), "Rotation angle: '%f'", rotation_msg.data);
      rotation_completed_ = false;
    }

    // 장애물 감지 경고가 울렸을 경우, 회전 추적 함수를 실행
    if (alert_announced_) {
      alert_announced_ = false;
      start_rotation_tracking();
    }

    if (tracking_rotation_) {
      update_rotation_tracking();
    }
  }

  // 'scan' 토픽으로부터 scan 데이터를 수신받아 장애물 감지 유무를 판별
  void obstacleCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    obstacle_detected_ = false;

    // 사용자 설정 각도 및 설정 범위 내에서 장애물이 감지되었는지 확인
    // 설정 각도: 0 ~ (1/12)*PI && (23/12)*PI ~ 2*PI
    // 설정 범위: 0.2m(obstacle_error_prevent_) ~ 2.5m(obstacle_threshold_)
    double angle = msg->angle_min;
    for (size_t i = 0; i < msg->ranges.size(); ++i) {
      if ((angle >= 0 && angle <= M_PI / 12) || (angle >= 23 * M_PI / 12 && angle <= 2 * M_PI)) {
        if (msg->ranges[i] >= obstacle_error_prevent_ && msg->ranges[i] <= msg->range_max && msg->ranges[i] < obstacle_threshold_) {
          obstacle_detected_ = true;
          break;
        }
      }
      angle += msg->angle_increment;
    }

    // 장애물 감지 경보가 울렸을 경우, 로봇이 장애물을 실제로 감지하고 있는지 확인
    if (alert_timer_active_ && !obstacle_detected_) {
      obstacle_clear_counter_++;
    } else {
      obstacle_clear_counter_ = 0;
    }
  }

  // 'odom' 토픽으로부터 odom 데이터를 수신받아 로봇의 회전 각도를 계산
  void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    last_odometry_pose_ = msg->pose.pose;

    tf2::Quaternion quat;
    tf2::fromMsg(last_odometry_pose_.orientation, quat);
    double roll, pitch, yaw;
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    // 로봇의 회전 각도 추적 플래그가 활성화된 경우, 회전 각도를 계산
    if (tracking_rotation_) {
      double current_rotation_angle = yaw - initial_yaw_;

      if (current_rotation_angle > M_PI) {
        current_rotation_angle -= 2 * M_PI;
      } else if (current_rotation_angle < -M_PI) {
        current_rotation_angle += 2 * M_PI;
      }

      sum_rotation_angle_ += current_rotation_angle;
      initial_yaw_ = yaw;
      rotation_completed_ = true;
    }
  }

  // 'alert_topic' 토픽으로부터 장애물 감지 경보 및 로봇 정지 메시지를 수신받음
  void alertCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    const std::string alert_msg = "Obstacle Detected";
    const std::string stop_msg = "Robot Stopped";

    // 장애물 감지 경보일 경우
    if (msg->data == alert_msg) {
      alert_announced_ = true;
    }
    // 로봇 정지 메시지일 경우
    else if (msg->data == stop_msg) {
      RCLCPP_INFO(this->get_logger(), "Received 'Robot Stopped' alert");
      alert_timer_->reset();
      obstacle_clear_counter_ = 0;
      alert_timer_active_ = true;
    }
  }

  // 장애물 감지 경보가 울린 경우, 회전 추적 세팅
  void start_rotation_tracking()
  {
    RCLCPP_INFO(this->get_logger(), "Start Rotation Tracking");
    tracking_rotation_ = true;
    sum_rotation_angle_ = 0.0;
    initial_yaw_ = get_current_yaw();
    tracking_start_time_ = std::chrono::steady_clock::now();
  }

  // 회전 추적에 필요한 로봇의 현재 상태 확인
  double get_current_yaw()
  {
    tf2::Quaternion quat;
    tf2::fromMsg(last_odometry_pose_.orientation, quat);
    double roll, pitch, yaw;
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    return yaw;
  }

  // 회전 추적 함수
  void update_rotation_tracking()
  {
    auto elapsed_time = std::chrono::steady_clock::now() - tracking_start_time_;
    // 장애물 감지 경보가 울린 경우, 회전 추적을 2초동안 실행 
    if (elapsed_time >= std::chrono::seconds(2)) {
      RCLCPP_INFO(this->get_logger(), "Update Rotation Tracking");
      auto spin_msg = std_msgs::msg::String();
      std::string execute_msg;
      tracking_rotation_ = false;
      // 장애물 감지 경보: 울림 o + 현재 장애물 감지 유무: 감지 x 
      // --> 로봇이 회전을 실행함 --> 다시 장애물을 마주하도록 재회전을 실행하도록 토픽을 발행 
      if (!obstacle_detected_ && sum_rotation_angle_ != 0.0) {
        auto correction_msg = std_msgs::msg::Float64();
        execute_msg = "Execute";
        correction_msg.data = (-sum_rotation_angle_);
        correction_publisher_->publish(correction_msg);
        RCLCPP_INFO(this->get_logger(), "Correction rotation angle sum: '%f'", correction_msg.data);
      }
      // 장애물 감지 경보: 울림 o + 현재 장애물 감지 유무: 감지 o 
      // --> 로봇이 회전을 실행하지 않음 --> 재회전이 필요하지 않다는 토픽을 발행
      else {
        execute_msg = "Unexecute";
      }
      spin_msg.data = execute_msg;
      spin_publisher_->publish(spin_msg);
      RCLCPP_INFO(this->get_logger(), "Send Spin Message");
    }
  }

  // 로봇이 정지했을 경우, 경로 상의 장애물이 사라졌는지 확인
  void check_for_clear_obstacle()
  {
    // 3초(500ms 주기로 6번)동안 장애물이 감지되지 않으면 장애물이 사라진 것으로 판단 후 토픽을 발행
    if (obstacle_clear_counter_ >= 6) {
      auto clear_message = std_msgs::msg::String();
      clear_message.data = "Obstacle Clear";
      clear_publisher_->publish(clear_message);
      RCLCPP_INFO(this->get_logger(), "Published 'Obstacle Clear'");
      alert_timer_->cancel(); // Stop the timer
      alert_timer_active_ = false;
    }
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr detect_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr rotation_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr correction_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr spin_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr clear_publisher_;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscriber_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr alert_subscriber_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr alert_timer_;

  bool obstacle_detected_;
  bool rotation_completed_;
  bool initial_orientation_set_;
  bool alert_announced_;
  bool tracking_rotation_;
  bool alert_timer_active_;

  double obstacle_threshold_;
  double obstacle_error_prevent_;
  double initial_yaw_;     
  double rotation_angle_;
  double sum_rotation_angle_;
  int obstacle_clear_counter_;

  geometry_msgs::msg::Pose last_odometry_pose_;
  std::chrono::time_point<std::chrono::steady_clock> tracking_start_time_; 
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObstacleDetectionNode>());
  rclcpp::shutdown();
  return 0;
}
