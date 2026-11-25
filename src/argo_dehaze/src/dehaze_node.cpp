#include "argo_dehaze/adaptive_dcp.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>

class ArgoDehazeNode : public rclcpp::Node {
public:
  ArgoDehazeNode() : Node("argo_dehaze_node"), frame_count_(0), total_time_ms_(0.0) {
    // 1. Declare ROS 2 Parameters
    this->declare_parameter<std::string>("in_image_topic", "/camera/image_raw");
    this->declare_parameter<std::string>("out_image_topic", "/camera/image_dehazed");
    this->declare_parameter<int>("patch_size", 15);
    this->declare_parameter<double>("omega_min", 0.65);
    this->declare_parameter<double>("omega_max", 0.90);
    this->declare_parameter<double>("t0_base", 0.22);
    this->declare_parameter<int>("guided_radius", 16);
    this->declare_parameter<double>("guided_eps", 1e-3);
    this->declare_parameter<int>("guided_subsample", 4);
    this->declare_parameter<double>("target_luminance", 115.0);
    this->declare_parameter<bool>("enable_auto_white_balance", true);
    this->declare_parameter<double>("awb_strength", 0.75);
    this->declare_parameter<double>("neutral_atm_ratio", 0.50);
    this->declare_parameter<bool>("enable_adaptive_tone", true);
    this->declare_parameter<bool>("enable_clahe", true);
    this->declare_parameter<double>("clahe_clip_limit", 1.8);

    // 2. Read Parameters
    std::string in_topic = this->get_parameter("in_image_topic").as_string();
    std::string out_topic = this->get_parameter("out_image_topic").as_string();

    zenith::AdaptiveDCPParams params;
    params.patch_size = this->get_parameter("patch_size").as_int();
    params.omega_min = static_cast<float>(this->get_parameter("omega_min").as_double());
    params.omega_max = static_cast<float>(this->get_parameter("omega_max").as_double());
    params.t0_base = static_cast<float>(this->get_parameter("t0_base").as_double());
    params.guided_radius = this->get_parameter("guided_radius").as_int();
    params.guided_eps = static_cast<float>(this->get_parameter("guided_eps").as_double());
    params.guided_subsample = this->get_parameter("guided_subsample").as_int();
    params.target_luminance = static_cast<float>(this->get_parameter("target_luminance").as_double());
    params.enable_auto_white_balance = this->get_parameter("enable_auto_white_balance").as_bool();
    params.awb_strength = static_cast<float>(this->get_parameter("awb_strength").as_double());
    params.neutral_atm_ratio = static_cast<float>(this->get_parameter("neutral_atm_ratio").as_double());
    params.enable_adaptive_tone = this->get_parameter("enable_adaptive_tone").as_bool();
    params.enable_clahe = this->get_parameter("enable_clahe").as_bool();
    params.clahe_clip_limit = static_cast<float>(this->get_parameter("clahe_clip_limit").as_double());

    dehazer_.setParams(params);

    // 3. QoS Setup (Best effort or reliable compatible with camera driver)
    auto qos = rclcpp::SensorDataQoS();

    // 4. Subscriber & Publisher
    sub_image_ = this->create_subscription<sensor_msgs::msg::Image>(
        in_topic, qos, std::bind(&ArgoDehazeNode::imageCallback, this, std::placeholders::_1));

    pub_image_ = this->create_publisher<sensor_msgs::msg::Image>(out_topic, 10);

    RCLCPP_INFO(this->get_logger(), "==========================================");
    RCLCPP_INFO(this->get_logger(), " Argo Dehaze Node (Project Argo) Online");
    RCLCPP_INFO(this->get_logger(), "  Subscribed: %s", in_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "  Publishing: %s", out_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "  Fast Guided Filter Subsample: %dx", params.guided_subsample);
    RCLCPP_INFO(this->get_logger(), "  Auto White Balance: %s (%.2f)", params.enable_auto_white_balance ? "ON" : "OFF", params.awb_strength);
    RCLCPP_INFO(this->get_logger(), "==========================================");
  }

private:
  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg) {
    cv_bridge::CvImageConstPtr cv_ptr;
    try {
      if (msg->encoding == "bgr8" || msg->encoding == "rgb8") {
        cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
      } else if (msg->encoding == "mono8") {
        // FAST-LIVO2 can take mono or bgr, but dehaze is color-based
        cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
      } else {
        cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
      }
    } catch (cv_bridge::Exception& e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    if (cv_ptr->image.empty()) return;

    zenith::DehazeInfo info;
    cv::Mat dehazed_img = dehazer_.dehaze(cv_ptr->image, &info);

    // Prepare outgoing ROS message with exact timestamp preservation for FAST-LIVO2
    cv_bridge::CvImage out_cv_image;
    out_cv_image.header = msg->header; // Crucial: retain stamp & frame_id
    out_cv_image.encoding = "bgr8";
    out_cv_image.image = dehazed_img;

    pub_image_->publish(*out_cv_image.toImageMsg());

    // Performance profiling statistics
    frame_count_++;
    total_time_ms_ += info.elapsed_ms;
    if (frame_count_ % 50 == 0) {
      double avg_time = total_time_ms_ / 50.0;
      double fps = (avg_time > 0.0) ? (1000.0 / avg_time) : 0.0;
      RCLCPP_INFO(this->get_logger(), 
                  "[Stats] Frame #%ld | Avg: %.2f ms (%.1f FPS) | HDI: %.2f | Omega: %.2f | Gamma: %.2f",
                  frame_count_, avg_time, fps, info.haze_density, info.omega_used, info.applied_gamma);
      total_time_ms_ = 0.0;
    }
  }

  zenith::AdaptiveDCP dehazer_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_image_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_image_;

  size_t frame_count_;
  double total_time_ms_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ArgoDehazeNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
