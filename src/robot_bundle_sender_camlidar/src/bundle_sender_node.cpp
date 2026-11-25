// Robot-side camera+LiDAR bundle sender: synchronised (image, point cloud, odometry) -> per-bundle package
// (per-frame points, one H.264 video segment, per-frame stamp/pose) sent when a bundle closes.
// No RADIO / Utonia / semantic inference here: the server unpacks the package and runs the semantic submapping.

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <unordered_set>
#include <vector>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include "robot_bundle_sender_camlidar/camlidar_builder.hpp"
#include "robot_bundle_sender_camlidar/bundle_sender.hpp"
#include "robot_bundle_sender_camlidar/odom_streamer.hpp"

namespace robot_bundle_sender_camlidar {

using sensor_msgs::msg::Image;
using sensor_msgs::msg::PointCloud2;
using nav_msgs::msg::Odometry;
using Policy = message_filters::sync_policies::ApproximateTime<Image, PointCloud2, Odometry>;

class CamLidarBundleSenderNode : public rclcpp::Node {
 public:
  CamLidarBundleSenderNode() : Node("robot_bundle_sender_camlidar") {
    const auto robot_id = declare_parameter<std::string>("robot_id", "robot");
    BuilderParams bp;
    bp.session = declare_parameter<std::string>("session", "");
    if (bp.session.empty()) bp.session = robot_id;
    bp.bundle_distance_m = declare_parameter<double>("bundle_distance", 40.0);
    bp.point_scale_m = declare_parameter<double>("point_scale_m", 0.01);
    bp.min_range_m = declare_parameter<double>("min_range_m", 0.0);
    bp.max_range_m = declare_parameter<double>("max_range_m", 0.0);
    bp.compress_points = declare_parameter<bool>("compress_points", true);
    bp.point_codec = declare_parameter<std::string>("point_codec", "zlib");
    bp.draco_speed = static_cast<int>(declare_parameter<int>("draco_speed", 9));
    bp.codec_threads = static_cast<int>(declare_parameter<int>("codec_threads", 2));
    frame_voxel_m_ = declare_parameter<double>("frame_voxel_size", 0.1);
    send_every_n_frames_ = std::max<int64_t>(1, declare_parameter<int>("send_every_n_frames", 1));
    flush_on_shutdown_ = declare_parameter<bool>("flush_on_shutdown", true);
    min_flush_distance_m_ = declare_parameter<double>("min_flush_distance", 5.0);

    auto* info = &bp.session_info;
    info->set_robot_id(robot_id);
    info->set_fx(declare_parameter<double>("camera_fx", 0.0));
    info->set_fy(declare_parameter<double>("camera_fy", 0.0));
    info->set_cx(declare_parameter<double>("camera_cx", 0.0));
    info->set_cy(declare_parameter<double>("camera_cy", 0.0));
    for (double d : declare_parameter<std::vector<double>>("camera_distortion", {0.0, 0.0, 0.0, 0.0})) info->add_distortion(d);
    info->set_distortion_model(declare_parameter<std::string>("distortion_model", "plumb_bob"));
    info->set_image_width(static_cast<uint32_t>(declare_parameter<int>("image_width", 0)));
    info->set_image_height(static_cast<uint32_t>(declare_parameter<int>("image_height", 0)));
    const std::vector<double> eye = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
    for (double v : declare_parameter<std::vector<double>>("T_base_cam", eye)) info->add_t_base_cam(v);
    for (double v : declare_parameter<std::vector<double>>("T_base_lidar", eye)) info->add_t_base_lidar(v);
    if (info->t_base_cam_size() != 16 || info->t_base_lidar_size() != 16)
      throw std::invalid_argument("T_base_cam and T_base_lidar need 16 values (row-major 4x4)");
    if (info->fx() <= 0.0 || info->fy() <= 0.0) throw std::invalid_argument("camera_fx/camera_fy must be set");

    VideoParams vp;
    vp.ffmpeg_path = declare_parameter<std::string>("ffmpeg_path", "ffmpeg");
    vp.codec = declare_parameter<std::string>("video_codec", "libx264");
    vp.bitrate = declare_parameter<std::string>("video_bitrate", "1M");
    vp.preset = declare_parameter<std::string>("video_preset", "veryfast");
    vp.gop = static_cast<int>(declare_parameter<int>("video_gop", 20));
    vp.threads = static_cast<int>(declare_parameter<int>("video_threads", 1));
    vp.nominal_fps = declare_parameter<double>("video_nominal_fps", 10.0);
    vp.tmp_dir = declare_parameter<std::string>("video_tmp_dir", "/tmp");

    SenderParams sp;
    sp.server_address = declare_parameter<std::string>("grpc_server", "127.0.0.1:50053");
    sp.queue_size = static_cast<size_t>(declare_parameter<int>("send_queue_size", 8));
    sp.retry_interval_s = declare_parameter<double>("retry_interval_s", 2.0);
    sp.rpc_timeout_s = declare_parameter<double>("rpc_timeout_s", 60.0);
    sp.backup_dir = declare_parameter<std::string>("backup_dir", "");
    sp.hello_timeout_s = declare_parameter<double>("hello_timeout_s", 3.0);

    builder_ = std::make_unique<CamLidarBuilder>(bp, vp);
    sender_ = std::make_unique<BundleSender>(
        sp, [this](const std::string& m) { RCLCPP_INFO(get_logger(), "%s", m.c_str()); },
        [this](const std::string& m) { RCLCPP_WARN(get_logger(), "%s", m.c_str()); });
    {
      proto::Hello h;   // startup connection check (logged on both sides before the first bundle)
      h.set_session(bp.session);
      h.set_robot_id(robot_id);
      h.set_bundle_distance_m(bp.bundle_distance_m);
      h.set_image_width(bp.session_info.image_width());
      h.set_image_height(bp.session_info.image_height());
      sender_->hello(std::move(h));
    }

    const auto image_topic = declare_parameter<std::string>("image_topic", "/camera/image_raw");
    const auto cloud_topic = declare_parameter<std::string>("pointcloud_topic", "/cloud_registered_local");
    const auto pose_topic = declare_parameter<std::string>("pose_topic", "/aft_mapped_to_init");
    const int depth = static_cast<int>(declare_parameter<int>("queue_size", 10));
    const double slop = declare_parameter<double>("slop", 0.1);
    const auto qos = rclcpp::QoS(depth).get_rmw_qos_profile();
    image_sub_.subscribe(this, image_topic, qos);
    cloud_sub_.subscribe(this, cloud_topic, qos);
    pose_sub_.subscribe(this, pose_topic, qos);
    sync_ = std::make_unique<message_filters::Synchronizer<Policy>>(Policy(depth), image_sub_, cloud_sub_, pose_sub_);
    sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(slop));
    sync_->registerCallback(&CamLidarBundleSenderNode::on_frame, this);

    // Live odometry stream: every message of the synchronised odometry topic, on its own connection (odom_live.jsonl on the
    // server). Defaults apply when a config does not mention these keys.
    if (declare_parameter<bool>("odom_stream", true)) {
      OdomStreamParams op;
      op.server_address = sp.server_address;
      op.session = bp.session;
      op.robot_id = robot_id;
      op.max_rate_hz = declare_parameter<double>("odom_stream_hz", 0.0);
      op.queue_size = static_cast<size_t>(declare_parameter<int>("odom_stream_queue", 200));
      op.retry_interval_s = sp.retry_interval_s;
      odom_ = std::make_unique<OdomStreamer>(
          op, [this](const std::string& m) { RCLCPP_INFO(get_logger(), "%s", m.c_str()); },
          [this](const std::string& m) { RCLCPP_WARN(get_logger(), "%s", m.c_str()); });
      pose_sub_.registerCallback([this](const Odometry::ConstSharedPtr& m) {
        const auto& p = m->pose.pose;
        odom_->push(stamp_ns(m->header.stamp), p.position.x, p.position.y, p.position.z, p.orientation.x, p.orientation.y,
                    p.orientation.z, p.orientation.w);
      });
      RCLCPP_INFO(get_logger(), "odometry stream: %s -> %s (%s)", pose_topic.c_str(), sp.server_address.c_str(),
                  op.max_rate_hz > 0.0 ? ("at most " + std::to_string(op.max_rate_hz) + " Hz").c_str() : "every message");
    }
    RCLCPP_INFO(get_logger(), "session '%s': %s + %s + %s -> bundle every %.1f m (frame voxel %.2f m, every %ld frame(s), points %s) -> %s",
                bp.session.c_str(), image_topic.c_str(), cloud_topic.c_str(), pose_topic.c_str(), bp.bundle_distance_m,
                frame_voxel_m_, static_cast<long>(send_every_n_frames_),
                (bp.point_codec == "draco" ? "draco x" + std::to_string(bp.codec_threads) + " threads" : bp.point_codec).c_str(),
                sp.server_address.c_str());
  }

  ~CamLidarBundleSenderNode() override { shutdown(); }

  void shutdown() {
    if (done_) return;
    done_ = true;
    if (flush_on_shutdown_ && builder_->frames() > 0 && builder_->distance() >= min_flush_distance_m_) {
      std::string err;
      if (auto s = builder_->flush(true, &err)) {
        if (!err.empty()) RCLCPP_WARN(get_logger(), "last bundle: %s", err.c_str());
        RCLCPP_INFO(get_logger(), "flushing the last bundle %u (%d frames, %.1f m)", s->bundle_index(), s->frames_size(),
                    s->distance_traveled_m());
        if (sendable(*s)) sender_->enqueue(std::move(*s));
      }
    }
    if (!sender_->drain(30.0)) RCLCPP_WARN(get_logger(), "some bundles were not sent before shutdown");
  }

 private:
  static int64_t stamp_ns(const builtin_interfaces::msg::Time& t) {
    return static_cast<int64_t>(t.sec) * 1000000000LL + t.nanosec;
  }

  // A bundle whose video segment failed cannot be unpacked on the server (frames without images): drop it with an error
  // instead of sending it.
  bool sendable(const proto::CamLidarBundle& s) {
    if (s.frames_size() > 0 && s.video().empty()) {
      RCLCPP_ERROR(get_logger(), "bundle %u has no video (encoder failed): %d frames dropped", s.bundle_index(), s.frames_size());
      return false;
    }
    return true;
  }

  // BGR bytes of the image (bgr8 as is; rgb8 / mono8 converted). False for other encodings.
  bool to_bgr(const Image& im, std::vector<uint8_t>* out) {
    const size_t w = im.width, h = im.height;
    out->resize(w * h * 3);
    if (im.encoding == "bgr8" || im.encoding == "rgb8") {
      const bool swap = im.encoding == "rgb8";
      for (size_t y = 0; y < h; ++y) {
        const uint8_t* row = im.data.data() + y * im.step;
        uint8_t* dst = out->data() + y * w * 3;
        for (size_t x = 0; x < w; ++x) {
          dst[3 * x] = row[3 * x + (swap ? 2 : 0)];
          dst[3 * x + 1] = row[3 * x + 1];
          dst[3 * x + 2] = row[3 * x + (swap ? 0 : 2)];
        }
      }
      return true;
    }
    if (im.encoding == "mono8") {
      for (size_t y = 0; y < h; ++y) {
        const uint8_t* row = im.data.data() + y * im.step;
        uint8_t* dst = out->data() + y * w * 3;
        for (size_t x = 0; x < w; ++x) dst[3 * x] = dst[3 * x + 1] = dst[3 * x + 2] = row[x];
      }
      return true;
    }
    return false;
  }

  // x/y/z of the cloud, one point per frame_voxel_m_ voxel (the first one, in message order) like mapping_data_sender.
  void extract_points(const PointCloud2& c) {
    const size_t n = static_cast<size_t>(c.width) * c.height;
    xyz_.clear();
    xyz_.reserve(n * 3);
    seen_.clear();
    const double inv = frame_voxel_m_ > 0.0 ? 1.0 / frame_voxel_m_ : 0.0;
    sensor_msgs::PointCloud2ConstIterator<float> x(c, "x"), y(c, "y"), z(c, "z");
    for (size_t i = 0; i < n; ++i, ++x, ++y, ++z) {
      if (!std::isfinite(*x) || !std::isfinite(*y) || !std::isfinite(*z)) continue;
      if (inv > 0.0) {
        const auto k = (static_cast<uint64_t>(static_cast<int64_t>(std::floor(*x * inv)) + (1 << 20)) << 42) ^
                       (static_cast<uint64_t>(static_cast<int64_t>(std::floor(*y * inv)) + (1 << 20)) << 21) ^
                       static_cast<uint64_t>(static_cast<int64_t>(std::floor(*z * inv)) + (1 << 20));
        if (!seen_.insert(k).second) continue;
      }
      xyz_.push_back(*x);
      xyz_.push_back(*y);
      xyz_.push_back(*z);
    }
  }

  void on_frame(const Image::ConstSharedPtr& im, const PointCloud2::ConstSharedPtr& cloud, const Odometry::ConstSharedPtr& odom) {
    if (done_) return;
    if ((sync_count_++) % send_every_n_frames_ != 0) return;
    if (!to_bgr(*im, &bgr_)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "unsupported image encoding '%s'", im->encoding.c_str());
      return;
    }
    extract_points(*cloud);
    const auto& p = odom->pose.pose;
    const Eigen::Vector3d position(p.position.x, p.position.y, p.position.z);
    const Eigen::Quaterniond orientation(p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z);  // sent as received
    std::string err;
    auto s = builder_->add_frame(stamp_ns(im->header.stamp), position, orientation, xyz_.data(), xyz_.size() / 3, 3, bgr_.data(),
                                 static_cast<int>(im->width), static_cast<int>(im->height), &err);
    if (!err.empty()) RCLCPP_WARN(get_logger(), "%s", err.c_str());
    if (s) {
      RCLCPP_INFO(get_logger(), "bundle %u closed: %d frames, %.1f m, points %zu KiB, video %zu KiB", s->bundle_index(),
                  s->frames_size(), s->distance_traveled_m(), point_payload_bytes(*s) / 1024, s->video().size() / 1024);
      if (sendable(*s)) sender_->enqueue(std::move(*s));
    }
  }

  std::unique_ptr<CamLidarBuilder> builder_;
  std::unique_ptr<BundleSender> sender_;
  std::unique_ptr<OdomStreamer> odom_;
  message_filters::Subscriber<Image> image_sub_;
  message_filters::Subscriber<PointCloud2> cloud_sub_;
  message_filters::Subscriber<Odometry> pose_sub_;
  std::unique_ptr<message_filters::Synchronizer<Policy>> sync_;
  std::vector<uint8_t> bgr_;
  std::vector<float> xyz_;
  std::unordered_set<uint64_t> seen_;
  double frame_voxel_m_ = 0.1;
  int64_t send_every_n_frames_ = 1;
  int64_t sync_count_ = 0;
  double min_flush_distance_m_ = 5.0;
  bool flush_on_shutdown_ = true;
  bool done_ = false;
};

}  // namespace robot_bundle_sender_camlidar

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<robot_bundle_sender_camlidar::CamLidarBundleSenderNode>();
  rclcpp::spin(node);
  node->shutdown();
  rclcpp::shutdown();
  return 0;
}
