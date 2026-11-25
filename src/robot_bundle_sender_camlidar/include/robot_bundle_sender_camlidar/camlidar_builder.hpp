#pragma once

#include <cstdint>
#include <future>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Geometry>

#include "camlidar_bundle.pb.h"
#include "robot_bundle_sender_camlidar/point_codec.hpp"
#include "robot_bundle_sender_camlidar/video_segment.hpp"

namespace robot_bundle_sender_camlidar {

struct BuilderParams {
  std::string session;
  double bundle_distance_m = 40.0;
  double point_scale_m = 0.01;  // int16 quantisation step of the per-frame points (0.01 = 1 cm, range +-327 m)
  double min_range_m = 0.0;     // per-frame range filter in the LiDAR frame (0 = off)
  double max_range_m = 0.0;
  bool compress_points = true;  // false = send the int16 points uncompressed (whatever point_codec says)
  std::string point_codec = "zlib";  // zlib | draco (per frame, 1 cm grid, lossless on it) | none
  int draco_speed = 9;          // Draco speed 0..10 (9 = DracoPy level 1; 10 = sequential encoder, barely beats zlib)
  int codec_threads = 2;        // Draco worker threads (frames are encoded in parallel, off the ROS callback)
  proto::SessionInfo session_info;
};

// Collects synchronised (pose, image, point cloud) frames. Points stay per frame (temporal geometry, LiDAR frame, int16),
// images are streamed into one video segment, and a bundle is closed every bundle_distance_m of travel.
class CamLidarBuilder {
 public:
  CamLidarBuilder(BuilderParams bp, VideoParams vp);
  // position, orientation: world <- base as received; sent unchanged. points: n x stride floats, x/y/z first,
  // in the LiDAR frame. bgr: width x height x 3 bytes.
  // Returns the closed bundle when this frame reaches bundle_distance_m. On a video error the frame is dropped (err set).
  std::optional<proto::CamLidarBundle> add_frame(int64_t stamp_ns, const Eigen::Vector3d& position,
                                                 const Eigen::Quaterniond& orientation, const float* points,
                                                 size_t n, size_t stride, const uint8_t* bgr, int width, int height,
                                                 std::string* err);
  // Closes the partial bundle (shutdown). Empty if it has no frames.
  std::optional<proto::CamLidarBundle> flush(bool last_in_session, std::string* err);
  double distance() const { return distance_; }
  size_t frames() const { return frames_.size(); }

 private:
  proto::CamLidarBundle finish(bool last_in_session, std::string* err);
  BuilderParams p_;
  VideoSegmentWriter video_;
  uint32_t index_ = 0;
  std::vector<proto::Frame> frames_;
  std::vector<int16_t> points_;   // all frames' int16 points (zlib/none payload, and the fallback if Draco fails)
  struct DracoSlot {
    std::vector<int16_t> xyz;
    std::string data, err;
    bool ok = false;
  };
  std::unique_ptr<WorkerPool> pool_;                 // only with point_codec draco
  std::vector<std::shared_ptr<DracoSlot>> draco_;    // one per frame of the open bundle
  std::vector<std::future<void>> pending_;
  std::optional<Eigen::Vector3d> last_position_;
  double distance_ = 0.0;
};

}  // namespace robot_bundle_sender_camlidar
