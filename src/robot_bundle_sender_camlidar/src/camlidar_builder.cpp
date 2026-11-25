#include "robot_bundle_sender_camlidar/camlidar_builder.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include <zlib.h>

namespace robot_bundle_sender_camlidar {

CamLidarBuilder::CamLidarBuilder(BuilderParams bp, VideoParams vp) : p_(std::move(bp)), video_(std::move(vp)) {
  if (p_.point_codec != "zlib" && p_.point_codec != "draco" && p_.point_codec != "none")
    throw std::invalid_argument("point_codec must be zlib, draco or none (got '" + p_.point_codec + "')");
  if (!p_.compress_points) p_.point_codec = "none";
  if (p_.point_codec == "draco") {
    if (!draco_available())
      throw std::invalid_argument("point_codec draco: this build has no Draco (install libdraco-dev and rebuild)");
    pool_ = std::make_unique<WorkerPool>(static_cast<size_t>(std::max(1, p_.codec_threads)));
  }
}

std::optional<proto::CamLidarBundle> CamLidarBuilder::add_frame(int64_t stamp_ns, const Eigen::Vector3d& position,
                                                                const Eigen::Quaterniond& orientation,
                                                                const float* points, size_t n, size_t stride,
                                                                const uint8_t* bgr, int width, int height,
                                                                std::string* err) {
  // The image goes first: if it cannot be encoded the whole frame is dropped, so frames, points and video stay aligned.
  if (!video_.add(bgr, width, height, err)) return std::nullopt;

  if (last_position_) distance_ += (position - *last_position_).norm();  // like semantic_submapper: carried across submaps
  last_position_ = position;

  const double inv = 1.0 / p_.point_scale_m;
  const double lim = std::numeric_limits<int16_t>::max();
  const double min2 = p_.min_range_m * p_.min_range_m, max2 = p_.max_range_m * p_.max_range_m;
  uint32_t kept = 0;
  const size_t first = points_.size();
  for (size_t i = 0; i < n; ++i) {
    const float* q = points + i * stride;
    if (!std::isfinite(q[0]) || !std::isfinite(q[1]) || !std::isfinite(q[2])) continue;
    const double r2 = double(q[0]) * q[0] + double(q[1]) * q[1] + double(q[2]) * q[2];
    if (r2 < min2 || (p_.max_range_m > 0.0 && r2 > max2)) continue;
    const double a = std::round(q[0] * inv), b = std::round(q[1] * inv), c = std::round(q[2] * inv);
    if (std::abs(a) > lim || std::abs(b) > lim || std::abs(c) > lim) continue;  // beyond the int16 range
    points_.push_back(static_cast<int16_t>(a));
    points_.push_back(static_cast<int16_t>(b));
    points_.push_back(static_cast<int16_t>(c));
    ++kept;
  }
  if (pool_) {   // Draco: encode this frame on a worker thread; collected in order when the bundle closes
    auto slot = std::make_shared<DracoSlot>();
    slot->xyz.assign(points_.begin() + first, points_.end());
    const int speed = p_.draco_speed;
    pending_.push_back(pool_->submit([slot, speed] {
      slot->ok = draco_encode_i16(slot->xyz.data(), slot->xyz.size() / 3, speed, &slot->data, &slot->err);
      slot->xyz = {};
    }));
    draco_.push_back(std::move(slot));
  }
  proto::Frame f;
  f.set_stamp_ns(stamp_ns);
  f.set_num_points(kept);
  auto* p = f.mutable_pose();
  p->set_x(position.x()); p->set_y(position.y()); p->set_z(position.z());
  p->set_qx(orientation.x()); p->set_qy(orientation.y()); p->set_qz(orientation.z()); p->set_qw(orientation.w());
  frames_.push_back(std::move(f));

  if (distance_ >= p_.bundle_distance_m) return finish(false, err);
  return std::nullopt;
}

std::optional<proto::CamLidarBundle> CamLidarBuilder::flush(bool last_in_session, std::string* err) {
  if (frames_.empty()) return std::nullopt;
  return finish(last_in_session, err);
}

proto::CamLidarBundle CamLidarBuilder::finish(bool last_in_session, std::string* err) {
  proto::CamLidarBundle m;
  m.set_session(p_.session);
  m.set_bundle_index(index_++);
  *m.mutable_session_info() = p_.session_info;
  for (auto& f : frames_) *m.add_frames() = std::move(f);
  m.set_distance_traveled_m(distance_);
  m.set_last_in_session(last_in_session);
  m.set_point_scale_m(p_.point_scale_m);

  const auto* raw = reinterpret_cast<const Bytef*>(points_.data());  // little-endian hosts (x86, ARM)
  const uLong raw_size = static_cast<uLong>(points_.size() * sizeof(int16_t));
  m.set_raw_size_bytes(raw_size);
  m.set_compression(proto::CamLidarBundle::NONE);
  bool draco_ok = false;
  if (pool_) {
    for (auto& f : pending_) f.wait();
    draco_ok = true;
    for (const auto& s : draco_) {
      if (!s->ok) {
        draco_ok = false;
        if (!err->empty()) *err += "; ";
        *err += "Draco failed (" + s->err + "), bundle sent with zlib";
        break;
      }
    }
    if (draco_ok) {
      m.set_compression(proto::CamLidarBundle::DRACO);
      for (const auto& s : draco_) m.add_points_draco(s->data);
    }
    pending_.clear();
    draco_.clear();
  }
  if (!draco_ok && p_.point_codec != "none" && raw_size > 0) {
    uLongf out_size = compressBound(raw_size);
    std::string buf(out_size, '\0');
    if (compress2(reinterpret_cast<Bytef*>(buf.data()), &out_size, raw, raw_size, Z_BEST_SPEED) == Z_OK) {
      buf.resize(out_size);
      m.set_compression(proto::CamLidarBundle::ZLIB);
      m.set_points_i16(std::move(buf));
    }
  }
  if (m.compression() == proto::CamLidarBundle::NONE) m.set_points_i16(reinterpret_cast<const char*>(raw), raw_size);
  m.set_point_codec_info(m.compression() == proto::CamLidarBundle::DRACO
                             ? "draco speed " + std::to_string(p_.draco_speed)
                             : (m.compression() == proto::CamLidarBundle::ZLIB ? "zlib level 1" : "none"));

  const std::vector<uint8_t> video = video_.finish(err);
  m.set_video_codec("h264");
  m.set_video_container("mp4");
  m.set_video(video.data(), video.size());

  frames_.clear();
  points_.clear();
  distance_ = 0.0;
  return m;
}

}  // namespace robot_bundle_sender_camlidar
