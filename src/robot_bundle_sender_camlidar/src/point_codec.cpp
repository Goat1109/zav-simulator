#include "robot_bundle_sender_camlidar/point_codec.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#ifdef HAVE_DRACO
#include <draco/compression/encode.h>
#include <draco/point_cloud/point_cloud_builder.h>
#endif

namespace robot_bundle_sender_camlidar {

#ifdef HAVE_DRACO
bool draco_available() { return true; }

bool draco_encode_i16(const int16_t* xyz, size_t n, int speed, std::string* out, std::string* err) {
  out->clear();
  if (n == 0) return true;  // an empty frame is sent as an empty buffer
  std::vector<float> v(n * 3);
  float lo[3] = {std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max()};
  float hi[3] = {std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest(),
                 std::numeric_limits<float>::lowest()};
  for (size_t i = 0; i < n * 3; ++i) {
    v[i] = static_cast<float>(xyz[i]);
    lo[i % 3] = std::min(lo[i % 3], v[i]);
    hi[i % 3] = std::max(hi[i % 3], v[i]);
  }
  draco::PointCloudBuilder builder;
  builder.Start(static_cast<draco::PointIndex::ValueType>(n));
  const int att = builder.AddAttribute(draco::GeometryAttribute::POSITION, 3, draco::DT_FLOAT32);
  builder.SetAttributeValuesForAllPoints(att, v.data(), 0);
  std::unique_ptr<draco::PointCloud> pc = builder.Finalize(false);
  // one quantization step = one int16 unit: range = 2^bits - 1 over the frame's extent
  const float span = std::max({hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2]});
  const int bits = std::max(1, static_cast<int>(std::ceil(std::log2(static_cast<double>(span) + 1.0))));
  draco::Encoder encoder;
  encoder.SetAttributeExplicitQuantization(draco::GeometryAttribute::POSITION, bits, 3, lo,
                                           static_cast<float>((1u << bits) - 1));
  encoder.SetSpeedOptions(speed, speed);
  draco::EncoderBuffer buffer;
  const draco::Status st = encoder.EncodePointCloudToBuffer(*pc, &buffer);
  if (!st.ok()) {
    *err = std::string("draco: ") + st.error_msg();
    return false;
  }
  out->assign(buffer.data(), buffer.size());
  return true;
}
#else
bool draco_available() { return false; }

bool draco_encode_i16(const int16_t*, size_t, int, std::string*, std::string* err) {
  *err = "built without Draco (install libdraco-dev and rebuild)";
  return false;
}
#endif

WorkerPool::WorkerPool(size_t threads) {
  for (size_t i = 0; i < std::max<size_t>(1, threads); ++i) workers_.emplace_back([this] { run(); });
}

WorkerPool::~WorkerPool() {
  {
    std::lock_guard<std::mutex> lk(mu_);
    stop_ = true;
  }
  cv_.notify_all();
  for (auto& w : workers_) w.join();
}

std::future<void> WorkerPool::submit(std::function<void()> job) {
  std::packaged_task<void()> task(std::move(job));
  std::future<void> f = task.get_future();
  {
    std::lock_guard<std::mutex> lk(mu_);
    jobs_.push_back(std::move(task));
  }
  cv_.notify_one();
  return f;
}

void WorkerPool::run() {
  for (;;) {
    std::packaged_task<void()> task;
    {
      std::unique_lock<std::mutex> lk(mu_);
      cv_.wait(lk, [this] { return stop_ || !jobs_.empty(); });
      if (jobs_.empty()) return;  // stop_ and nothing left
      task = std::move(jobs_.front());
      jobs_.pop_front();
    }
    task();
  }
}

}  // namespace robot_bundle_sender_camlidar
