#pragma once

#include <condition_variable>
#include <cstdint>
#include <deque>
#include <functional>
#include <future>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace robot_bundle_sender_camlidar {

// Size of a bundle's point payload in bytes, whichever codec it uses (for logs).
template <class Bundle>
size_t point_payload_bytes(const Bundle& m) {
  size_t n = m.points_i16().size();
  for (const auto& b : m.points_draco()) n += b.size();
  return n;
}

// True when the package was built against the Draco library (CMake found it).
bool draco_available();

// Encodes one frame's points (n x 3 int16, units of point_scale_m) as a Draco point cloud. The grid is pinned to one
// int16 unit (explicit quantization over the frame's extent), so decoding and rounding gives back exactly the same
// integer points (order may differ). speed: Draco encoding/decoding speed 0..10 (9 = DracoPy compression_level 1;
// 10 switches to the sequential encoder, which compresses little better than zlib). Returns false and sets err on failure.
bool draco_encode_i16(const int16_t* xyz, size_t n, int speed, std::string* out, std::string* err);

// Fixed-size worker pool: frames are compressed off the ROS callback thread, in parallel.
class WorkerPool {
 public:
  explicit WorkerPool(size_t threads);
  ~WorkerPool();
  WorkerPool(const WorkerPool&) = delete;
  WorkerPool& operator=(const WorkerPool&) = delete;
  std::future<void> submit(std::function<void()> job);

 private:
  void run();
  std::mutex mu_;
  std::condition_variable cv_;
  std::deque<std::packaged_task<void()>> jobs_;
  bool stop_ = false;
  std::vector<std::thread> workers_;
};

}  // namespace robot_bundle_sender_camlidar
