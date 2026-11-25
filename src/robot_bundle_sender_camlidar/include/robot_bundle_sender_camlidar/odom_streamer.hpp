#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "camlidar_bundle.pb.h"

namespace robot_bundle_sender_camlidar {

struct OdomStreamParams {
  std::string server_address = "127.0.0.1:50053";
  std::string session, robot_id;
  double max_rate_hz = 0.0;        // 0 = every odometry message; otherwise at most this many per second
  size_t queue_size = 200;         // samples waiting while the link is slow; the oldest are dropped first
  double retry_interval_s = 2.0;   // reconnect delay after the stream breaks
};

// Streams live odometry to the server (StreamOdom) on its own gRPC channel, so a large bundle upload on the bundle channel
// does not hold it back at the gRPC level. push() never blocks. While the server is unreachable samples wait in a queue of
// queue_size (a failed write puts its samples back); when it overflows the oldest are dropped (the server sees the gap in
// seq). The stream reconnects after failures.
class OdomStreamer {
 public:
  using Logger = std::function<void(const std::string&)>;
  OdomStreamer(OdomStreamParams params, Logger info, Logger warn);
  ~OdomStreamer();
  OdomStreamer(const OdomStreamer&) = delete;
  OdomStreamer& operator=(const OdomStreamer&) = delete;
  void push(int64_t stamp_ns, double x, double y, double z, double qx, double qy, double qz, double qw);
  uint64_t dropped() const { return dropped_; }

 private:
  void run();
  OdomStreamParams p_;
  Logger info_, warn_;
  struct Impl;
  std::unique_ptr<Impl> impl_;
  std::mutex mu_;
  std::condition_variable cv_;
  std::deque<proto::OdomSample> queue_;
  uint64_t seq_ = 0;               // guarded by mu_
  int64_t last_push_ns_ = 0;       // guarded by mu_ (rate limit)
  std::atomic<uint64_t> dropped_{0};
  std::atomic<bool> stop_{false};
  std::thread worker_;
};

}  // namespace robot_bundle_sender_camlidar
