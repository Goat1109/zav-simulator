#pragma once

#include <atomic>
#include <condition_variable>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "camlidar_bundle.pb.h"

namespace robot_bundle_sender_camlidar {

struct SenderParams {
  std::string server_address = "127.0.0.1:50053";
  size_t queue_size = 32;        // bundles waiting to be sent; the oldest is dropped when full (still kept in backup_dir)
  double retry_interval_s = 2.0;
  double rpc_timeout_s = 20.0;
  std::string backup_dir;        // if set, every bundle is also written there as <session>_<index>.pb
  double hello_timeout_s = 3.0;  // startup connection check (SayHello) deadline per attempt
};

// Sends closed bundles on a background thread so the ROS callbacks never wait for the network. Failed sends are retried.
class BundleSender {
 public:
  using Logger = std::function<void(const std::string&)>;
  BundleSender(SenderParams params, Logger info, Logger warn);
  ~BundleSender();
  void enqueue(proto::CamLidarBundle m);
  // Starts the connection check: SayHello is retried every retry_interval_s until the server answers (logged once on
  // success; failures are logged on the first attempt and then every 30 s). A successful bundle also counts as connected.
  void hello(proto::Hello h);
  bool connected() const { return connected_; }
  // Waits up to timeout_s for the queue to drain (used at shutdown).
  bool drain(double timeout_s);

 private:
  void run();
  bool send(const proto::CamLidarBundle& m, std::string* err);
  void try_hello();
  void write_backup(const proto::CamLidarBundle& m);

  SenderParams p_;
  Logger info_, warn_;
  struct Impl;
  std::unique_ptr<Impl> impl_;
  std::mutex mu_;
  std::condition_variable cv_;
  std::deque<std::shared_ptr<const proto::CamLidarBundle>> queue_;
  std::atomic<bool> stop_{false};
  std::atomic<bool> busy_{false};
  std::atomic<bool> connected_{false};
  bool has_hello_ = false;           // guarded by mu_
  proto::Hello hello_msg_;           // guarded by mu_
  int hello_failures_ = 0;           // worker thread only
  std::thread worker_;
};

}  // namespace robot_bundle_sender_camlidar
