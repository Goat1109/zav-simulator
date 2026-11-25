#include "robot_bundle_sender_camlidar/bundle_sender.hpp"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <filesystem>
#include <fstream>

#include <grpcpp/grpcpp.h>
#include "camlidar_bundle.grpc.pb.h"
#include "robot_bundle_sender_camlidar/point_codec.hpp"

namespace robot_bundle_sender_camlidar {

struct BundleSender::Impl {
  std::shared_ptr<grpc::Channel> channel;
  std::unique_ptr<proto::CamLidarBundleService::Stub> stub;
};

BundleSender::BundleSender(SenderParams params, Logger info, Logger warn)
    : p_(std::move(params)), info_(std::move(info)), warn_(std::move(warn)), impl_(std::make_unique<Impl>()) {
  grpc::ChannelArguments args;
  args.SetMaxSendMessageSize(256 * 1024 * 1024);
  impl_->channel = grpc::CreateCustomChannel(p_.server_address, grpc::InsecureChannelCredentials(), args);
  impl_->stub = proto::CamLidarBundleService::NewStub(impl_->channel);
  if (!p_.backup_dir.empty()) std::filesystem::create_directories(p_.backup_dir);
  worker_ = std::thread([this] { run(); });
}

BundleSender::~BundleSender() {
  stop_ = true;
  cv_.notify_all();
  if (worker_.joinable()) worker_.join();
}

void BundleSender::enqueue(proto::CamLidarBundle m) {
  write_backup(m);
  auto ptr = std::make_shared<const proto::CamLidarBundle>(std::move(m));
  std::lock_guard<std::mutex> lk(mu_);
  if (queue_.size() >= p_.queue_size) {
    warn_("send queue full: dropping " + queue_.front()->session() + " bundle " +
          std::to_string(queue_.front()->bundle_index()) + " (kept in backup_dir if set)");
    queue_.pop_front();
  }
  queue_.push_back(std::move(ptr));
  cv_.notify_all();
}

void BundleSender::hello(proto::Hello h) {
  std::lock_guard<std::mutex> lk(mu_);
  hello_msg_ = std::move(h);
  has_hello_ = true;
  cv_.notify_all();
}

void BundleSender::try_hello() {
  proto::Hello h;
  {
    std::lock_guard<std::mutex> lk(mu_);
    h = hello_msg_;
  }
  h.set_robot_time_ns(std::chrono::duration_cast<std::chrono::nanoseconds>(
                          std::chrono::system_clock::now().time_since_epoch()).count());
  grpc::ClientContext ctx;
  ctx.set_deadline(std::chrono::system_clock::now() + std::chrono::milliseconds(static_cast<int>(p_.hello_timeout_s * 1000)));
  proto::Ack ack;
  const auto t0 = std::chrono::steady_clock::now();
  const grpc::Status st = impl_->stub->SayHello(&ctx, h, &ack);
  const double rtt_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
  if (st.ok() && ack.ok()) {
    connected_ = true;
    char buf[64];
    std::snprintf(buf, sizeof(buf), "%.1f ms", rtt_ms);
    info_("connected to server " + p_.server_address + " (round trip " + buf + "): " + ack.message());
    return;
  }
  const std::string err = st.ok() ? "server rejected: " + ack.message() : st.error_message();
  // first failure, then about every 30 s
  const int every = std::max(1, static_cast<int>(30.0 / std::max(p_.retry_interval_s, 0.1)));
  if (hello_failures_++ % every == 0) {
    char interval[32];
    std::snprintf(interval, sizeof(interval), "%.1f s", p_.retry_interval_s);
    warn_("server " + p_.server_address + " not reachable (" + err + ") -- retrying every " + interval +
          "; bundles are kept in the send queue");
  }
}

bool BundleSender::drain(double timeout_s) {
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_s);
  while (std::chrono::steady_clock::now() < deadline) {
    {
      std::lock_guard<std::mutex> lk(mu_);
      if (queue_.empty() && !busy_) return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  return false;
}

bool BundleSender::send(const proto::CamLidarBundle& m, std::string* err) {
  grpc::ClientContext ctx;
  ctx.set_deadline(std::chrono::system_clock::now() + std::chrono::milliseconds(static_cast<int>(p_.rpc_timeout_s * 1000)));
  proto::Ack ack;
  const grpc::Status st = impl_->stub->SendBundle(&ctx, m, &ack);
  if (!st.ok()) {
    *err = st.error_message();
    return false;
  }
  if (!ack.ok()) {
    *err = "server rejected: " + ack.message();
    return false;
  }
  if (ack.message().rfind("rejected:", 0) == 0)   // the server kept it aside as undecodable: do not resend
    warn_("server could not unpack " + m.session() + " bundle " + std::to_string(m.bundle_index()) + " and kept it aside (" +
          ack.message() + ")");
  return true;
}

void BundleSender::run() {
  while (!stop_) {
    bool want_hello;
    {
      std::lock_guard<std::mutex> lk(mu_);
      want_hello = has_hello_ && !connected_;
    }
    if (want_hello) try_hello();
    std::shared_ptr<const proto::CamLidarBundle> m;
    {
      std::unique_lock<std::mutex> lk(mu_);
      if (has_hello_ && !connected_)   // keep checking the connection while nothing is queued
        cv_.wait_for(lk, std::chrono::duration<double>(p_.retry_interval_s), [this] { return stop_ || !queue_.empty(); });
      else
        cv_.wait(lk, [this] { return stop_ || !queue_.empty() || (has_hello_ && !connected_); });
      if (stop_ && queue_.empty()) return;
      if (queue_.empty()) continue;
      m = queue_.front();
      busy_ = true;
    }
    std::string err;
    const auto t0 = std::chrono::steady_clock::now();
    if (send(*m, &err)) {
      const double dt = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
      std::lock_guard<std::mutex> lk(mu_);
      if (!queue_.empty() && queue_.front() == m) queue_.pop_front();
      busy_ = false;
      connected_ = true;
      info_("sent " + m->session() + " bundle " + std::to_string(m->bundle_index()) + " (" +
            std::to_string(m->frames_size()) + " frames, points " + std::to_string(point_payload_bytes(*m) / 1024) +
            " KiB (" + m->point_codec_info() + "), video " + std::to_string(m->video().size() / 1024) + " KiB, " + std::to_string(dt) + " s)");
    } else {
      busy_ = false;
      warn_("send failed for " + m->session() + " bundle " + std::to_string(m->bundle_index()) + ": " + err + " -- retrying");
      std::unique_lock<std::mutex> lk(mu_);
      cv_.wait_for(lk, std::chrono::duration<double>(p_.retry_interval_s), [this] { return stop_.load(); });
    }
  }
}

void BundleSender::write_backup(const proto::CamLidarBundle& m) {
  if (p_.backup_dir.empty()) return;
  char name[64];
  std::snprintf(name, sizeof(name), "_%04u.pb", m.bundle_index());
  const std::filesystem::path path = std::filesystem::path(p_.backup_dir) / (m.session() + name);
  std::ofstream f(path.string() + ".tmp", std::ios::binary);
  if (!f || !m.SerializeToOstream(&f)) {
    warn_("could not write backup " + path.string());
    return;
  }
  f.close();
  std::filesystem::rename(path.string() + ".tmp", path);
}

}  // namespace robot_bundle_sender_camlidar
