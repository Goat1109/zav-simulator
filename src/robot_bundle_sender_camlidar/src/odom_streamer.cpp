#include "robot_bundle_sender_camlidar/odom_streamer.hpp"

#include <chrono>

#include <grpcpp/grpcpp.h>

#include "camlidar_bundle.grpc.pb.h"

namespace robot_bundle_sender_camlidar {

namespace {
int64_t wall_ns() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
}
}  // namespace

struct OdomStreamer::Impl {
  std::shared_ptr<grpc::Channel> channel;
  std::unique_ptr<proto::CamLidarBundleService::Stub> stub;
  std::mutex ctx_mu;
  grpc::ClientContext* ctx = nullptr;   // the open stream, so the destructor can cancel it
};

OdomStreamer::OdomStreamer(OdomStreamParams params, Logger info, Logger warn)
    : p_(std::move(params)), info_(std::move(info)), warn_(std::move(warn)), impl_(std::make_unique<Impl>()) {
  grpc::ChannelArguments args;
  args.SetInt(GRPC_ARG_USE_LOCAL_SUBCHANNEL_POOL, 1);   // its own connection, not shared with the bundle channel
  args.SetInt(GRPC_ARG_KEEPALIVE_TIME_MS, 10000);
  args.SetInt(GRPC_ARG_KEEPALIVE_TIMEOUT_MS, 5000);
  args.SetInt(GRPC_ARG_KEEPALIVE_PERMIT_WITHOUT_CALLS, 1);
  impl_->channel = grpc::CreateCustomChannel(p_.server_address, grpc::InsecureChannelCredentials(), args);
  impl_->stub = proto::CamLidarBundleService::NewStub(impl_->channel);
  worker_ = std::thread([this] { run(); });
}

OdomStreamer::~OdomStreamer() {
  stop_ = true;
  cv_.notify_all();
  {
    std::lock_guard<std::mutex> lk(impl_->ctx_mu);
    if (impl_->ctx != nullptr) impl_->ctx->TryCancel();
  }
  if (worker_.joinable()) worker_.join();
}

void OdomStreamer::push(int64_t stamp_ns, double x, double y, double z, double qx, double qy, double qz, double qw) {
  std::lock_guard<std::mutex> lk(mu_);
  if (p_.max_rate_hz > 0.0 && last_push_ns_ != 0 &&
      stamp_ns - last_push_ns_ < static_cast<int64_t>(1e9 / p_.max_rate_hz))
    return;
  last_push_ns_ = stamp_ns;
  proto::OdomSample s;
  s.set_stamp_ns(stamp_ns);
  s.set_seq(seq_++);
  auto* p = s.mutable_pose();
  p->set_x(x); p->set_y(y); p->set_z(z);
  p->set_qx(qx); p->set_qy(qy); p->set_qz(qz); p->set_qw(qw);
  queue_.push_back(std::move(s));
  while (queue_.size() > p_.queue_size) {
    queue_.pop_front();
    ++dropped_;
  }
  cv_.notify_one();
}

void OdomStreamer::run() {
  bool connected_once = false;
  int failures = 0;
  while (!stop_) {
    grpc::ClientContext ctx;
    {
      std::lock_guard<std::mutex> lk(impl_->ctx_mu);
      impl_->ctx = &ctx;
    }
    proto::Ack ack;
    std::unique_ptr<grpc::ClientWriter<proto::OdomBatch>> writer(impl_->stub->StreamOdom(&ctx, &ack));
    bool ok = true;
    uint64_t sent = 0;
    while (ok && !stop_) {
      proto::OdomBatch batch;
      {
        std::unique_lock<std::mutex> lk(mu_);
        cv_.wait_for(lk, std::chrono::milliseconds(500), [this] { return stop_ || !queue_.empty(); });
        if (stop_) break;
        if (queue_.empty()) continue;
        batch.set_session(p_.session);
        batch.set_robot_id(p_.robot_id);
        const int64_t now = wall_ns();
        while (!queue_.empty()) {
          proto::OdomSample s = std::move(queue_.front());
          queue_.pop_front();
          s.set_robot_send_ns(now);
          *batch.add_samples() = std::move(s);
        }
      }
      ok = writer->Write(batch);
      if (!ok) {   // not delivered: put the samples back in front (newest kept if the queue overflows)
        std::lock_guard<std::mutex> lk(mu_);
        for (int i = batch.samples_size() - 1; i >= 0; --i) queue_.push_front(std::move(*batch.mutable_samples(i)));
        while (queue_.size() > p_.queue_size) {
          queue_.pop_front();
          ++dropped_;
        }
      }
      if (ok && sent++ == 0 && !connected_once) {
        connected_once = true;
        info_("odometry stream to " + p_.server_address + " started");
      }
      if (ok) failures = 0;
    }
    writer->WritesDone();
    const grpc::Status st = writer->Finish();
    {
      std::lock_guard<std::mutex> lk(impl_->ctx_mu);
      impl_->ctx = nullptr;
    }
    if (stop_) break;
    if (failures++ % 15 == 0)   // first failure, then about every 30 s at the default retry interval
      warn_("odometry stream to " + p_.server_address + " interrupted (" +
            (st.ok() ? std::string("closed by server") : st.error_message()) + ") -- reconnecting");
    connected_once = false;
    std::unique_lock<std::mutex> lk(mu_);
    cv_.wait_for(lk, std::chrono::duration<double>(p_.retry_interval_s), [this] { return stop_.load(); });
  }
}

}  // namespace robot_bundle_sender_camlidar
