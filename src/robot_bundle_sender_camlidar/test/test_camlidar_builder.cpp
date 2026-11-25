// Checks of CamLidarBuilder: bundles close at the travel distance, frames/points/video stay aligned, int16 round trip.
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <unistd.h>
#include <fstream>
#include <string>
#include <vector>

#include <zlib.h>

#include "robot_bundle_sender_camlidar/camlidar_builder.hpp"

#ifdef HAVE_DRACO
#include <draco/compression/decode.h>
#endif
#include <algorithm>
#include <random>
#include <array>

using namespace robot_bundle_sender_camlidar;

#define CHECK(c)                                                         \
  do {                                                                   \
    if (!(c)) {                                                          \
      std::fprintf(stderr, "%s:%d CHECK failed: %s\n", __FILE__, __LINE__, #c); \
      std::exit(1);                                                      \
    }                                                                    \
  } while (0)

static std::vector<int16_t> unpack(const proto::CamLidarBundle& m) {
  std::vector<int16_t> out(m.raw_size_bytes() / 2);
  if (m.compression() == proto::CamLidarBundle::ZLIB) {
    uLongf n = m.raw_size_bytes();
    CHECK(uncompress(reinterpret_cast<Bytef*>(out.data()), &n, reinterpret_cast<const Bytef*>(m.points_i16().data()),
                     m.points_i16().size()) == Z_OK);
  } else {
    std::memcpy(out.data(), m.points_i16().data(), m.points_i16().size());
  }
  return out;
}

static int count_video_frames(const std::string& bytes) {
  const std::string path = "/tmp/test_camlidar_builder_" + std::to_string(getpid()) + ".mp4";
  std::ofstream(path, std::ios::binary).write(bytes.data(), bytes.size());
  const std::string cmd = "ffprobe -v error -count_frames -select_streams v:0 -show_entries stream=nb_read_frames -of csv=p=0 '" + path + "'";
  FILE* f = popen(cmd.c_str(), "r");
  int n = -1;
  if (f) {
    if (std::fscanf(f, "%d", &n) != 1) n = -1;
    pclose(f);
  }
  std::remove(path.c_str());
  return n;
}

int main() {
  BuilderParams bp;
  bp.session = "test";
  bp.bundle_distance_m = 1.0;
  bp.max_range_m = 50.0;
  VideoParams vp;
  CamLidarBuilder b(bp, vp);

  const int W = 64, H = 48;
  std::vector<uint8_t> img(W * H * 3);
  std::vector<proto::CamLidarBundle> out;
  std::vector<std::vector<float>> sent;
  for (int i = 0; i < 25; ++i) {
    for (size_t k = 0; k < img.size(); ++k) img[k] = static_cast<uint8_t>((k + 7 * i) % 251);
    std::vector<float> pts = {1.234f, -2.345f, 0.5f, float(i), 0.0f, 0.0f, 60.0f, 0.0f, 0.0f /* out of range */, NAN, 0, 0};
    sent.push_back({1.234f, -2.345f, 0.5f, float(i), 0.0f, 0.0f});
    const Eigen::Vector3d position(0.1 * i, 0.0, 0.0);
    std::string err;
    auto s = b.add_frame(1000 + i, position, Eigen::Quaterniond::Identity(), pts.data(), 4, 3, img.data(), W, H, &err);
    CHECK(err.empty());
    if (s) out.push_back(std::move(*s));
  }
  std::string err;
  auto last = b.flush(true, &err);
  CHECK(err.empty());
  CHECK(last.has_value());
  CHECK(last->last_in_session());
  out.push_back(std::move(*last));

  // 0.1 m per frame, 1 m per bundle: the first bundle closes on the frame that reaches 1.0 m (frames 0..10).
  CHECK(out.size() == 3);
  CHECK(out[0].frames_size() == 11 || out[0].frames_size() == 12);  // float sum of 0.1 m steps
  size_t frame = 0;
  for (const auto& m : out) {
    const auto pts = unpack(m);
    size_t off = 0;
    for (const auto& f : m.frames()) {
      CHECK(f.stamp_ns() == static_cast<int64_t>(1000 + frame));
      CHECK(f.num_points() == 2);
      for (int j = 0; j < 6; ++j) CHECK(std::abs(pts[off + j] * m.point_scale_m() - sent[frame][j]) <= 0.005 + 1e-6);
      off += 3 * f.num_points();
      ++frame;
    }
    CHECK(off == pts.size());
    CHECK(count_video_frames(m.video()) == m.frames_size());
  }
  CHECK(frame == 25);
#ifdef HAVE_DRACO
  {   // point_codec draco: every frame decodes back to exactly the int16 points (order may differ)
    BuilderParams dp = bp;
    dp.point_codec = "draco";
    dp.codec_threads = 3;
    dp.max_range_m = 0.0;
    CamLidarBuilder db(dp, vp);
    std::mt19937 rng(1);
    std::uniform_real_distribution<float> u(-40.f, 40.f);
    std::vector<std::vector<int16_t>> expect;
    std::vector<proto::CamLidarBundle> bundles;
    for (int i = 0; i < 25; ++i) {
      std::vector<float> pts;
      std::vector<int16_t> q;
      const int n = (i == 4) ? 0 : 300 + 37 * i;   // one frame without points
      for (int k = 0; k < n * 3; ++k) {
        const float v = u(rng);
        pts.push_back(v);
        q.push_back(static_cast<int16_t>(std::round(v * (1.0 / dp.point_scale_m))));   // same rounding as the builder
      }
      expect.push_back(q);
      std::string err;
      auto s = db.add_frame(5000 + i, Eigen::Vector3d(0.1 * i, 0, 0), Eigen::Quaterniond::Identity(), pts.data(), n, 3,
                            img.data(), W, H, &err);
      CHECK(err.empty());
      if (s) bundles.push_back(std::move(*s));
    }
    std::string err;
    auto last = db.flush(true, &err);
    CHECK(err.empty() && last.has_value());
    bundles.push_back(std::move(*last));
    size_t frame = 0;
    for (const auto& m : bundles) {
      CHECK(m.compression() == proto::CamLidarBundle::DRACO);
      CHECK(m.points_draco_size() == m.frames_size() && m.points_i16().empty());
      for (int j = 0; j < m.frames_size(); ++j, ++frame) {
        std::vector<int16_t> got;
        if (!m.points_draco(j).empty()) {
          draco::DecoderBuffer buf;
          buf.Init(m.points_draco(j).data(), m.points_draco(j).size());
          draco::Decoder dec;
          auto pc = dec.DecodePointCloudFromBuffer(&buf);
          CHECK(pc.ok());
          const draco::PointAttribute* att = pc.value()->GetNamedAttribute(draco::GeometryAttribute::POSITION);
          for (draco::PointIndex p(0); p < pc.value()->num_points(); ++p) {
            float v[3];
            att->GetMappedValue(p, v);
            for (float x : v) got.push_back(static_cast<int16_t>(std::lround(x)));
          }
        }
        auto as_rows = [](const std::vector<int16_t>& a) {
          std::vector<std::array<int16_t, 3>> r(a.size() / 3);
          for (size_t k = 0; k < r.size(); ++k) r[k] = {a[3 * k], a[3 * k + 1], a[3 * k + 2]};
          std::sort(r.begin(), r.end());
          return r;
        };
        CHECK(m.frames(j).num_points() == expect[frame].size() / 3);
        CHECK(as_rows(got) == as_rows(expect[frame]));
      }
    }
    CHECK(frame == 25);
  }
#endif
  std::printf("test_camlidar_builder: ok\n");
  return 0;
}
