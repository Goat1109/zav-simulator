#include "robot_bundle_sender_camlidar/video_segment.hpp"

#include <csignal>
#include <fstream>
#include <iterator>
#include <sstream>
#include <unistd.h>

namespace robot_bundle_sender_camlidar {

VideoSegmentWriter::VideoSegmentWriter(VideoParams p) : p_(std::move(p)) {
  std::signal(SIGPIPE, SIG_IGN);  // a dead ffmpeg must not kill the node; the failed write is reported instead
}

VideoSegmentWriter::~VideoSegmentWriter() {
  if (pipe_ != nullptr) pclose(pipe_);
  if (!out_path_.empty()) std::remove(out_path_.c_str());
}

bool VideoSegmentWriter::start(int width, int height, std::string* err) {
  std::ostringstream path;
  path << p_.tmp_dir << "/robot_bundle_sender_camlidar_" << getpid() << "_" << serial_++ << ".mp4";
  out_path_ = path.str();
  std::ostringstream cmd;
  // exec setsid: ffmpeg runs in its own session, so a Ctrl-C in the robot's terminal (SIGINT to the whole foreground
  // process group) does not kill it mid-segment; the node then closes the last segment normally at shutdown.
  cmd << "exec setsid " << p_.ffmpeg_path << " -hide_banner -loglevel error -y -f rawvideo -pix_fmt bgr24 -s " << width << "x" << height
      << " -r " << p_.nominal_fps << " -i - -c:v " << p_.codec << " -preset " << p_.preset << " -threads " << p_.threads
      << " -bf 0 -g " << p_.gop << " -b:v " << p_.bitrate << " -maxrate " << p_.bitrate << " -bufsize " << p_.bitrate
      << " -pix_fmt yuv420p -movflags frag_keyframe+empty_moov -f mp4 '" << out_path_ << "'";
  pipe_ = popen(cmd.str().c_str(), "w");
  if (pipe_ == nullptr) {
    *err = "could not start ffmpeg: " + cmd.str();
    return false;
  }
  width_ = width;
  height_ = height;
  frames_ = 0;
  return true;
}

bool VideoSegmentWriter::add(const uint8_t* bgr, int width, int height, std::string* err) {
  if (pipe_ == nullptr && !start(width, height, err)) return false;
  if (width != width_ || height != height_) {
    *err = "image size changed inside a segment";
    return false;
  }
  const size_t n = static_cast<size_t>(width) * height * 3;
  if (std::fwrite(bgr, 1, n, pipe_) != n) {
    *err = "write to ffmpeg failed";
    return false;
  }
  ++frames_;
  return true;
}

std::vector<uint8_t> VideoSegmentWriter::finish(std::string* err) {
  std::vector<uint8_t> out;
  if (pipe_ == nullptr) return out;
  const int rc = pclose(pipe_);
  pipe_ = nullptr;
  if (rc != 0) {
    *err = "ffmpeg exited with status " + std::to_string(rc);
  } else {
    std::ifstream f(out_path_, std::ios::binary);
    out.assign(std::istreambuf_iterator<char>(f), std::istreambuf_iterator<char>());
    if (out.empty()) *err = "ffmpeg produced an empty segment";
  }
  std::remove(out_path_.c_str());
  out_path_.clear();
  frames_ = 0;
  return out;
}

}  // namespace robot_bundle_sender_camlidar
