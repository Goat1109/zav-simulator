#pragma once

#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

namespace robot_bundle_sender_camlidar {

struct VideoParams {
  std::string ffmpeg_path = "ffmpeg";
  std::string codec = "libx264";   // software H.264; a hardware encoder name (e.g. h264_v4l2m2m) can be used if available
  std::string bitrate = "1M";
  std::string preset = "veryfast";
  int gop = 20;                    // frames between key frames (2 s at 10 Hz)
  int threads = 1;
  double nominal_fps = 10.0;       // only for the container; real stamps travel as metadata
  std::string tmp_dir = "/tmp";
};

// Encodes the frames of one bundle into one fragmented-MP4 H.264 segment by streaming raw BGR frames to an ffmpeg child
// process: frames are compressed as they arrive, so only the encoder's own reference frames stay in memory. Every segment
// starts with a key frame and decodes on its own. No B-frames, so frame order in the file = order of add().
class VideoSegmentWriter {
 public:
  explicit VideoSegmentWriter(VideoParams p);
  ~VideoSegmentWriter();
  VideoSegmentWriter(const VideoSegmentWriter&) = delete;
  VideoSegmentWriter& operator=(const VideoSegmentWriter&) = delete;

  // Starts a segment on the first frame; every frame must have this size. Returns false (and logs to err) on failure.
  bool add(const uint8_t* bgr, int width, int height, std::string* err);
  // Closes the segment and returns its bytes (empty if no frame was added or encoding failed).
  std::vector<uint8_t> finish(std::string* err);
  size_t frames() const { return frames_; }
  int width() const { return width_; }
  int height() const { return height_; }

 private:
  bool start(int width, int height, std::string* err);
  VideoParams p_;
  FILE* pipe_ = nullptr;
  std::string out_path_;
  int width_ = 0, height_ = 0;
  size_t frames_ = 0;
  uint64_t serial_ = 0;
};

}  // namespace robot_bundle_sender_camlidar
