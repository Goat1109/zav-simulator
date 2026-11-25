#pragma once

#include <opencv2/opencv.hpp>
#include <string>
#include <chrono>

namespace zenith {

struct AdaptiveDCPParams {
  int patch_size = 15;            // Dark channel min-filter kernel size (patch_size x patch_size)
  float omega_min = 0.65f;        // Minimum haze removal factor (clear condition)
  float omega_max = 0.90f;        // Maximum haze removal factor (dense haze condition)
  float t0_base = 0.22f;          // Safe transmission lower bound (prevents noise explosion)
  int guided_radius = 16;         // Guided filter window radius
  float guided_eps = 1e-3f;       // Guided filter regularization factor
  int guided_subsample = 4;       // Fast guided filter downscale factor (e.g. 4x speedup)
  float target_luminance = 115.0f;// Target average luminance for adaptive gamma
  bool enable_auto_white_balance = true; // Auto white balance to eliminate haze color cast (blue tint)
  float awb_strength = 0.75f;     // AWB strength [0.0 ~ 1.0]
  float neutral_atm_ratio = 0.50f;// Ratio to neutralize atmospheric light (chroma suppression)
  bool enable_adaptive_tone = true; // Adaptive luminance gamma & CLAHE contrast
  bool enable_clahe = true;       // CLAHE for structure & feature edge preservation
  float clahe_clip_limit = 1.8f;  // CLAHE clip limit
};

struct DehazeInfo {
  float haze_density = 0.0f;      // Estimated haze density index [0.0 ~ 1.0]
  float omega_used = 0.0f;        // Dynamically selected omega
  cv::Vec3f atm_light = cv::Vec3f(0.0f, 0.0f, 0.0f); // Atmospheric light (B, G, R)
  float mean_transmission = 0.0f; // Average transmission
  float applied_gamma = 1.0f;     // Applied adaptive gamma
  double elapsed_ms = 0.0;        // Total processing time in milliseconds
};

class AdaptiveDCP {
public:
  explicit AdaptiveDCP(const AdaptiveDCPParams& params = AdaptiveDCPParams());
  ~AdaptiveDCP() = default;

  void setParams(const AdaptiveDCPParams& params) { params_ = params; }
  const AdaptiveDCPParams& getParams() const { return params_; }

  // Main dehazing interface
  cv::Mat dehaze(const cv::Mat& bgr_image, DehazeInfo* info = nullptr);

  // Quality metric calculators
  static double calculatePSNR(const cv::Mat& img1, const cv::Mat& img2);
  static double calculateSSIM(const cv::Mat& img1, const cv::Mat& img2);

private:
  AdaptiveDCPParams params_;

  cv::Mat autoWhiteBalance(const cv::Mat& bgr_image, float strength);
  cv::Mat computeDarkChannel(const cv::Mat& bgr_norm);
  cv::Vec3f estimateAtmosphericLight(const cv::Mat& bgr_image, const cv::Mat& dark_channel);
  float estimateHazeDensity(const cv::Mat& dark_channel);
  cv::Mat fastGuidedFilter(const cv::Mat& guide_gray, const cv::Mat& p, int radius, float eps, int s);
  cv::Mat adaptiveToneMapping(const cv::Mat& bgr_img, float haze_density, float& applied_gamma);
};

} // namespace zenith
