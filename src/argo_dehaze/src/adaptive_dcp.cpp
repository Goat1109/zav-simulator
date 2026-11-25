#include "argo_dehaze/adaptive_dcp.hpp"
#include <algorithm>
#include <vector>
#include <cmath>
#include <omp.h>

namespace zenith {

AdaptiveDCP::AdaptiveDCP(const AdaptiveDCPParams& params)
  : params_(params) {}

cv::Mat AdaptiveDCP::autoWhiteBalance(const cv::Mat& bgr_image, float strength) {
  std::vector<cv::Mat> channels;
  cv::split(bgr_image, channels);

  cv::Scalar mean_b = cv::mean(channels[0]);
  cv::Scalar mean_g = cv::mean(channels[1]);
  cv::Scalar mean_r = cv::mean(channels[2]);

  double mean_gray = (mean_b[0] + mean_g[0] + mean_r[0]) / 3.0;

  double scale_b = (1.0 - strength) + strength * (mean_gray / (mean_b[0] + 1e-4));
  double scale_g = (1.0 - strength) + strength * (mean_gray / (mean_g[0] + 1e-4));
  double scale_r = (1.0 - strength) + strength * (mean_gray / (mean_r[0] + 1e-4));

  channels[0].convertTo(channels[0], CV_8U, scale_b);
  channels[1].convertTo(channels[1], CV_8U, scale_g);
  channels[2].convertTo(channels[2], CV_8U, scale_r);

  cv::Mat balanced;
  cv::merge(channels, balanced);
  return balanced;
}

cv::Mat AdaptiveDCP::computeDarkChannel(const cv::Mat& bgr_norm) {
  int rows = bgr_norm.rows;
  int cols = bgr_norm.cols;

  // 1. Min across B, G, R color channels
  cv::Mat min_channel(rows, cols, CV_32FC1);

#pragma omp parallel for
  for (int r = 0; r < rows; ++r) {
    const cv::Vec3f* src_ptr = bgr_norm.ptr<cv::Vec3f>(r);
    float* dst_ptr = min_channel.ptr<float>(r);
    for (int c = 0; c < cols; ++c) {
      const cv::Vec3f& pix = src_ptr[c];
      dst_ptr[c] = std::min({pix[0], pix[1], pix[2]});
    }
  }

  // 2. Min filter via morphological erosion
  int ksize = params_.patch_size;
  if (ksize % 2 == 0) ksize += 1;
  cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(ksize, ksize));
  cv::Mat dark_channel;
  cv::erode(min_channel, dark_channel, kernel);

  return dark_channel;
}

cv::Vec3f AdaptiveDCP::estimateAtmosphericLight(const cv::Mat& bgr_image, const cv::Mat& dark_channel) {
  int rows = bgr_image.rows;
  int cols = bgr_image.cols;
  int total_pixels = rows * cols;
  int top_count = std::max(1, static_cast<int>(total_pixels * 0.001)); // Top 0.1%

  std::vector<std::pair<float, cv::Point>> dark_pixels;
  dark_pixels.reserve(total_pixels);

  for (int r = 0; r < rows; ++r) {
    const float* ptr = dark_channel.ptr<float>(r);
    for (int c = 0; c < cols; ++c) {
      dark_pixels.emplace_back(ptr[c], cv::Point(c, r));
    }
  }

  std::nth_element(dark_pixels.begin(), dark_pixels.begin() + top_count, dark_pixels.end(),
                   [](const auto& a, const auto& b) { return a.first > b.first; });

  float max_intensity = -1.0f;
  cv::Vec3f best_A(220.0f, 220.0f, 220.0f);

  for (int i = 0; i < top_count; ++i) {
    cv::Point pt = dark_pixels[i].second;
    cv::Vec3b bgr = bgr_image.at<cv::Vec3b>(pt);
    float intensity = 0.114f * bgr[0] + 0.587f * bgr[1] + 0.299f * bgr[2];
    if (intensity > max_intensity) {
      max_intensity = intensity;
      best_A = cv::Vec3f(bgr[0], bgr[1], bgr[2]);
    }
  }

  // Neutralize atmospheric light to prevent color tinting (e.g. blue/red cast)
  float mean_A = (best_A[0] + best_A[1] + best_A[2]) / 3.0f;
  float ratio = params_.neutral_atm_ratio;
  best_A = (1.0f - ratio) * best_A + ratio * cv::Vec3f(mean_A, mean_A, mean_A);

  // Safety clamping
  best_A[0] = std::clamp(best_A[0], 80.0f, 245.0f);
  best_A[1] = std::clamp(best_A[1], 80.0f, 245.0f);
  best_A[2] = std::clamp(best_A[2], 80.0f, 245.0f);

  return best_A;
}

float AdaptiveDCP::estimateHazeDensity(const cv::Mat& dark_channel) {
  cv::Scalar mean, stddev;
  cv::meanStdDev(dark_channel, mean, stddev);

  float mu = static_cast<float>(mean[0]);
  float hdi = (mu - 0.08f) / 0.50f;
  return std::clamp(hdi, 0.0f, 1.0f);
}

cv::Mat AdaptiveDCP::fastGuidedFilter(const cv::Mat& guide_gray, const cv::Mat& p, int radius, float eps, int s) {
  cv::Size orig_size = guide_gray.size();
  int sub_r = std::max(1, radius / s);

  cv::Mat guide_sub, p_sub;
  cv::Size sub_size(orig_size.width / s, orig_size.height / s);
  cv::resize(guide_gray, guide_sub, sub_size, 0, 0, cv::INTER_LINEAR);
  cv::resize(p, p_sub, sub_size, 0, 0, cv::INTER_LINEAR);

  cv::Mat mean_I, mean_p, mean_Ip, mean_II;
  cv::boxFilter(guide_sub, mean_I, CV_32F, cv::Size(sub_r * 2 + 1, sub_r * 2 + 1));
  cv::boxFilter(p_sub, mean_p, CV_32F, cv::Size(sub_r * 2 + 1, sub_r * 2 + 1));
  cv::boxFilter(guide_sub.mul(p_sub), mean_Ip, CV_32F, cv::Size(sub_r * 2 + 1, sub_r * 2 + 1));
  cv::boxFilter(guide_sub.mul(guide_sub), mean_II, CV_32F, cv::Size(sub_r * 2 + 1, sub_r * 2 + 1));

  cv::Mat cov_Ip = mean_Ip - mean_I.mul(mean_p);
  cv::Mat var_I = mean_II - mean_I.mul(mean_I);

  cv::Mat a = cov_Ip / (var_I + eps);
  cv::Mat b = mean_p - a.mul(mean_I);

  cv::Mat mean_a, mean_b;
  cv::boxFilter(a, mean_a, CV_32F, cv::Size(sub_r * 2 + 1, sub_r * 2 + 1));
  cv::boxFilter(b, mean_b, CV_32F, cv::Size(sub_r * 2 + 1, sub_r * 2 + 1));

  cv::Mat mean_a_orig, mean_b_orig;
  cv::resize(mean_a, mean_a_orig, orig_size, 0, 0, cv::INTER_LINEAR);
  cv::resize(mean_b, mean_b_orig, orig_size, 0, 0, cv::INTER_LINEAR);

  cv::Mat q = mean_a_orig.mul(guide_gray) + mean_b_orig;
  return q;
}

cv::Mat AdaptiveDCP::adaptiveToneMapping(const cv::Mat& bgr_img, float haze_density, float& applied_gamma) {
  // Use CIELAB color space to preserve chromaticity (a*, b*) and enhance only Lightness (L*)
  cv::Mat lab;
  cv::cvtColor(bgr_img, lab, cv::COLOR_BGR2Lab);

  std::vector<cv::Mat> channels;
  cv::split(lab, channels);

  // Analyze Lightness channel
  cv::Scalar mean_l = cv::mean(channels[0]);
  float cur_lum = std::max(10.0, mean_l[0]);
  float target_lum = params_.target_luminance;

  if (cur_lum < target_lum) {
    applied_gamma = std::log(target_lum / 255.0f) / std::log(cur_lum / 255.0f);
    applied_gamma = std::clamp(applied_gamma, 0.65f, 1.05f);
  } else {
    applied_gamma = 1.0f;
  }

  // Fast LUT for gamma correction on L channel
  unsigned char lut[256];
  for (int i = 0; i < 256; ++i) {
    float norm = i / 255.0f;
    float corrected = std::pow(norm, applied_gamma) * 255.0f;
    lut[i] = cv::saturate_cast<uchar>(corrected);
  }
  cv::Mat lut_mat(1, 256, CV_8U, lut);
  cv::LUT(channels[0], lut_mat, channels[0]);

  // CLAHE on L channel for structure & corner edge contrast
  if (params_.enable_clahe) {
    float clip = params_.clahe_clip_limit + 0.3f * haze_density;
    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(clip, cv::Size(8, 8));
    clahe->apply(channels[0], channels[0]);
  }

  cv::merge(channels, lab);
  cv::Mat result;
  cv::cvtColor(lab, result, cv::COLOR_Lab2BGR);
  return result;
}

cv::Mat AdaptiveDCP::dehaze(const cv::Mat& bgr_image, DehazeInfo* info) {
  auto start_time = std::chrono::high_resolution_clock::now();

  int rows = bgr_image.rows;
  int cols = bgr_image.cols;

  // 1. Auto White Balance to eliminate haze color cast (Mie/Rayleigh scattering tint)
  cv::Mat work_img = bgr_image;
  if (params_.enable_auto_white_balance) {
    work_img = autoWhiteBalance(bgr_image, params_.awb_strength);
  }

  // 2. Normalized image I / 255.0f
  cv::Mat img_float;
  work_img.convertTo(img_float, CV_32FC3, 1.0f / 255.0f);

  // 3. Initial Dark Channel for Haze Density Estimation & Atmospheric Light
  cv::Mat dark_init = computeDarkChannel(img_float);
  cv::Vec3f atm_light = estimateAtmosphericLight(work_img, dark_init);
  float haze_density = estimateHazeDensity(dark_init);

  // 4. Adaptive parameters
  float omega = params_.omega_min + haze_density * (params_.omega_max - params_.omega_min);
  float t0 = params_.t0_base + 0.05f * haze_density; // Safe lower bound [0.22 ~ 0.27]

  // 5. Normalized image by Atmospheric Light: I^c / A^c
  cv::Mat img_norm(rows, cols, CV_32FC3);
  cv::Vec3f A_norm = atm_light * (1.0f / 255.0f);

#pragma omp parallel for
  for (int r = 0; r < rows; ++r) {
    const cv::Vec3f* src = img_float.ptr<cv::Vec3f>(r);
    cv::Vec3f* dst = img_norm.ptr<cv::Vec3f>(r);
    for (int c = 0; c < cols; ++c) {
      dst[c][0] = src[c][0] / A_norm[0];
      dst[c][1] = src[c][1] / A_norm[1];
      dst[c][2] = src[c][2] / A_norm[2];
    }
  }

  // 6. Dark channel of normalized image
  cv::Mat dark_norm = computeDarkChannel(img_norm);

  // 7. Raw transmission map
  cv::Mat t_raw = 1.0f - omega * dark_norm;

  // 8. Fast Guided Filter for edge-preserving transmission refinement
  cv::Mat guide_gray;
  cv::cvtColor(img_float, guide_gray, cv::COLOR_BGR2GRAY);
  cv::Mat t_refined = fastGuidedFilter(guide_gray, t_raw, params_.guided_radius, params_.guided_eps, params_.guided_subsample);

  // Clamp transmission to safe lower bound [t0, 1.0]
  cv::max(t_refined, t0, t_refined);
  cv::min(t_refined, 1.0f, t_refined);

  // 9. Scene radiance recovery: J = (I_wb - A) / max(t, t0) + A
  cv::Mat dehazed_raw(rows, cols, CV_8UC3);

#pragma omp parallel for
  for (int r = 0; r < rows; ++r) {
    const cv::Vec3b* src_pix = work_img.ptr<cv::Vec3b>(r);
    const float* t_ptr = t_refined.ptr<float>(r);
    cv::Vec3b* dst_pix = dehazed_raw.ptr<cv::Vec3b>(r);

    for (int k = 0; k < 3; ++k) {
      float A_k = atm_light[k];
      for (int c = 0; c < cols; ++c) {
        float t_val = t_ptr[c];
        float j_val = (static_cast<float>(src_pix[c][k]) - A_k) / t_val + A_k;
        dst_pix[c][k] = cv::saturate_cast<uchar>(j_val);
      }
    }
  }

  // 10. Adaptive Luminance Tone Mapping (LAB Gamma + CLAHE)
  float applied_gamma = 1.0f;
  cv::Mat final_result = dehazed_raw;
  if (params_.enable_adaptive_tone) {
    final_result = adaptiveToneMapping(dehazed_raw, haze_density, applied_gamma);
  }

  auto end_time = std::chrono::high_resolution_clock::now();
  double elapsed_ms = std::chrono::duration<double, std::milli>(end_time - start_time).count();

  if (info) {
    info->haze_density = haze_density;
    info->omega_used = omega;
    info->atm_light = atm_light;
    cv::Scalar mean_t = cv::mean(t_refined);
    info->mean_transmission = static_cast<float>(mean_t[0]);
    info->applied_gamma = applied_gamma;
    info->elapsed_ms = elapsed_ms;
  }

  return final_result;
}

double AdaptiveDCP::calculatePSNR(const cv::Mat& img1, const cv::Mat& img2) {
  cv::Mat s1;
  cv::absdiff(img1, img2, s1);
  s1.convertTo(s1, CV_32F);
  s1 = s1.mul(s1);

  cv::Scalar s = cv::sum(s1);
  double sse = s.val[0] + s.val[1] + s.val[2];

  if (sse <= 1e-10) return 100.0;
  double mse = sse / (double)(img1.channels() * img1.total());
  double psnr = 10.0 * std::log10((255.0 * 255.0) / mse);
  return psnr;
}

double AdaptiveDCP::calculateSSIM(const cv::Mat& img1, const cv::Mat& img2) {
  const double C1 = 6.5025, C2 = 58.5225;

  std::vector<cv::Mat> ch1, ch2;
  cv::split(img1, ch1);
  cv::split(img2, ch2);

  double ssim_sum = 0.0;
  for (int c = 0; c < (int)ch1.size(); ++c) {
    cv::Mat I1, I2;
    ch1[c].convertTo(I1, CV_32F);
    ch2[c].convertTo(I2, CV_32F);

    cv::Mat I1_2 = I1.mul(I1);
    cv::Mat I2_2 = I2.mul(I2);
    cv::Mat I1_I2 = I1.mul(I2);

    cv::Mat mu1, mu2;
    cv::GaussianBlur(I1, mu1, cv::Size(11, 11), 1.5);
    cv::GaussianBlur(I2, mu2, cv::Size(11, 11), 1.5);

    cv::Mat mu1_2 = mu1.mul(mu1);
    cv::Mat mu2_2 = mu2.mul(mu2);
    cv::Mat mu1_mu2 = mu1.mul(mu2);

    cv::Mat sigma1_2, sigma2_2, sigma12;
    cv::GaussianBlur(I1_2, sigma1_2, cv::Size(11, 11), 1.5);
    sigma1_2 -= mu1_2;

    cv::GaussianBlur(I2_2, sigma2_2, cv::Size(11, 11), 1.5);
    sigma2_2 -= mu2_2;

    cv::GaussianBlur(I1_I2, sigma12, cv::Size(11, 11), 1.5);
    sigma12 -= mu1_mu2;

    cv::Mat t1 = 2 * mu1_mu2 + C1;
    cv::Mat t2 = 2 * sigma12 + C2;
    cv::Mat t3 = t1.mul(t2);

    t1 = mu1_2 + mu2_2 + C1;
    t2 = sigma1_2 + sigma2_2 + C2;
    cv::Mat t4 = t1.mul(t2);

    cv::Mat ssim_map;
    cv::divide(t3, t4, ssim_map);

    cv::Scalar mssim = cv::mean(ssim_map);
    ssim_sum += mssim[0];
  }

  return ssim_sum / ch1.size();
}

} // namespace zenith
