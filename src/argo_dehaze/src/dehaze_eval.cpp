#include "argo_dehaze/adaptive_dcp.hpp"
#include <iostream>
#include <iomanip>
#include <filesystem>
#include <vector>
#include <numeric>

namespace fs = std::filesystem;

void printUsage() {
  std::cout << "Usage:" << std::endl;
  std::cout << "  Single image mode:" << std::endl;
  std::cout << "    argo_dehaze_eval --hazy <hazy_img> --gt <gt_img> [--out_dir <dir>] [--camera_res]" << std::endl;
  std::cout << "  Dataset benchmark mode:" << std::endl;
  std::cout << "    argo_dehaze_eval --dataset <Dense-Haze dir> [--limit <num>] [--out_dir <dir>] [--camera_res]" << std::endl;
  std::cout << "  Option --camera_res: Resizes inputs to 640x512 (Hikrobot camera resolution) to benchmark drone runtime." << std::endl;
}

int main(int argc, char** argv) {
  std::string hazy_path = "";
  std::string gt_path = "";
  std::string dataset_dir = "";
  std::string out_dir = "/home/radxa/zenith_ws/dehaze_results";
  int limit = -1;
  bool camera_res = false;

  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg == "--hazy" && i + 1 < argc) hazy_path = argv[++i];
    else if (arg == "--gt" && i + 1 < argc) gt_path = argv[++i];
    else if (arg == "--dataset" && i + 1 < argc) dataset_dir = argv[++i];
    else if (arg == "--out_dir" && i + 1 < argc) out_dir = argv[++i];
    else if (arg == "--limit" && i + 1 < argc) limit = std::stoi(argv[++i]);
    else if (arg == "--camera_res") camera_res = true;
    else if (arg == "-h" || arg == "--help") {
      printUsage();
      return 0;
    }
  }

  if (hazy_path.empty() && dataset_dir.empty()) {
    // Default to the Dense-Haze dataset in zenith_ws
    dataset_dir = "/home/radxa/zenith_ws/Dense-Haze CVPR 2019";
    std::cout << "No arguments provided. Defaulting to: " << dataset_dir << std::endl;
  }

  fs::create_directories(out_dir);

  zenith::AdaptiveDCP dehazer;

  if (!hazy_path.empty()) {
    // Single image evaluation
    cv::Mat hazy = cv::imread(hazy_path);
    if (hazy.empty()) {
      std::cerr << "Failed to read hazy image: " << hazy_path << std::endl;
      return 1;
    }

    cv::Mat gt;
    if (!gt_path.empty()) {
      gt = cv::imread(gt_path);
    }

    if (camera_res) {
      cv::resize(hazy, hazy, cv::Size(640, 512));
      if (!gt.empty()) cv::resize(gt, gt, cv::Size(640, 512));
    }

    zenith::DehazeInfo info;
    cv::Mat dehazed = dehazer.dehaze(hazy, &info);

    std::cout << "\n================ Single Image Evaluation ================" << std::endl;
    std::cout << "Image size: " << hazy.cols << "x" << hazy.rows << std::endl;
    std::cout << "Elapsed time: " << std::fixed << std::setprecision(2) << info.elapsed_ms << " ms" << std::endl;
    std::cout << "Haze density (HDI): " << info.haze_density << std::endl;
    std::cout << "Omega used: " << info.omega_used << std::endl;
    std::cout << "Atmospheric light: [" << (int)info.atm_light[0] << ", " 
              << (int)info.atm_light[1] << ", " << (int)info.atm_light[2] << "]" << std::endl;
    std::cout << "Applied Gamma: " << info.applied_gamma << std::endl;

    if (!gt.empty()) {
      double hazy_psnr = zenith::AdaptiveDCP::calculatePSNR(hazy, gt);
      double hazy_ssim = zenith::AdaptiveDCP::calculateSSIM(hazy, gt);
      double dehazed_psnr = zenith::AdaptiveDCP::calculatePSNR(dehazed, gt);
      double dehazed_ssim = zenith::AdaptiveDCP::calculateSSIM(dehazed, gt);

      std::cout << "---------------- Quality Metrics (vs GT) ----------------" << std::endl;
      std::cout << "Hazy Input   -> PSNR: " << hazy_psnr << " dB | SSIM: " << hazy_ssim << std::endl;
      std::cout << "Dehazed Output -> PSNR: " << dehazed_psnr << " dB | SSIM: " << dehazed_ssim << std::endl;
      std::cout << "Improvement  -> Delta PSNR: " << (dehazed_psnr - hazy_psnr) 
                << " dB | Delta SSIM: " << (dehazed_ssim - hazy_ssim) << std::endl;

      // Save comparison image: [Hazy | Dehazed | GT]
      cv::Mat comparison;
      cv::hconcat(std::vector<cv::Mat>{hazy, dehazed, gt}, comparison);
      std::string comp_path = out_dir + "/single_compare.png";
      cv::imwrite(comp_path, comparison);
      std::cout << "Comparison image saved to: " << comp_path << std::endl;
    } else {
      std::string out_path = out_dir + "/single_dehazed.png";
      cv::imwrite(out_path, dehazed);
      std::cout << "Dehazed image saved to: " << out_path << std::endl;
    }

    return 0;
  }

  // Dataset evaluation mode
  std::string hazy_dir = dataset_dir + "/hazy";
  std::string gt_dir = dataset_dir + "/GT";

  if (!fs::exists(hazy_dir) || !fs::exists(gt_dir)) {
    std::cerr << "Dataset directories not found: " << hazy_dir << " or " << gt_dir << std::endl;
    return 1;
  }

  std::vector<fs::path> hazy_files;
  for (const auto& entry : fs::directory_iterator(hazy_dir)) {
    if (entry.path().extension() == ".png" || entry.path().extension() == ".jpg") {
      hazy_files.push_back(entry.path());
    }
  }
  std::sort(hazy_files.begin(), hazy_files.end());

  int total_to_eval = (limit > 0 && limit < (int)hazy_files.size()) ? limit : hazy_files.size();

  std::cout << "\n================ Dense-Haze Dataset Benchmark ================" << std::endl;
  std::cout << "Evaluating " << total_to_eval << " images from: " << dataset_dir << std::endl;
  std::cout << std::string(85, '-') << std::endl;
  std::cout << std::setw(12) << "Image" 
            << std::setw(10) << "Hazy PSNR" 
            << std::setw(11) << "Dehaze PSNR" 
            << std::setw(10) << "Delta"
            << std::setw(11) << "SSIM"
            << std::setw(14) << "ORB Feats"
            << std::setw(12) << "Time (ms)" << std::endl;
  std::cout << std::string(85, '-') << std::endl;

  std::vector<double> psnr_hazy_vec, psnr_dehazed_vec;
  std::vector<double> ssim_hazy_vec, ssim_dehazed_vec;
  std::vector<double> time_vec;

  for (int i = 0; i < total_to_eval; ++i) {
    fs::path h_path = hazy_files[i];
    std::string stem = h_path.stem().string(); // e.g. "01_hazy"
    std::string prefix = stem.substr(0, stem.find("_hazy"));
    fs::path g_path = fs::path(gt_dir) / (prefix + "_GT.png");

    if (!fs::exists(g_path)) {
      std::cerr << "GT file missing for: " << h_path << std::endl;
      continue;
    }

    cv::Mat hazy = cv::imread(h_path.string());
    cv::Mat gt = cv::imread(g_path.string());
    if (hazy.empty() || gt.empty()) continue;

    if (camera_res) {
      cv::resize(hazy, hazy, cv::Size(640, 512));
      cv::resize(gt, gt, cv::Size(640, 512));
    }

    zenith::DehazeInfo info;
    cv::Mat dehazed = dehazer.dehaze(hazy, &info);

    double h_psnr = zenith::AdaptiveDCP::calculatePSNR(hazy, gt);
    double d_psnr = zenith::AdaptiveDCP::calculatePSNR(dehazed, gt);
    double h_ssim = zenith::AdaptiveDCP::calculateSSIM(hazy, gt);
    double d_ssim = zenith::AdaptiveDCP::calculateSSIM(dehazed, gt);

    psnr_hazy_vec.push_back(h_psnr);
    psnr_dehazed_vec.push_back(d_psnr);
    ssim_hazy_vec.push_back(h_ssim);
    ssim_dehazed_vec.push_back(d_ssim);
    time_vec.push_back(info.elapsed_ms);

    // Calculate FAST/ORB Feature Count for SLAM feasibility
    cv::Ptr<cv::ORB> orb = cv::ORB::create(2000);
    std::vector<cv::KeyPoint> kp_hazy, kp_dehazed;
    orb->detect(hazy, kp_hazy);
    orb->detect(dehazed, kp_dehazed);

    std::cout << std::setw(12) << (prefix + ".png")
              << std::setw(10) << std::fixed << std::setprecision(2) << h_psnr << " dB"
              << std::setw(11) << std::fixed << std::setprecision(2) << d_psnr << " dB"
              << std::setw(10) << std::showpos << (d_psnr - h_psnr) << std::noshowpos << " dB"
              << std::setw(11) << std::fixed << std::setprecision(4) << d_ssim
              << std::setw(10) << (std::to_string(kp_hazy.size()) + "->" + std::to_string(kp_dehazed.size()))
              << std::setw(10) << std::fixed << std::setprecision(1) << info.elapsed_ms << " ms"
              << std::endl;

    // Save comparison for first 5 images or if limit is small
    if (i < 5) {
      cv::Mat comp;
      cv::Mat h_s, d_s, g_s;
      if (camera_res) {
        h_s = hazy.clone();
        d_s = dehazed.clone();
        g_s = gt.clone();
      } else {
        cv::resize(hazy, h_s, cv::Size(), 0.5, 0.5);
        cv::resize(dehazed, d_s, cv::Size(), 0.5, 0.5);
        cv::resize(gt, g_s, cv::Size(), 0.5, 0.5);
      }
      cv::hconcat(std::vector<cv::Mat>{h_s, d_s, g_s}, comp);

      // Clean, professional text labels with background banner
      int bar_h = 35;
      cv::Rect bar_rect(0, 0, comp.cols, bar_h);
      cv::rectangle(comp, bar_rect, cv::Scalar(30, 30, 30), cv::FILLED);

      cv::putText(comp, "Hazy Input (Original)", cv::Point(15, 24), cv::FONT_HERSHEY_SIMPLEX, 0.65, cv::Scalar(80, 80, 255), 2);
      cv::putText(comp, "Dehazed (Adaptive DCP)", cv::Point(h_s.cols + 15, 24), cv::FONT_HERSHEY_SIMPLEX, 0.65, cv::Scalar(80, 255, 80), 2);
      cv::putText(comp, "Ground Truth", cv::Point(h_s.cols * 2 + 15, 24), cv::FONT_HERSHEY_SIMPLEX, 0.65, cv::Scalar(255, 220, 80), 2);

      cv::imwrite(out_dir + "/compare_" + prefix + ".png", comp);
    }
  }

  // Summary
  double avg_h_psnr = std::accumulate(psnr_hazy_vec.begin(), psnr_hazy_vec.end(), 0.0) / psnr_hazy_vec.size();
  double avg_d_psnr = std::accumulate(psnr_dehazed_vec.begin(), psnr_dehazed_vec.end(), 0.0) / psnr_dehazed_vec.size();
  double avg_h_ssim = std::accumulate(ssim_hazy_vec.begin(), ssim_hazy_vec.end(), 0.0) / ssim_hazy_vec.size();
  double avg_d_ssim = std::accumulate(ssim_dehazed_vec.begin(), ssim_dehazed_vec.end(), 0.0) / ssim_dehazed_vec.size();
  double avg_time = std::accumulate(time_vec.begin(), time_vec.end(), 0.0) / time_vec.size();

  std::cout << std::string(85, '=') << std::endl;
  std::cout << "SUMMARY (" << total_to_eval << " images evaluated):" << std::endl;
  std::cout << "  - Average Input Hazy PSNR:   " << std::fixed << std::setprecision(2) << avg_h_psnr << " dB" << std::endl;
  std::cout << "  - Average Dehazed PSNR:      " << std::fixed << std::setprecision(2) << avg_d_psnr << " dB (Delta: " 
            << std::showpos << (avg_d_psnr - avg_h_psnr) << std::noshowpos << " dB)" << std::endl;
  std::cout << "  - Average Input Hazy SSIM:   " << std::fixed << std::setprecision(4) << avg_h_ssim << std::endl;
  std::cout << "  - Average Dehazed SSIM:      " << std::fixed << std::setprecision(4) << avg_d_ssim << " (Delta: " 
            << std::showpos << (avg_d_ssim - avg_h_ssim) << std::noshowpos << ")" << std::endl;
  std::cout << "  - Average Processing Time:   " << std::fixed << std::setprecision(2) << avg_time << " ms (" 
            << (1000.0 / avg_time) << " FPS at 1600x1200)" << std::endl;
  std::cout << "  - Results saved in:          " << out_dir << std::endl;
  std::cout << std::string(85, '=') << std::endl;

  return 0;
}
