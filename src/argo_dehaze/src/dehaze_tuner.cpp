#include "argo_dehaze/adaptive_dcp.hpp"
#include <iostream>
#include <iomanip>
#include <filesystem>
#include <vector>
#include <fstream>

namespace fs = std::filesystem;

// Global state for trackbars
int g_awb_pct = 75;        // 0.75
int g_omega_pct = 90;      // 0.90
int g_t0_x100 = 22;        // 0.22
int g_guided_r = 16;       // 16
int g_target_lum = 115;    // 115.0
int g_clahe_x10 = 18;      // 1.8
int g_neutral_a = 50;      // 0.50

bool g_needs_update = true;

void onTrackbar(int, void*) {
  g_needs_update = true;
}

void saveConfig(const std::string& filepath) {
  std::ofstream ofs(filepath);
  if (!ofs.is_open()) {
    std::cerr << "Failed to open config file for saving: " << filepath << std::endl;
    return;
  }
  ofs << "/**:\n"
      << "  ros__parameters:\n"
      << "    in_image_topic: \"/camera/image_raw\"\n"
      << "    out_image_topic: \"/camera/image_dehazed\"\n"
      << "    patch_size: 15\n"
      << "    omega_min: 0.65\n"
      << "    omega_max: " << (g_omega_pct / 100.0) << "\n"
      << "    t0_base: " << (g_t0_x100 / 100.0) << "\n"
      << "    guided_radius: " << g_guided_r << "\n"
      << "    guided_eps: 0.001\n"
      << "    guided_subsample: 4\n"
      << "    target_luminance: " << (double)g_target_lum << "\n"
      << "    enable_auto_white_balance: true\n"
      << "    awb_strength: " << (g_awb_pct / 100.0) << "\n"
      << "    neutral_atm_ratio: " << (g_neutral_a / 100.0) << "\n"
      << "    enable_adaptive_tone: true\n"
      << "    enable_clahe: true\n"
      << "    clahe_clip_limit: " << (g_clahe_x10 / 10.0) << "\n";
  ofs.close();
  std::cout << "\n>>> [SAVED] Current slider parameters successfully saved to: " << filepath << std::endl;
}

int main(int argc, char** argv) {
  std::string dataset_dir = "/root/zenith_ws/Dense-Haze CVPR 2019";
  std::string config_path = "/root/zenith_ws/src/argo_dehaze/config/dehaze_params.yaml";

  if (argc > 1) dataset_dir = argv[1];

  std::string hazy_dir = dataset_dir + "/hazy";
  std::string gt_dir = dataset_dir + "/GT";

  std::vector<fs::path> hazy_files;
  for (const auto& entry : fs::directory_iterator(hazy_dir)) {
    if (entry.path().extension() == ".png" || entry.path().extension() == ".jpg") {
      hazy_files.push_back(entry.path());
    }
  }
  std::sort(hazy_files.begin(), hazy_files.end());

  if (hazy_files.empty()) {
    std::cerr << "No images found in: " << hazy_dir << std::endl;
    return 1;
  }

  const std::string win_name = "Zenith Dehaze Tuner (Sliders: AWB, Omega, t0, Lum, CLAHE)";
  cv::namedWindow(win_name, cv::WINDOW_NORMAL);
  cv::resizeWindow(win_name, 1400, 700);

  // Create trackbars
  cv::createTrackbar("AWB (0~100)", win_name, &g_awb_pct, 100, onTrackbar);
  cv::createTrackbar("Omega (30~100)", win_name, &g_omega_pct, 100, onTrackbar);
  cv::createTrackbar("t0 Lower (x100)", win_name, &g_t0_x100, 50, onTrackbar);
  cv::createTrackbar("Guided R (4~40)", win_name, &g_guided_r, 40, onTrackbar);
  cv::createTrackbar("Target Lum", win_name, &g_target_lum, 180, onTrackbar);
  cv::createTrackbar("CLAHE Clip (x10)", win_name, &g_clahe_x10, 40, onTrackbar);
  cv::createTrackbar("Neutral A (0~100)", win_name, &g_neutral_a, 100, onTrackbar);

  size_t cur_idx = 0;
  cv::Mat orig_hazy, orig_gt;
  cv::Mat hazy_cam, gt_cam;

  auto loadImages = [&](size_t idx) {
    fs::path h_path = hazy_files[idx];
    std::string stem = h_path.stem().string();
    std::string prefix = stem.substr(0, stem.find("_hazy"));
    fs::path g_path = fs::path(gt_dir) / (prefix + "_GT.png");

    orig_hazy = cv::imread(h_path.string());
    orig_gt = cv::imread(g_path.string());

    // Scale to Hikrobot Camera Resolution: 640x512
    cv::resize(orig_hazy, hazy_cam, cv::Size(640, 512));
    if (!orig_gt.empty()) {
      cv::resize(orig_gt, gt_cam, cv::Size(640, 512));
    } else {
      gt_cam = cv::Mat::zeros(512, 640, CV_8UC3);
    }
    g_needs_update = true;
    std::cout << "\n[Loaded Image " << (idx + 1) << "/" << hazy_files.size() << "] " << h_path.filename().string() << std::endl;
  };

  loadImages(cur_idx);

  cv::Ptr<cv::ORB> orb = cv::ORB::create(2000);
  std::vector<cv::KeyPoint> kp_hazy;
  orb->detect(hazy_cam, kp_hazy);

  std::cout << "\n=======================================================" << std::endl;
  std::cout << "  Zenith Interactive Dehaze Tuner Ready!" << std::endl;
  std::cout << "  - Drag any slider on the window to tune live" << std::endl;
  std::cout << "  - Press [N] : Next Image (" << hazy_files.size() << " total)" << std::endl;
  std::cout << "  - Press [P] : Previous Image" << std::endl;
  std::cout << "  - Press [S] : SAVE current parameters to dehaze_params.yaml!" << std::endl;
  std::cout << "  - Press [Q] or [ESC] : Quit" << std::endl;
  std::cout << "=======================================================" << std::endl;

  zenith::AdaptiveDCP dehazer;

  while (true) {
    if (g_needs_update) {
      g_needs_update = false;

      // Update parameters
      zenith::AdaptiveDCPParams p;
      p.enable_auto_white_balance = (g_awb_pct > 0);
      p.awb_strength = std::clamp(g_awb_pct / 100.0f, 0.0f, 1.0f);
      p.omega_max = std::clamp(g_omega_pct / 100.0f, 0.3f, 1.0f);
      p.t0_base = std::clamp(g_t0_x100 / 100.0f, 0.05f, 0.6f);
      p.guided_radius = std::max(4, g_guided_r);
      p.target_luminance = std::clamp((float)g_target_lum, 60.0f, 200.0f);
      p.enable_clahe = (g_clahe_x10 > 0);
      p.clahe_clip_limit = std::clamp(g_clahe_x10 / 10.0f, 0.1f, 5.0f);
      p.neutral_atm_ratio = std::clamp(g_neutral_a / 100.0f, 0.0f, 1.0f);

      dehazer.setParams(p);

      zenith::DehazeInfo info;
      cv::Mat dehazed = dehazer.dehaze(hazy_cam, &info);

      double psnr_val = 0.0, ssim_val = 0.0;
      if (!gt_cam.empty()) {
        psnr_val = zenith::AdaptiveDCP::calculatePSNR(dehazed, gt_cam);
        ssim_val = zenith::AdaptiveDCP::calculateSSIM(dehazed, gt_cam);
      }

      std::vector<cv::KeyPoint> kp_dehazed;
      orb->detect(dehazed, kp_dehazed);

      // Concatenate 3 images: [Hazy | Dehazed | GT]
      cv::Mat display;
      cv::hconcat(std::vector<cv::Mat>{hazy_cam, dehazed, gt_cam}, display);

      // Top info bar
      int bar_h = 45;
      cv::Rect bar_rect(0, 0, display.cols, bar_h);
      cv::rectangle(display, bar_rect, cv::Scalar(25, 25, 25), cv::FILLED);

      char info_text[256];
      snprintf(info_text, sizeof(info_text),
               "PSNR: %.2f dB | SSIM: %.4f | ORB Feats: %zu -> %zu (%.0f ms / %.1f FPS) | [S]: Save, [N]/[P]: Image",
               psnr_val, ssim_val, kp_hazy.size(), kp_dehazed.size(), info.elapsed_ms, (1000.0 / std::max(1.0, info.elapsed_ms)));

      cv::putText(display, info_text, cv::Point(20, 28), cv::FONT_HERSHEY_SIMPLEX, 0.58, cv::Scalar(0, 255, 255), 2);

      // Bottom labels
      cv::putText(display, "Hazy Input (640x512)", cv::Point(20, display.rows - 15), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 255), 2);
      cv::putText(display, "Dehazed (Live Slider)", cv::Point(640 + 20, display.rows - 15), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);
      cv::putText(display, "Ground Truth", cv::Point(1280 + 20, display.rows - 15), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 0), 2);

      cv::imshow(win_name, display);
    }

    int key = cv::waitKey(30);
    if (key == 27 || key == 'q' || key == 'Q') {
      break;
    } else if (key == 'n' || key == 'N') {
      cur_idx = (cur_idx + 1) % hazy_files.size();
      loadImages(cur_idx);
      orb->detect(hazy_cam, kp_hazy);
    } else if (key == 'p' || key == 'P') {
      cur_idx = (cur_idx == 0) ? hazy_files.size() - 1 : cur_idx - 1;
      loadImages(cur_idx);
      orb->detect(hazy_cam, kp_hazy);
    } else if (key == 's' || key == 'S') {
      saveConfig(config_path);
    }
  }

  cv::destroyAllWindows();
  return 0;
}
