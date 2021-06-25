
// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2019 Intel Corporation. All Rights Reserved.

#include <iostream>              // for cout
#include <librealsense2/rs.hpp>  // Include RealSense Cross Platform API

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <leptonica/allheaders.h>
#include <tesseract/baseapi.h>

// Hello RealSense example demonstrates the basics of connecting to a RealSense
// device and taking advantage of depth data
int main(int argc, char* argv[]) try {
  // Create a Pipeline - this serves as a top-level API for streaming and
  // processing frames
  rs2::pipeline p;

  rs2::config cfg;
  cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
  cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);

  // Configure and start the pipeline
  p.start(cfg);

  tesseract::TessBaseAPI* api = new tesseract::TessBaseAPI();
//   api->Init(NULL, "eng", tesseract::OEM_LSTM_ONLY);
  api->Init(NULL, "chi_sim", tesseract::OEM_LSTM_ONLY);
  api->SetPageSegMode(tesseract::PSM_AUTO);

  while (true) {
    // Block program until frames arrive
    rs2::frameset frames = p.wait_for_frames();

    // Try to get a frame of a depth image
    rs2::depth_frame depth = frames.get_depth_frame();
    rs2::video_frame video = frames.get_color_frame();

    // Get the depth frame's dimensions
    auto width = depth.get_width();
    auto height = depth.get_height();

    // Query the distance from the camera to the object in the center of the
    // image
    float dist_to_center = depth.get_distance(width / 2, height / 2);

    int w = video.get_width();
    int h = video.get_height();

    cv::Mat image(cv::Size(w, h), CV_8UC3, (void*)video.get_data(),
                  cv::Mat::AUTO_STEP);
    // rs2::frameset aligned_set = rs2::align_to.process(frames);
    // auto color_mat = rs2::frame_to_mat(aligned_set.get_color_frame());

    // Print the distance
    // std::cout << "The camera is facing an object " << dist_to_center << "
    // meters away \r";
    cv::imshow("img", image);
    cv::waitKey(1);

    api->SetImage(image.data, image.cols, image.rows, 3, image.step);
    std::string text = std::string(api->GetUTF8Text());
    std::cout << "result: " << text << std::endl;
  }

  api->End();
  return EXIT_SUCCESS;
} catch (const rs2::error& e) {
  std::cerr << "RealSense error calling " << e.get_failed_function() << "("
            << e.get_failed_args() << "):\n    " << e.what() << std::endl;
  return EXIT_FAILURE;
} catch (const std::exception& e) {
  std::cerr << e.what() << std::endl;
  return EXIT_FAILURE;
}
