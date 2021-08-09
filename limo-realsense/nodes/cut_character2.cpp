
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#define V_PROJECT 1
#define H_PROJECT 2

typedef struct {
  int begin;
  int end;
} char_range;

void draw(std::vector<int>& pos, int mode) {
  std::vector<int>::iterator max = std::max_element(begin(pos), end(pos));
  if (mode == V_PROJECT) {
    int height = *max;
    int width = pos.size();
    cv::Mat project(height, width, CV_8UC1, cv::Scalar(0, 0, 0));

    cv::imshow("vertical", project);
  } else if (mode == H_PROJECT) {
  }
}
int GetPeekRange(std::vector<int>& vertical_pos,
                 std::vector<char_range>& peek_range, int min_thresh = 2,
                 int min_range = 10) {
  return 0;
}
int GetTextProject(cv::Mat& src, std::vector<int>& pos, int mode) {
  if (mode == V_PROJECT) {
    for (int i = 0; i < src.rows; i++) {
      for (int j = 0; j < src.cols; j++) {
        if (src.at<uchar>(i, j) == 0) {
          pos[j]++;
        }
      }
    }
    draw(pos, mode);
  } else if (mode == H_PROJECT) {
    for (int i = 0; i < src.cols; i++) {
      for (int j = 0; j < src.rows; j++) {
        if (src.at<uchar>(j, i) == 0) {
          pos[j]++;
        }
      }
    }
    draw(pos, mode);
  }
  return 0;
}

std::vector<cv::Mat> CutSingleChar(cv::Mat& img) {
  cv::Mat src;
  cv::threshold(src, src, 0, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);
  std::vector<int> horizion_pos(src.rows, 0);
  std::vector<char_range> h_peek_range;
  GetTextProject(src, horizion_pos, H_PROJECT);
  GetPeekRange(horizion_pos, h_peek_range, 2, 10);
}

int main(int argc, char* argv[]) {
  cv::Mat src_img = cv::imread(
      "/opt/ros_ws/src/libot/libot-realsense/dataset/temperature.jpeg");
  std::vector<cv::Mat> chars_set = CutSingleChar(src_img);
  cv::imshow("src_img", src_img);
  while (cv::waitKey() != 'q') {
  }
  return 0;
}