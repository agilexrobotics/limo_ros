
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// https://blog.csdn.net/zhu_hongji/article/details/80443585

class HistogramND {
 private:
  cv::Mat image;
  int his_size[1], his_width, his_height;
  float range[2];
  const float* ranges;
  cv::Mat channel_rgb[3];
  cv::MatND output_rgb[3];

 public:
  HistogramND() {
    his_size[0] = 256;
    his_width = 400;
    his_height = 400;
    range[0] = 0.0;
    range[1] = 255.0;
    ranges = &range[0];
  }

  bool ReadImage(std::string path) {
      image=cv::imread(path);
      if(image.empty()){
          return false;
      }
      return true;
  }

  void SplitChannels(){
      cv::split(image, channel_rgb);
  }

    void GetHistogram(){
        cv::calcHist(&channel_rgb[0], 1,0,cv::Mat(),output_rgb[0],1,his_size, &ranges);
        cv::calcHist(&channel_rgb[1], 1,0,cv::Mat(),output_rgb[1],1,his_size, &ranges);
        cv::calcHist(&channel_rgb[2], 1,0,cv::Mat(),output_rgb[2],1,his_size, &ranges);

        for(int i=0; i<his_size[0]; ++i){

        }
    }
    void DisplayHistogram(){
        cv::Mat rgb_hist[3];
        
    }

};
int main(int argc, char* argv[]) {}