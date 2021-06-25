#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/ml.hpp>
#include <opencv2/ml/ml.hpp>
#include <string>

#include <fstream>

// https://www.programmersought.com/article/88904965655/

std::string img_path = "/home/qie/Pictures/a.png";

std::vector<std::vector<cv::String>> GetImgPath(std::string s, int& num) {
  std::vector<std::vector<cv::String>> all_path;

  std::ifstream path(s);
  std::vector<std::string> ipath_num;
  std::string buf;

  while (path) {
    if (getline(path, buf)) {
      ipath_num.push_back(buf);
    }
  }
  path.close();

  for (size_t i = 0; i < ipath_num.size(); i++) {
    std::string pattern_img = ipath_num[i];
    std::vector<cv::String> files;
    cv::glob(pattern_img, files, false);
    if (files.size() == 0) {
      std::cout << "no image [png]" << std::endl;
    }
    num += files.size();
    all_path.push_back(files);
  }

  return all_path;
}

void FilterImage(cv::Mat& img) {
  for (int i = 0; i < img.rows; i++) {
    for (int j = 0; j < img.cols; j++) {
      // img.at<uint8_t>(i,j) = 255;
      // img.at<uint8_t>(i,j+1) = 255;
      // std::cout << std::hex <<  img.at<uint8_t>(i,j) <<" " <<
      // img.at<uint8_t>(i,j+1) << " " << img.at<uint8_t>(i,j+2) << std::endl;
      // printf("%02x %02x %02x\n", img.at<uint8_t>(i,j),
      // img.at<uint8_t>(i,j+1), img.at<uint8_t>(i,j+2));

      img.at<uint8_t>(i, j) = 255 - img.at<uint8_t>(i, j);
      if (img.at<uint8_t>(i, j) < 20) {
        img.at<uint8_t>(i, j) = 0;
      }
      if (img.at<uint8_t>(i, j) > 100) {
        img.at<uint8_t>(i, j) = 255;
      }
    }
  }
}
int main(int argc, char* argv[]) {
  auto img = cv::imread(img_path, CV_LOAD_IMAGE_GRAYSCALE);
  // auto img = cv::imread(img_path);

  std::vector<std::vector<cv::String>> img_from_dir;
  int img_num = 0;
  img_from_dir = GetImgPath(
      "/opt/ros_ws/src/libot/libot-realsense/PathForTrain.txt", img_num);
  std::cout << "img_num : " << img_num << std::endl;

  //图片预处理
  int height = 100;
  int width = 100;
  cv::Mat data_mat = cv::Mat(img_num, height * width, CV_8UC1);
  cv::Mat labels_mat = cv::Mat(img_num, 1, CV_32SC1);
  int index = 0;
  for (size_t i = 0; i < img_from_dir.size(); i++) {
    for (size_t j = 0; j < img_from_dir[i].size(); j++) {
      cv::Mat image0 = cv::imread(img_from_dir[i][j], 0);
      cv::resize(image0, image0, cv::Size(width, height));

      if (image0.empty()) {
        std::cout << "can not load image: " << img_from_dir[i][j] << std::endl;
        continue;
      }

      FilterImage(image0);

      cv::Mat reshaped = cv::Mat(1, height * width, CV_32SC1);
      reshaped = image0.reshape(0, 1);
      reshaped.row(0).copyTo(data_mat.row(index));
      labels_mat.at<int>(index, 0) = i;
      index++;
      //   cv::imshow("img", image0);
      //   cv::waitKey(0);
    }
  }

  // pca特征提取
  int K = img_num * 0.5;
  cv::PCA pca(data_mat, cv::Mat(), cv::PCA::DATA_AS_ROW, K);

  //   cv::Mat mean = pca.mean.clone();
  //   cv::Mat eigenvalues = pca.eigenvalues.clone();
  //   cv::Mat eigenvectors = pca.eigenvectors.clone();

  cv::Mat dst = pca.project(data_mat);

  //保存PCA模型
  cv::FileStorage fs("PCA.xml", cv::FileStorage::WRITE);
  pca.write(fs);
  fs.release();

  cv::Mat project_mat = pca.project(data_mat);

  // svm分类参数
  cv::Ptr<cv::ml::SVM> svm_params = cv::ml::SVM::create();
  svm_params->setType(cv::ml::SVM::C_SVC);
  svm_params->setKernel(cv::ml::SVM::LINEAR);
  svm_params->setC(2.0);
  svm_params->setTermCriteria(cv::TermCriteria(
      cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS, 1000, 0.01));

  // svm分类训练
  std::cout << project_mat.size() << ", " << labels_mat.size() << std::endl;
  std::cout << project_mat.rows << " , " << project_mat.cols << std::endl;

  svm_params->train(project_mat, cv::ml::ROW_SAMPLE, labels_mat);
  svm_params->save("PCA_SVM.xml");

  // cv::imshow("img", eigenvectors);
  // cv::imshow("img", eigenvectors.reshape(1, img.rows));
  // cv::imshow("img", img);
  // cv::imshow("img", project_mat);
  // cv::waitKey(0);

  // test
  char result[512];
  std::vector<std::vector<cv::String>> img_test_path;
  int test_num = 0;
  int wrong_num = 0;
  img_test_path = GetImgPath(
      "/opt/ros_ws/src/libot/libot-realsense/PathForTest.txt", test_num);
  std::cout << "test num: " << test_num << std::endl;

  std::ofstream predict_txt("SVM_PREDICT.txt");
  for(size_t i=0; i!= img_test_path.size(); i++){
for(size_t j=0; j<img_test_path[i].size(); j++){
    cv::Mat img_test = cv::imread(img_test_path[i][j], 0);
    if(img_test.empty()){
        std::cout << "can not load image: " <<img_test_path[i][j] << std::endl;
        continue;
    }
     cv::resize(img_test,img_test, cv::Size(width, height));
    FilterImage(img_test);

    cv::Mat test = img_test.reshape(0,1);
    cv::Mat test_num(1,K,CV_32FC1);
    pca.project(test,test_num);
    int ret = svm_params->predict(test_num);
    if(ret != i-1){
        wrong_num++;
    }
    std::cout <<"predict ret: " << ret<<std::endl;
}
  }

  return 0;
}