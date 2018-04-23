#ifndef ESTIMATOR_H
#define ESTIMATOR_H

#include <iostream>
#include <opencv2/opencv.hpp>

#include <cmath>

using namespace std;
using namespace cv;

namespace boat_landing {

class Estimator {

public:
  Estimator();
  ~Estimator();

  void update(int idx, Point2f point, double yaw);
  Point2f getEstimate(int idx);
  Point2f getPixEstimate(int idx, double yaw);

private:
  std::map<int, Point2f> estimates_;
  double fx_, fy_, cx_, cy_;
  double lpf_;
  cv::Mat K_;
  cv::Mat dist_coeffs_;

};

} // end namespace boat_landing

#endif
