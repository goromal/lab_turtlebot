#ifndef FEATURE_TRACKER_H
#define FEATURE_TRACKER_H

#include <opencv2/opencv.hpp>
#include <iostream>

using namespace std;
using namespace cv;

namespace boat_landing {

class FeatureTracker {

public:
  FeatureTracker();
  ~FeatureTracker();

  void detectNewFeatures(Mat& img, vector<Point2f>& points);
  void trackFeatures(Mat& img1, vector<Point2f> points1, Mat& img2, vector<Point2f>& points2, vector<int>& idxs);
  void saveTemplates(Mat& img, vector<Point2f>& points, vector<int>& idxs);

  bool recoverFeature(int idx, Mat& img, Point2f& estimate, Point2f& recovered);

private:
  int template_width_;
  cv::Mat mask_;
  std::map<int, Mat> templates_;
};

} // end namespace boat_landing

#endif
