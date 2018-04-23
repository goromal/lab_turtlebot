#include "feature_tracker/feature_tracker.h"

using namespace std;
using namespace cv;

namespace boat_landing {

FeatureTracker::FeatureTracker()
{
  // Set width of templates for features
  template_width_ = 50;

  // Create mask to only search interior of image
  mask_ = cv::Mat::zeros(480, 640, CV_8UC1);
  rectangle(mask_, Point(template_width_, template_width_), Point(mask_.cols - template_width_,mask_.rows - template_width_), Scalar(255, 255, 255), CV_FILLED);
}

FeatureTracker::~FeatureTracker()
{}

void FeatureTracker::detectNewFeatures(Mat& img, vector<Point2f>& points)
{
  // Return nothing if no image
  if (img.empty())
    return;

  // Detect features
  int max_corners_ = 20;
  double quality_level = 0.10;
  double min_distance = 150;
  goodFeaturesToTrack(img, points, max_corners_, quality_level, min_distance, mask_, 3, true, 0.04 );
}

void FeatureTracker::trackFeatures(Mat& img1, vector<Point2f> points1, Mat& img2, vector<Point2f>& points2, vector<int>& idxs)
{
  // Calculate optical flow of points1
  vector<uchar> status;
  vector<float> err;
  calcOpticalFlowPyrLK(img1, img2, points1, points2, status, err);

  // Get rid of failed points or outside of frame
  int idx = 0;
  for (int i = 0; i < status.size(); i++)
  {
    // Grab current point
    Point2f pt;
    pt = points2.at(i - idx);

    // If OF failed or outside of our buffer
    if ((status.at(i) == 0) || (pt.x<template_width_) || (pt.y<template_width_) || (pt.x>(mask_.cols - template_width_)) || (pt.y>(mask_.rows - template_width_)))
    {
      // Erase corresponding point and index from tracked features
      points2.erase(points2.begin() + (i - idx));
      idxs.erase(idxs.begin() + (i - idx));
      idx++;
    }
  }
}

void FeatureTracker::saveTemplates(Mat& img, vector<Point2f>& points, vector<int>& idxs)
{
  // If a point is sufficiently close to the border, save off a template for it
  // Loop through points
  for (int i = 0; i < points.size(); i++)
  {
    // grab current point
    Point2f pt;
    pt = points[i];

    // If point is close to border
    if ( (pt.x < template_width_*2) || (pt.y < template_width_*2) || (pt.x > (mask_.cols - 2*template_width_)) || (pt.y > (mask_.rows - 2*template_width_)) )
    {
      // Make template around feature point
      Mat templ;
      img(Range(pt.y - template_width_/2, pt.y + template_width_/2), Range(pt.x - template_width_/2, pt.x + template_width_/2)).copyTo(templ);

      // Save template to corresponding index
      templates_[idxs[i]] = templ;
    }
  }
  
}

bool FeatureTracker::recoverFeature(int idx, Mat& img, Point2f& estimate, Point2f& recovered)
{
  // Grab template for index
  Mat curr_templ = templates_[idx];

  // TODO Create mask for search region
  //Mat templ_mask = cv::Mat::zeros(480, 640, CV_8UC1);
  //rectangle(templ_mask, Point(estimate.x - 3*template_width_, estimate.y - 3*template_width_), Point(estimate.x + 3*template_width_, estimate.y + 3*template_width_), Scalar(255, 255, 255), CV_FILLED);

  // Create mat of search area
  Mat search_area;
  img(Range(estimate.y - 2*template_width_, estimate.y + 2*template_width_), Range(estimate.x - 2*template_width_, estimate.x + 2*template_width_)).copyTo(search_area);

  // Do Matching and normalize
  Mat result;
  matchTemplate(search_area, curr_templ, result, CV_TM_SQDIFF, cv::noArray());
  normalize(result, result, 0, 1, NORM_MINMAX, -1, Mat() );

  // Localizing the best match with minMaxLoc
  double maxVal; Point minLoc; Point maxLoc;
  double minVal;
  Point2f loc;
  minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc, Mat());

  // Get recovered feature point
  loc = minLoc;
  recovered.x = estimate.x - 2*template_width_ + loc.x + template_width_/2;
  recovered.y = estimate.y - 2*template_width_ + loc.y + template_width_/2;

  // Check our match is valid
  double minval_thresh = 1e-10;

  if (minVal < minval_thresh)
  {
    if ((recovered.x > 50) && (recovered.y > 50) && (recovered.x < (640 - 50)) && (recovered.y < (480 - 50)))
    {}
    else
      return false;
  }
  else
  {
    //cout << "Threshold not met!!!!!!!!!!!" << endl;
    return false;
  }

  // Make image to draw on
  Mat img2;
  cvtColor(img, img2, COLOR_GRAY2BGR);

  // Draw search region
  cv::rectangle(img2, Point(estimate.x - 2*template_width_, estimate.y - 2*template_width_), Point(estimate.x + 2*template_width_, estimate.y + 2*template_width_), Scalar(0, 255, 0), 3);

  // Draw estimated pixel location
  cv::circle(img2, estimate, 5, Scalar(0, 255, 0), 3);

  // Draw rectangle and circle for found location
  cv::rectangle(img2, Point(recovered.x - template_width_/2, recovered.y - template_width_/2), Point(recovered.x + template_width_/2, recovered.y + template_width_/2), Scalar(255, 0, 0), 3);
  cv::circle(img2, recovered, 5, Scalar(255, 0, 0), 3);

  // Dispay images
  imshow("image", img2);
  imshow("template", curr_templ);
  imshow("result", result);
  imshow("search_area", search_area);
  //waitKey(0);

  return true;
}

} // end namespace
