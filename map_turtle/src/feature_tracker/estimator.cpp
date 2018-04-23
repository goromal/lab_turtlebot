#include "feature_tracker/estimator.h"

using namespace std;
using namespace cv;

namespace boat_landing {

Estimator::Estimator()
{
  // Camera intrinsic parameters
  fx_ = 526.634263;
  fy_ = 527.615563;
  cx_ = 327.719899;
  cy_ = 247.717581;

  // camera matrix
  K_ = Mat::zeros(3, 3, CV_64F);
  K_.at<double>(0, 0) = fx_;
  K_.at<double>(1, 1) = fy_;
  K_.at<double>(0, 2) = cx_;
  K_.at<double>(1, 2) = cy_;
  K_.at<double>(2, 2) = 1.0f;

  // Distortion coeffs
  dist_coeffs_ = Mat::zeros(1, 5, CV_64F);
  dist_coeffs_.at<double>(0, 0) = 0.022655;
  dist_coeffs_.at<double>(0, 1) = -0.107507;
  dist_coeffs_.at<double>(0, 2) = 0.000871;
  dist_coeffs_.at<double>(0, 3) = 0.001737;
  dist_coeffs_.at<double>(0, 4) = 0.000000;

  // Low pass filter for estimates
  lpf_ = 0.5;
}

Estimator::~Estimator()
{}

void Estimator::update(int idx, Point2f point, double yaw)
{
  // Undistort point
  vector<Point2f> raw, undistort;
  raw.push_back(point);
  cv::undistortPoints(raw, undistort, K_, dist_coeffs_);

  // Find angle to feature
  double alpha = yaw + atan(undistort[0].x);
  double beta = atan(undistort[0].y);

  Point2f current_est = estimates_[idx];

  if ((current_est.x == 0) && (current_est.y == 0))
  {
    // If not initialized, set equal to first meas
    //cout << "current_est not initialized" << endl;
    Point2f current_meas(alpha, beta);
    estimates_[idx] = current_meas;
  }
  else
  {
    // lpf estimates
    current_est.x = lpf_*current_est.x + (1.0f - lpf_)*alpha;
    current_est.y = lpf_*current_est.y + (1.0f - lpf_)*beta;
    estimates_[idx] = current_est;
  }
}

Point2f Estimator::getEstimate(int idx)
{
  //cout << "get estimate" << endl;
  return estimates_[idx];
}

Point2f Estimator::getPixEstimate(int idx, double yaw)
{
  //cout << "get estimate" << endl;
  double az = estimates_[idx].x - yaw;
  double beta = estimates_[idx].y;

  if ((estimates_[idx].x == 0) && (estimates_[idx].y == 0))
  {
    Point2f zero_p(0.0, 0.0);
    return zero_p;
  }
  else if (abs(az) > 1.4)
  {
    Point2f zero_p(0.0, 0.0);
    return zero_p;
  }

  Point2f est_pix;
  est_pix.x = fx_*tan(az) + cx_;
  est_pix.y = fy_*tan(beta) + cy_;

  return est_pix;
}

} // end namespace boat_landing
