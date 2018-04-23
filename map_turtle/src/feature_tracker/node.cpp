#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <opencv2/opencv.hpp>
#include <stdio.h>
#include <string>

#include "feature_tracker/feature_tracker.h"
#include "feature_tracker/estimator.h"

static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  ros::Subscriber pos_sub_;
  boat_landing::FeatureTracker feat_track_;
  cv::Mat prev_gray_;
  cv::Mat img_gray_;
  cv::Mat img_out_;
  vector<Point2f> points_;
  vector<int> point_idxs_;

  boat_landing::Estimator est_;

  int num_points_ = 0;
  int max_points_ = 20;
  double yaw_angle_ = 0.0;

  // To write video
  cv::VideoWriter writer_;

public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/camera/rgb/image_raw", 1,
      &ImageConverter::imageCb, this);
    pos_sub_ = nh_.subscribe("/NED/vrpn_client/Turtlebot/pose", 1, &ImageConverter::poseCallback, this);

    cv::namedWindow(OPENCV_WINDOW);

    // To write video
    //int fourcc = writer_.fourcc('M','J','P','G');
    writer_.open("feat_track.avi", CV_FOURCC('M','J','P','G'), 30.0, cv::Size(640,480), true);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
    writer_.release();
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // Convert image to gray
    cv::cvtColor(cv_ptr->image, img_gray_, cv::COLOR_BGR2GRAY);

    // If we need new points, get them, else track our existing points
    if (prev_gray_.empty() || points_.size() == 0)
    {
      if (num_points_ < max_points_)
      {
        // get new features
        feat_track_.detectNewFeatures(img_gray_, points_);

        // clear vector and put in idxs for new points
        point_idxs_.clear();
        for (int i = 0; i < points_.size(); i++)
        {
          num_points_++;
          point_idxs_.push_back(num_points_);
        }
      }
      else
      {
        //cout << "Filled max num points" << endl;
      }
    }
    else
    {
      // use optical flow to track features to next frame
      feat_track_.trackFeatures(prev_gray_, points_, img_gray_, points_, point_idxs_);
    }

    // Reacquire features
    // Loop through all feature #'s
    for (int i = 1; i < (max_points_ + 1); i++)
    {
      // Check if feature i is currently be tracked
      bool tracked = false;
      for (int j = 0; j < point_idxs_.size(); j++)
      {
        if (i == point_idxs_[j])
        {
          tracked = true;
          break;
        }
      }

      // If we are already tracking feature i, move on to the next one
      if (tracked)
        continue;

      // If feat i is not tracked, get estimated pixel
      Point2f est_pix = est_.getPixEstimate(i, yaw_angle_);

      // If feat not initialized, continue to next one
      if ((est_pix.x == 0) && (est_pix.y == 0))
        continue;

      // If estimate is inside of our buffer, reacquire feat
      if ((est_pix.x > 150) && (est_pix.x < (640-150)) && (est_pix.y > 150) && (est_pix.y < (480-150)))
      {
        // Try to reacquire feature
        Point2f est_point(est_pix.x, est_pix.y);
        Point2f actual_point;
        bool good_recover = feat_track_.recoverFeature(i, img_gray_, est_point, actual_point);

        if (good_recover)
        {
          // Draw rectangle at found location
          cv::rectangle(cv_ptr->image, Point(actual_point.x - 25, actual_point.y - 25), Point(actual_point.x + 25, actual_point.y + 25), Scalar(255, 0, 0), 3);

          cout << "Recovered feature # " << i << endl;

          // Put feature back into arrays if good recovery
          point_idxs_.push_back(i);
          points_.push_back(actual_point);
        }
      }
    }

    // Save templates
    feat_track_.saveTemplates(img_gray_, points_, point_idxs_);

    // Draw points on image
    for (int i = 0; i < points_.size(); i++)
    {
      // Draw circle at feature pixels
      circle(cv_ptr->image, points_[i], 5, Scalar(255, 0, 0), 3);

      // Label feature with index number
      stringstream ss;
      ss << point_idxs_[i];
      putText(cv_ptr->image, ss.str(), points_[i], 1, 2, Scalar(0, 0, 255), 2, 0);

      // Update estimated location of feature
      est_.update(point_idxs_[i], points_[i], yaw_angle_);
    }

    // Show image
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);

    // Write image to video
    writer_.write(cv_ptr->image);
    cv::waitKey(3);

    // Save off gray image for next iteration
    cv::swap(img_gray_, prev_gray_);
  }

  void poseCallback(const geometry_msgs::PoseStampedConstPtr& msg)
  {
    // Get Euler yaw angle for Pose message
    tf::Pose pose;
    tf::poseMsgToTF(msg->pose, pose);
    yaw_angle_ = tf::getYaw(pose.getRotation());
  }
};

int main(int argc, char** argv)
{
  // Initialize ros node
  ros::init(argc, argv, "image_converter");

  // Instatiate object
  ImageConverter ic;

  // Let ROS do its thing
  ros::spin();

  return 0;
}
