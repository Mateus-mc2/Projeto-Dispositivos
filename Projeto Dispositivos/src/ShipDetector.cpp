#include "ShipDetector.h"

#include <cassert>
#include <vector>

#include <opencv2/imgproc.hpp>

namespace detection {

void ShipDetector::init(const cv::Mat &mapTemplate) {
  this->mapTemplate = mapTemplate;

  this->srcPoints[0] = cv::Point2f(0.0, 0.0);
  this->srcPoints[1] = cv::Point2f(mapTemplate.rows-1, 0.0);
  this->srcPoints[2] = cv::Point2f(0.0, mapTemplate.cols-1);
  this->srcPoints[3] = cv::Point2f(mapTemplate.rows-1, mapTemplate.cols-1);
}

cv::Mat ShipDetector::detectShip(const cv::Mat &frame) {
  // TODO(Mateus): test detection with homography later.
  cv::Mat diff = frame - this->mapTemplate;
  const int channels = diff.channels();

  for (int i = 0; i < diff.rows; ++i) {
    uchar *pixel = diff.ptr<uchar>(i);

    for (int j = 0; j < channels*diff.cols; ++j) {
      *pixel = *pixel > 0 ? 255 : 0;
    }
  }

  cv::cvtColor(diff, diff, CV_BGR2GRAY);
  for (int i = 0; i < diff.rows*diff.cols; ++i) {
    diff.data[i] = diff.data[i] > 20 ? 255 : 0;
  }

  return diff;
}

}  // namespace detection
