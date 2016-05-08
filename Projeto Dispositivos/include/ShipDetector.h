#ifndef SHIP_DETECTOR_H_
#define SHIP_DETECTOR_H_

#include <vector>

#include <opencv2/core.hpp>

namespace detection {

class ShipDetector {
 public:
  ShipDetector() : srcPoints(4), dstPoints(4) {}
  ~ShipDetector() {}

  void init(const cv::Mat &mapTemplate);
  cv::Mat detectShip(const cv::Mat &frame);
 private:
  cv::Mat mapTemplate;
  std::vector<cv::Point2f> srcPoints;
  std::vector<cv::Point2f> dstPoints;
};

}  // namespace detection

#endif  // SHIP_DETECTOR_H_
