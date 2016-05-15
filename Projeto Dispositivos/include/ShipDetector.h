#ifndef SHIP_DETECTOR_H_
#define SHIP_DETECTOR_H_

#include <array>
#include <vector>

#include <opencv2/core.hpp>

namespace detection {

const int kNumShips = 1;

typedef std::array<int, kNumShips> Candidates;

class ShipDetector {
 public:
  ShipDetector() : srcPoints(4), dstPoints(4) {}
  ~ShipDetector() {}

  void init(const cv::Mat &mapTemplate);
  cv::Mat thresholdImage(const cv::Mat &frame, int threshold);
  int findShipsBlobs(const std::vector<std::vector<cv::Point2i>> &contours,
                     Candidates *ships);

 private:
  void tryInsertBlob(const std::vector<std::vector<cv::Point2i>> &contours, int blob,
                     Candidates *ships, int begin, int end, int *numShipsDetected);

  cv::Mat mapTemplate;
  std::vector<cv::Point2f> srcPoints;
  std::vector<cv::Point2f> dstPoints;
};

}  // namespace detection

#endif  // SHIP_DETECTOR_H_
