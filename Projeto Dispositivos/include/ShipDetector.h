#ifndef SHIP_DETECTOR_H_
#define SHIP_DETECTOR_H_

#include <array>
#include <vector>

#include <opencv2/core.hpp>

#include "MapNode.h"

namespace detection {

const int kNumShips = 3;

typedef std::array<int, kNumShips> Candidates;
typedef std::vector<MapNode> MapNodes;
typedef std::array<cv::Vec3b, kNumShips> CandidatesColors;

class ShipDetector {
 public:
  ShipDetector() {}
  ~ShipDetector() {}

  void init(const cv::Mat &mapTemplate);
  cv::Mat thresholdImage(const cv::Mat &frame, int threshold);
  int findShipsBlobs(const std::vector<std::vector<cv::Point2i>> &contours,
                     const cv::Mat &binImage, Candidates *ships);

  //cv::Rect getMapROI() const { return this->mapROI; }
  cv::Mat getHomography() const { return this->H; }
  MapNodes getNodes() const { return this->nodes; }

 private:
  void tryInsertBlob(const std::vector<std::vector<cv::Point2i>> &contours, int blob,
                     Candidates *ships, int begin, int end, int *numShipsDetected);

  cv::Mat mapTemplate;
  cv::Mat H;
  MapNodes nodes;
  /*std::vector<cv::Point2f> srcPoints;
  std::vector<cv::Point2f> dstPoints;*/
};

}  // namespace detection

#endif  // SHIP_DETECTOR_H_
