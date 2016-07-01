#ifndef SHIP_DETECTOR_H_
#define SHIP_DETECTOR_H_

#include <array>
#include <vector>

#include <opencv2/core.hpp>

//#include "Circle.h"
#include "MapNode.h"

namespace detection {

const int kNumShips = 3;

typedef std::array<int, kNumShips> Candidates;
typedef std::vector<MapNode> MapNodes;
//typedef std::vector<geometry::Circle> MapNodes;
typedef std::array<cv::Vec3b, kNumShips> CandidatesColors;

class ShipDetector {
 public:
  ShipDetector() {}
  ~ShipDetector() {}

  void init(const cv::Mat &mapTemplate);
  cv::Mat thresholdImage(const cv::Mat &frame);
  int findShipsBlobs(const std::vector<std::vector<cv::Point2i>> &contours,
                     const cv::Mat &frame, const cv::Mat &binImage, Candidates *ships);

  //cv::Rect getMapROI() const { return this->mapROI; }
  cv::Mat getHomography() const { return this->H; }
  MapNodes getNodes() const { return this->nodes; }

  void SetParametersAt(int index, const cv::Scalar &lower, const cv::Scalar &upper);

 private:
  void tryInsertBlob(const std::vector<std::vector<cv::Point2i>> &contours, size_t blob,
                     Candidates *ships, size_t ship);
  void tryInsertBlob(const std::vector<std::vector<cv::Point2i>> &contours, int blob,
                     Candidates *ships, int begin, int end, int *numShipsDetected);

  cv::Mat mapTemplate;
  cv::Mat H;
  MapNodes nodes;
  std::array<std::vector<cv::Scalar>, 3> thresholdBounds;
  /*std::vector<cv::Point2f> srcPoints;
  std::vector<cv::Point2f> dstPoints;*/
};

}  // namespace detection

#endif  // SHIP_DETECTOR_H_
