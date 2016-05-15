#include "ShipDetector.h"

#include <cassert>
#include <vector>

#include <opencv2/imgproc.hpp>

namespace detection {

void ShipDetector::init(const cv::Mat &mapTemplate) {
  this->mapTemplate = mapTemplate;

  this->srcPoints[0] = cv::Point2f(0.0, 0.0);
  this->srcPoints[1] = cv::Point2f(static_cast<float>(mapTemplate.rows - 1), 0.0);
  this->srcPoints[2] = cv::Point2f(0.0, static_cast<float>(mapTemplate.cols - 1));
  this->srcPoints[3] = cv::Point2f(static_cast<float>(mapTemplate.rows - 1),
                                   static_cast<float>(mapTemplate.cols - 1));
}

cv::Mat ShipDetector::thresholdImage(const cv::Mat &frame, int threshold) {
  // TODO(Mateus): test detection with homography later.
  cv::Mat diff = frame - this->mapTemplate;
  cv::cvtColor(diff, diff, CV_BGR2GRAY);
  cv::threshold(diff, diff, threshold, 255, CV_THRESH_BINARY);

  return diff;
}

int ShipDetector::findShipsBlobs(const std::vector<std::vector<cv::Point2i>> &contours,
                                 Candidates *ships) {
  ships->fill(-1);
  int numShipsDetected = 0;

  for (size_t i = 0; i < contours.size(); ++i) {
    this->tryInsertBlob(contours, static_cast<int>(i), ships, 0, kNumShips - 1, &numShipsDetected);
  }

  return numShipsDetected;
}

void ShipDetector::tryInsertBlob(const std::vector<std::vector<cv::Point2i>> &contours, int blob,
                                 Candidates *ships, int begin, int end, int *numShipsDetected) {
  int mid = begin + (end - begin) / 2;
  int currentIndex = (*ships)[mid];
  size_t candidateSize = contours[blob].size();
  size_t currentSize = currentIndex == -1 ? 0 : contours[currentIndex].size();

  if (begin < end) {
    // Non-increasing ordering.
    if (candidateSize > currentSize) {
      this->tryInsertBlob(contours, blob, ships, begin, mid, numShipsDetected);
      return;
    } else if (candidateSize < currentSize) {
      this->tryInsertBlob(contours, blob, ships, mid + 1, end, numShipsDetected);
      return;
    }
  }

  // It's the last index, so it may still be smaller than the previous one.
  if (mid == kNumShips - 1 && candidateSize > currentSize) {
    (*ships)[mid] = blob;

    if (*numShipsDetected < kNumShips)
      ++(*numShipsDetected);
  } else if (mid < kNumShips - 1) {
    memmove(ships->data() + mid + 1, ships->data() + mid,
            sizeof(int)*(kNumShips - mid - 1));
    (*ships)[mid] = blob;

    if (*numShipsDetected < kNumShips)
      ++(*numShipsDetected);
  }
}

}  // namespace detection
