#include "ShipDetector.h"

#include <algorithm>
#include <cassert>
#include <fstream>
#include <iostream>
#include <vector>

#include <opencv2/imgproc.hpp>

namespace detection {
namespace {

static std::string map_settings = "../../../data/map_roi.txt";
static std::string external_nodes_settings = "../../../data/map_ext_nodes.txt";
static std::string inner_nodes_settings = "../../../data/map_inner_nodes.txt";

}

void ShipDetector::init(const cv::Mat &mapTemplate) {
  // Set map dimensions on camera.
  std::ifstream fileStream(map_settings.c_str());
  std::array<char, 256> line;
  cv::Point2i topLeft, bottomRight;

  while (!fileStream.eof()) {
    fileStream.getline(line.data(), line.size());

    if (line[0] != ';') {
      std::istringstream iss(std::string(line.data()));
      std::vector<std::string> tokens;
      std::copy(std::istream_iterator<std::string>(iss), std::istream_iterator<std::string>(),
                std::back_inserter(tokens));

      if (!tokens[0].compare("TL")) {
        topLeft.x = std::stoi(tokens[1]);
        topLeft.y = std::stoi(tokens[2]);
      } else if (!tokens[0].compare("BR")) {
        bottomRight.x = std::stoi(tokens[1]);
        bottomRight.y = std::stoi(tokens[2]);
      }
    }
  }

  fileStream.close();
  this->mapROI = cv::Rect(topLeft, bottomRight);
  this->mapTemplate = mapTemplate(this->mapROI);

  // Set external nodes and inner nodes info.
}

cv::Mat ShipDetector::thresholdImage(const cv::Mat &frame, int threshold) {
  // TODO(Mateus): test detection with homography later.
  cv::Mat diff = frame - this->mapTemplate;
  cv::blur(diff, diff, cv::Size(7, 7));
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
