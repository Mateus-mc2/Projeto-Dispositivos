#include "ShipDetector.h"

#include <algorithm>
#include <cassert>
#include <fstream>
#include <iostream>
#include <vector>

#include <opencv2/imgproc.hpp>

namespace detection {
namespace {

static std::string map_settings = "../../../data/map_homography.txt";
static std::string external_nodes_settings = "../../../data/map_ext_nodes.txt";
static std::string inner_nodes_settings = "../../../data/map_inner_nodes.txt";

cv::Scalar thresholdInterval(30, 30, 30), color(200, 180, 160);
cv::Scalar min = color - thresholdInterval;
cv::Scalar max = color + thresholdInterval;

bool operator >=(const cv::Scalar &a, const cv::Scalar &b) {
  return a(0) >= b(0) && a(1) >= b(1) && a(2) >= b(2);
}

const int kHueInterval = 10;
const int kPlayerOneHue = 60;
const int kPlayerTwoHue = 0;
const int kPlayerThreeHue = 240;

cv::Mat ThresholdPlayerShip(const cv::Mat &frame, int player) {
  cv::Scalar lowerBound, upperBound;

  switch (player) {
    case 1:
      lowerBound = cv::Scalar(kPlayerOneHue - kHueInterval, 50, 50);
      upperBound = cv::Scalar(kPlayerOneHue + kHueInterval, 255, 255);
      break;

    case 2:
      lowerBound = cv::Scalar(kPlayerTwoHue - kHueInterval, 50, 50);
      upperBound = cv::Scalar(kPlayerTwoHue + kHueInterval, 255, 255);
      break;

    case 3:
      upperBound = cv::Scalar(kPlayerThreeHue + kHueInterval, 50, 50);
      lowerBound = cv::Scalar(kPlayerThreeHue - kHueInterval, 255, 255);
      break;

    default:
      lowerBound = cv::Scalar(0, 0, 0);
      upperBound = cv::Scalar(255, 255, 255);
      break;
  }

  cv::Mat mask;
  cv::inRange(frame, lowerBound, upperBound, mask);

  return mask;
}

}

void ShipDetector::init(const cv::Mat &mapTemplate) {
  // Set map dimensions on camera.
  std::ifstream fileStream(map_settings.c_str());
  std::array<char, 256> line;
  std::vector<float> entries;

  while (!fileStream.eof()) {
    fileStream.getline(line.data(), line.size());
    std::istringstream iss(std::string(line.data()));
    std::vector<std::string> tokens;
    std::copy(std::istream_iterator<std::string>(iss), std::istream_iterator<std::string>(),
              std::back_inserter(tokens));

    for (int i = 0; i < tokens.size(); ++i)
      entries.push_back(std::stof(tokens[i]));
  }

  fileStream.close();
  fileStream.clear();
  this->H = (cv::Mat_<float>(3, 3) << entries[0], entries[1], entries[2],
                                      entries[3], entries[4], entries[5],
                                      entries[6], entries[7], entries[8]);
  cv::warpPerspective(mapTemplate, this->mapTemplate, H.inv(), mapTemplate.size());

  // Set external nodes and inner nodes info.
  std::vector<std::vector<cv::Point2i>> polygons(10);
  fileStream.open(external_nodes_settings.c_str());

  while (!fileStream.eof()) {
    fileStream.getline(line.data(), line.size());
    std::istringstream iss(std::string(line.data()));
    std::vector<std::string> tokens;
    std::copy(std::istream_iterator<std::string>(iss), std::istream_iterator<std::string>(),
              std::back_inserter(tokens));

    int idx = std::stoi(tokens[0]);
    for (int i = 1; i < tokens.size(); ++i) {
      int mid = tokens[i].find(",");
      int x = std::stoi(tokens[i].substr(1, tokens[i].size() - mid));
      int y = std::stoi(tokens[i].substr(mid + 1, tokens[i].size() - 1));

      polygons[idx].push_back(cv::Point2i(x, y));
    }
  }

  fileStream.close();
  fileStream.clear();

  for (int i = 0; i < polygons.size(); ++i) {
    MapNode node(polygons[i], std::vector<geometry::Circle>());
    //node.drawExternalNode(&this->mapTemplate);
    this->nodes.push_back(node);
  }
}

cv::Mat ShipDetector::thresholdImage(const cv::Mat &frame, int threshold) {
  // TODO(Mateus): test detection with homography later.
  //cv::Mat diff = frame - this->mapTemplate;
  //cv::blur(diff, diff, cv::Size(7, 7));
  //cv::medianBlur(diff, diff, 7);

  cv::Mat mask1, mask2, mask3;

  mask1 = ThresholdPlayerShip(frame, 1);
  mask2 = ThresholdPlayerShip(frame, 2);
  mask3 = ThresholdPlayerShip(frame, 3);

  cv::Mat result = mask1 | mask2 | mask3;

  /*cv::cvtColor(diff, diff, CV_BGR2GRAY);
  cv::threshold(diff, diff, threshold, 255, CV_THRESH_BINARY);*/
  //return diff;

  return result;
}

int ShipDetector::findShipsBlobs(const std::vector<std::vector<cv::Point2i>> &contours,
                                 const cv::Mat &binImage, Candidates *ships) {
  ships->fill(-1);
  int numShipsDetected = 0;

  for (size_t i = 0; i < contours.size(); ++i) {
    cv::Rect bounds = cv::boundingRect(contours[i]);
    cv::Scalar mean = cv::mean(this->mapTemplate(bounds), binImage(bounds) != 0);

    if (mean >= min && max >= mean) {
      this->tryInsertBlob(contours, static_cast<int>(i), ships, 0, kNumShips - 1, &numShipsDetected);
    }
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
