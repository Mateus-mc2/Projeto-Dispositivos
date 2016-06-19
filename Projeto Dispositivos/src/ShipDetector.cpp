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
//static std::string nodes_settings = "../../../data/map_nodes.txt";
static std::string threshold_settings = "../../../data/player_thresholds.txt";

bool operator >=(const cv::Scalar &a, const cv::Scalar &b) {
  return a(0) >= b(0) && a(1) >= b(1) && a(2) >= b(2);
}

cv::Mat ThresholdPlayerShip(const cv::Mat &frame, const cv::Scalar &lowerBound, const cv::Scalar &upperBound) {
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
  //cv::warpPerspective(mapTemplate, this->mapTemplate, H.inv(), mapTemplate.size());

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
      int mid = static_cast<int>(tokens[i].find(","));
      int x = std::stoi(tokens[i].substr(1, tokens[i].size() - mid));
      int y = std::stoi(tokens[i].substr(mid + 1, tokens[i].size() - 1));

      polygons[idx].push_back(cv::Point2i(x, y));
    }
  }

  fileStream.close();
  fileStream.clear();

  std::vector<geometry::Circle> circles;
  fileStream.open(inner_nodes_settings.c_str());
  //fileStream.open(nodes_settings.c_str());

  while (!fileStream.eof()) {
    fileStream.getline(line.data(), line.size());
    std::istringstream iss(std::string(line.data()));
    std::vector<std::string> tokens;
    std::copy(std::istream_iterator<std::string>(iss), std::istream_iterator<std::string>(),
              std::back_inserter(tokens));

    int idx = std::stoi(tokens[0]);
    float x = std::stof(tokens[1].substr(1, tokens[1].size() - 1));
    float y = std::stof(tokens[2].substr(0, tokens[2].size() - 1));
    int radius = std::stoi(tokens[3]);

    circles.push_back(geometry::Circle(cv::Point2f(x, y), radius, idx));
  }

  fileStream.close();
  fileStream.clear();

  /*for (size_t i = 0; i < circles.size(); ++i) {
    circles[i].drawMapNode(&this->mapTemplate);
    this->nodes.push_back(circles[i]);
  }*/

  for (int i = 0; i < polygons.size(); ++i) {
    MapNode node(polygons[i]);

    for (size_t j = 0; j < circles.size(); ++j) {
      if (node.contains(circles[j].center())) {
        node.pushBackInnerNode(circles[j]);
      }
    }

    //node.drawExternalNode(&this->mapTemplate);
    //node.drawInnerNodes(&this->mapTemplate);
    this->nodes.push_back(node);
  }


  fileStream.open(threshold_settings.c_str());

  while (!fileStream.eof()) {
    fileStream.getline(line.data(), line.size());
    std::istringstream iss(std::string(line.data()));
    std::vector<std::string> tokens;
    std::copy(std::istream_iterator<std::string>(iss), std::istream_iterator<std::string>(),
      std::back_inserter(tokens));

    for (int i = 1; i < tokens.size(); ++i) {
      size_t current;
      size_t next = 0;
      int idx = 0;
      std::array<int, 3> bounds;

      do {
        current = next + 1;
        next = tokens[i].find_first_of(",]", current);
        assert(idx < bounds.size());
        bounds[idx++] = std::stoi(tokens[i].substr(current, next - current));
      } while (next != tokens[i].size() - 1);

      cv::Scalar bound(bounds[0], bounds[1], bounds[2]);

      if (!tokens[0].compare("Y")) {
        this->thresholdBounds[0].push_back(bound);
      } else if (!tokens[0].compare("G")) {
        this->thresholdBounds[1].push_back(bound);
      } else {
        this->thresholdBounds[2].push_back(bound);
      }
    }
  }

  fileStream.close();
  fileStream.clear();
}

cv::Mat ShipDetector::thresholdImage(const cv::Mat &frame) {
  // TODO(Mateus): test detection with homography later.
  cv::Mat diff = frame - this->mapTemplate;
  //cv::blur(diff, diff, cv::Size(7, 7));
  cv::medianBlur(diff, diff, 7);
  cv::cvtColor(diff, diff, CV_HSV2BGR);
  cv::cvtColor(diff, diff, CV_BGR2GRAY);
  cv::threshold(diff, diff, 10, 255, CV_THRESH_BINARY);

  cv::Mat mask1, mask2, mask3;

  mask1 = ThresholdPlayerShip(frame, this->thresholdBounds[0][0], this->thresholdBounds[0][1]);
  mask2 = ThresholdPlayerShip(frame, this->thresholdBounds[1][0], this->thresholdBounds[1][1]);
  mask3 = ThresholdPlayerShip(frame, this->thresholdBounds[2][0], this->thresholdBounds[2][1]);

  //cv::Mat result = (mask1 | mask2 | mask3) & diff;
  cv::Mat result = (mask1 | mask2 | mask3);
  cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(11, 11));

  cv::dilate(result, result, kernel);
  cv::erode(result, result, kernel);

  return result;
}

int ShipDetector::findShipsBlobs(const std::vector<std::vector<cv::Point2i>> &contours,
                                 const cv::Mat &frame, const cv::Mat &binImage, Candidates *ships) {
  ships->fill(-1);
  //int numShipsDetected = 0;

  for (size_t i = 0; i < contours.size(); ++i) {
    cv::Rect bounds = cv::boundingRect(contours[i]);
    cv::Scalar mean = cv::mean(frame(bounds), binImage(bounds) != 0);

    if (mean >= this->thresholdBounds[0][0] && this->thresholdBounds[0][1] >= mean) {
      //this->tryInsertBlob(contours, static_cast<int>(i), ships, 0, kNumShips - 1, &numShipsDetected);
      this->tryInsertBlob(contours, i, ships, 0);
    } else if (mean >= this->thresholdBounds[1][0] && this->thresholdBounds[1][1] >= mean) {
      this->tryInsertBlob(contours, i, ships, 1);
    } else if (mean >= this->thresholdBounds[2][0] && this->thresholdBounds[2][1] >= mean) {
      this->tryInsertBlob(contours, i, ships, 2);
    }
  }

  //return numShipsDetected;
  return 0;
}

void ShipDetector::tryInsertBlob(const std::vector<std::vector<cv::Point2i>> &contours, size_t blob,
                                 Candidates *ships, size_t ship) {
  int shipIdx = (*ships)[ship];

  if (shipIdx == -1 || contours[shipIdx].size() < contours[blob].size()) {
    (*ships)[ship] = blob;
  }
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
