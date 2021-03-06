#ifndef MAP_NODE_H_
#define MAP_NODE_H_

#include <opencv2/imgproc.hpp>

#include "Circle.h"

namespace detection {

class MapNode{
 public:
  explicit MapNode(const std::vector<cv::Point2i> &vertices) : vertices_(vertices) {}

  bool contains(const cv::Point2f &point);
  void pushBackInnerNode(const geometry::Circle &circle);
  // Get the inner node which the ship lies within. If it doesn't belong to any 
  int getInnerNode(const cv::Point2f &point);
  void drawExternalNode(cv::Mat *frame);
  void drawInnerNodes(cv::Mat *frame);

 private:
  static const int kOutsideInnerNodes;

  std::vector<geometry::Circle> inner_nodes_;
  std::vector<cv::Point2i> vertices_;
};
}

#endif  // MAP_NODE_H_
