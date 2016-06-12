#include "MapNode.h"

namespace detection {

const int MapNode::kOutsideInnerNodes = -1;

bool MapNode::contains(const cv::Point2f &point) {
  return cv::pointPolygonTest(this->vertices_, point, false) >= 0;
}

void MapNode::pushBackInnerNode(const geometry::Circle &circle) {
  this->inner_nodes_.push_back(circle);
}

int MapNode::getInnerNode(const cv::Point2f &point) {
  for (int i = 0; i < this->inner_nodes_.size(); ++i) {
    if (this->inner_nodes_[i].contains(point)) {
      return this->inner_nodes_[i].idx();
    }
  }

  return this->kOutsideInnerNodes;
}

void MapNode::drawExternalNode(cv::Mat *frame) {
  cv::polylines(*frame, this->vertices_, true, cv::Scalar(255, 0, 0));
}

void MapNode::drawInnerNodes(cv::Mat *frame) {
  for (size_t i = 0; i < this->inner_nodes_.size(); ++i) {
    cv::circle(*frame, this->inner_nodes_[i].center(),
               static_cast<int>(this->inner_nodes_[i].radius()), cv::Scalar(0, 0, 255));
  }
}
}  // namespace detection
