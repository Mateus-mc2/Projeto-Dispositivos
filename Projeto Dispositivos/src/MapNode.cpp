#include "MapNode.h"

namespace detection {

const int MapNode::kOutsideInnerNodes = -1;

bool MapNode::contains(const cv::Point2f &point) {
  return cv::pointPolygonTest(this->vertices_, point, false) >= 0;
}

int MapNode::getInnerNode(const cv::Point2f &point) {
  for (int i = 0; i < this->inner_nodes_.size(); ++i) {
    if (this->inner_nodes_[i].contains(point)) {
      return i;
    }
  }

  return this->kOutsideInnerNodes;
}

void MapNode::drawExternalNode(cv::Mat *frame) {
  cv::polylines(*frame, this->vertices_, true, cv::Scalar(255, 0, 0));
}

}  // namespace detection
