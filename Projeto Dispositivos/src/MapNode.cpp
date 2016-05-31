#include "MapNode.h"

namespace detection {

bool MapNode::contains(const cv::Point2f &point) {
  return cv::pointPolygonTest(this->vertices_, point, false) >= 0;
}

}  // namespace detection
