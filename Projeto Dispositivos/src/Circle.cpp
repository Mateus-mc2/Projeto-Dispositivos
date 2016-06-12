#include "Circle.h"

namespace geometry {

bool Circle::contains(const cv::Point2f &point) {
  float x0 = this->center_.x, y0 = this->center_.y;
  float x = point.x, y = point.y;

  return std::pow(x - x0, 2) + std::pow(y - y0, 2) <= std::pow(this->radius_, 2);
}

void Circle::drawMapNode(cv::Mat *frame) {
  cv::circle(*frame, this->center_, static_cast<int>(this->radius_), cv::Scalar(0, 0, 255));
}

}  // namespace geometry
