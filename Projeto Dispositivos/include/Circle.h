#ifndef CIRCLE_H_
#define CIRCLE_H_

#include <opencv2/core.hpp>

namespace geometry {

class Circle {
 public:
  Circle(const cv::Point2d &center, double radius) : center_(center), radius_(radius) {}
  ~Circle() {}

  bool contains(const cv::Point2d &point);

 private:
  cv::Point2d center_;
  double radius_;
};

}  // namespace geometry

#endif  // CIRCLE_H_
