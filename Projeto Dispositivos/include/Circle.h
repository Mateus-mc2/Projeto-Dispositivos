#ifndef CIRCLE_H_
#define CIRCLE_H_

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

namespace geometry {

class Circle {
 public:
  /*Circle(const Circle &circle) : center_(circle.center()), radius_(circle.radius()),
                                          idx_(circle.idx()) {}*/
  Circle(const cv::Point2f &center, double radius, int idx) : center_(center), radius_(radius),
                                                              idx_(idx) {}
  ~Circle() {}

  bool contains(const cv::Point2f &point);
  void drawMapNode(cv::Mat *frame);

  // Accessors.
  int idx() const { return this->idx_; }
  cv::Point2f center() const { return this->center_; }
  double radius() const { return this->radius_; }

 private:
  cv::Point2f center_;
  double radius_;
  int idx_;

};

}  // namespace geometry

#endif  // CIRCLE_H_
