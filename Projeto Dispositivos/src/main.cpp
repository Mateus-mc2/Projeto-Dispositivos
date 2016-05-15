#include <chrono>
#include <iostream>
#include <vector>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/ximgproc.hpp>

#include "ShipDetector.h"

int main(int argc, char* argv[]) {
  cv::VideoCapture capture(0);

  if (!capture.isOpened()) {
    std::cout << "Couldn't find video device." << std::endl;
    system("pause");
    return -1;
  }

  cv::namedWindow("Video");
  cv::namedWindow("Binary");

  detection::ShipDetector detector;
  bool start = false;
  int currFrame = 1;

  while (true) {
    cv::Mat frame, filtered;
    capture >> frame;
    cv::Ptr<cv::ximgproc::DTFilter> ptrFilter = cv::ximgproc::createDTFilter(frame, 30, 10);

    ptrFilter->filter(frame, filtered);
    cv::GaussianBlur(filtered, filtered, cv::Size(7, 7), 1.5, 1.5);

    if (!start && currFrame == 10) {
      detector.init(filtered);
      start = true;
    } else if (start) {
      cv::Mat binImage = detector.thresholdImage(filtered, 10);
      cv::imshow("Binary", binImage);

      std::vector<std::vector<cv::Point2i>> contours;
      std::vector<cv::Vec4i> contoursHierarchy;

      cv::findContours(binImage, contours, contoursHierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE,
                       cv::Point2i(0, 0));
      detection::Candidates ships;
      int numShipsDetected = detector.findShipsBlobs(contours, &ships);

      for (int i = 0; i < numShipsDetected; ++i) {
        if (!contours[ships[i]].empty()) {
          cv::Rect shipBounds = cv::boundingRect(contours[ships[i]]);
          cv::rectangle(filtered, shipBounds, cv::Scalar(0, 255, 0));
        }
      }
    }

    cv::imshow("Video", filtered);
    ++currFrame;

    if (cv::waitKey(30) == 27)
    break;
  }

  cv::destroyAllWindows();

  return 0;
}
