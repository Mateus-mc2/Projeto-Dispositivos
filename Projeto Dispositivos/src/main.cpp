#include <chrono>
#include <condition_variable>
#include <fstream>
#include <iostream>
#include <mutex>
#include <queue>
#include <thread>
#include <vector>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/video.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/ximgproc.hpp>

#include "ShipDetector.h"
#include "ClientSocket.h"

enum class Trackbar { kHueMin, kHueMax, kSatMin, kSatMax, kValMin, kValMax };

int idx = 0;
const std::string kDefaultPort = "8001";
const std::string kServerHost = "localhost";
std::mutex guard;
std::condition_variable condition;
std::string message = "";
bool end = false;

bool LockCondition() { return message.compare("") != 0; }

void OnMouseCallback(int mouse_event, int x, int y, int flags, void *param) {
  if (mouse_event == CV_EVENT_LBUTTONDOWN) {
    std::vector<cv::Point2f> *ptr = static_cast<std::vector<cv::Point2f>*>(param);
    ptr->push_back(cv::Point2f(static_cast<float>(x), static_cast<float>(y)));
  }
}

int hMin = 0;
int hMax = 180;
int sMin = 0;
int sMax = 255;
int vMin = 0;
int vMax = 255;

void OnTrackbarCallback(int value, void *param) {
  Trackbar *bar = reinterpret_cast<Trackbar*>(param);

  switch (*bar) {
    case Trackbar::kHueMin:
      hMin = value;
      break;

    case Trackbar::kHueMax:
      hMax = value;
      break;

    case Trackbar::kSatMin:
      sMin = value;
      break;

    case Trackbar::kSatMax:
      sMax = value;
      break;

    case Trackbar::kValMin:
      vMin = value;
      break;

    case Trackbar::kValMax:
      vMax = value;
      break;

    default:
      break;
  }
}

void SendInfoToPlayer() {
  connection::ClientSocket socket(kServerHost, kDefaultPort);

  while (true) {
    try {
      socket.Connect();
      break;
    } catch (connection::SocketException &e) {
      std::cout << e.what() << std::endl;
    }
  }

  try {
    while (!end) {
      std::unique_lock<std::mutex> lock(guard);
      condition.wait(lock, LockCondition);

      std::cout << "Vou enviar..." << std::endl;
      socket.Send(message);
      std::cout << "Enviei!" << std::endl;
      message = "";
    }

    socket.Close();
  } catch (connection::SocketException &e) {
    std::cout << "  Exception caught (SocketException): " << e.what() << std::endl;
  }
}

int main(int argc, char* argv[]) {
  cv::VideoCapture capture(1);

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
  cv::VideoWriter outputVideo;
  cv::Mat H;

  std::thread connectionThread(SendInfoToPlayer);

  while (true) {
    cv::Mat frame, filtered;
    capture >> frame;
    cv::resize(frame, frame, cv::Size(705,400));

    filtered = cv::Mat::zeros(frame.size(), frame.type());
    cv::Ptr<cv::ximgproc::DTFilter> ptrFilter = cv::ximgproc::createDTFilter(frame, 30, 10);

    ptrFilter->filter(frame, filtered);
    cv::GaussianBlur(filtered, filtered, cv::Size(7, 7), 1.5, 1.5);
    cv::cvtColor(filtered, filtered, CV_BGR2YCrCb);

    /*std::vector<cv::Mat> channels;
    cv::split(filtered, channels);
    cv::equalizeHist(channels[0], channels[0]);
    cv::merge(channels, filtered);
    cv::cvtColor(filtered, filtered, CV_YCrCb2BGR);*/
    cv::cvtColor(filtered, filtered, CV_BGR2HSV);

    if (!start && currFrame == 10) {
      // Get homography.
      cv::namedWindow("Homography Settings");

      std::vector<cv::Point2f> domain_pts(4);
      /*float height = static_cast<float>(capture.get(CV_CAP_PROP_FRAME_HEIGHT));
      float width = static_cast<float>(capture.get(CV_CAP_PROP_FRAME_WIDTH));*/
      float height = 400;
      float width = 705.5;

      domain_pts[0] = cv::Point2f(0, 0);
      domain_pts[1] = cv::Point2f(width, 0);
      domain_pts[2] = cv::Point2f(width, height);
      domain_pts[3] = cv::Point2f(0, height);

      std::vector<cv::Point2f> image_pts;
      cv::setMouseCallback("Homography Settings", OnMouseCallback, static_cast<void*>(&image_pts));

      while (image_pts.size() < 4) {
        cv::imshow("Homography Settings", frame);
        cv::waitKey(30);
      }

      cv::destroyWindow("Homography Settings");
      H = cv::getPerspectiveTransform(image_pts, domain_pts);
      cv::warpPerspective(filtered, filtered, H, filtered.size());
      detector.init(filtered);

      // Get thresholding parameters.
      cv::namedWindow("Thresholded");
      cv::namedWindow("HSV parameters");

      Trackbar hueMin = Trackbar::kHueMin;
      Trackbar hueMax = Trackbar::kHueMax;
      Trackbar satMin = Trackbar::kSatMin;
      Trackbar satMax = Trackbar::kSatMax;
      Trackbar valMin = Trackbar::kValMin;
      Trackbar valMax = Trackbar::kValMax;

      cv::createTrackbar("HueMin", "HSV parameters", &hMin, 180, OnTrackbarCallback, &hueMin);
      cv::createTrackbar("HueMax", "HSV parameters", &hMax, 180, OnTrackbarCallback, &hueMax);
      cv::createTrackbar("SatMin", "HSV parameters", &sMin, 255, OnTrackbarCallback, &satMin);
      cv::createTrackbar("SatMax", "HSV parameters", &sMax, 255, OnTrackbarCallback, &satMax);
      cv::createTrackbar("ValMin", "HSV parameters", &vMin, 255, OnTrackbarCallback, &valMin);
      cv::createTrackbar("ValMax", "HSV parameters", &vMax, 255, OnTrackbarCallback, &valMax);

      cv::Mat mask;

      for (int i = 0; i < 3; ++i) {
        while (true) {
          cv::Scalar lower(hMin, sMin, vMin);
          cv::Scalar upper(hMax, sMax, vMax);

          cv::inRange(filtered, lower, upper, mask);
          if (!mask.empty()) cv::imshow("Thresholded", mask);

          if (cv::waitKey(30) == 27) {
            detector.SetParametersAt(i, lower, upper);
            break;
          }
        }
      }

      cv::destroyWindow("Thresholded");
      cv::destroyWindow("HSV parameters");

      //H = detector.getHomography().inv();
      outputVideo.open("demo.avi", CV_FOURCC('M', 'J', 'P', 'G'),
                       30, filtered.size());

      if (!outputVideo.isOpened()) {
        std::cout << "Bla" << std::endl;
        system("pause");
        return -1;
      }

      start = true;
    } else if (start) {
      cv::warpPerspective(filtered, filtered, H, filtered.size());
      cv::Mat binImage = detector.thresholdImage(filtered);
      cv::imshow("Binary", binImage);
      detection::MapNodes nodes = detector.getNodes();

      std::vector<std::vector<cv::Point2i>> contours;
      std::vector<cv::Vec4i> contoursHierarchy;

      cv::findContours(binImage, contours, contoursHierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE,
                       cv::Point2i(0, 0));
      detection::Candidates ships;
      detector.findShipsBlobs(contours, filtered, binImage, &ships);

      while (LockCondition())
        std::this_thread::yield();

      std::unique_lock<std::mutex> lock(guard);

      for (int i = 0; i < ships.size(); ++i) {
        bool found = false;

        if (ships[i] != -1) {
          cv::Moments shipMoments = cv::moments(contours[ships[i]]);
          cv::Point2i shipCentroid = cv::Point2i(cvRound(shipMoments.m10 / shipMoments.m00),
                                                 cvRound(shipMoments.m01 / shipMoments.m00));

          for (int j = 0; j < nodes.size() && !found; ++j) {
            if (nodes[j].contains(shipCentroid)) {
              message += std::to_string(j+1) + "," +
                         std::to_string(nodes[j].getInnerNode(shipCentroid)) + ",";
              found = true;
            }
          }
        }

        if (!found) {
          message += "-1,-1,";
        }
      }

      message.replace(message.end() - 1, message.end(), "\n");
      condition.notify_all();
    }

    if (start) {
       cv::warpPerspective(frame, frame, H, frame.size());
       detection::MapNodes nodes = detector.getNodes();

       for (int i = 0; i < nodes.size(); ++i) {
         nodes[i].drawExternalNode(&frame);
         nodes[i].drawInnerNodes(&frame);
         //nodes[i].drawMapNode(&filtered);
       }
    }

    cv::imshow("Video", frame);
    outputVideo.write(filtered);
    ++currFrame;

    if (cv::waitKey(30) == 27) {
      message = "Close this connection\n";
      end = true;
      condition.notify_all();
      connectionThread.join();
      break;
    }
  }

  capture.release();
  outputVideo.release();
  cv::destroyAllWindows();

  return 0;
}
