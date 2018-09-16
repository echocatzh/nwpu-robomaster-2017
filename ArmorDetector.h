#ifndef ARMOR_DETECTOR_H
#define ARMOR_DETECTOR_H

#include <opencv2/opencv.hpp>
#include <iostream>

#define POINT_DIST(p1, p2) std::sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y))

#define IMAGE_WIDTH 640
#define IMAGE_HEIGHT 480
#define FARSIZE 100

enum ARMORTYPE
{
  NONE = 0x00,
  SMALL = 0x01,
  LARGE = 0x02
};

struct ArmorPos
{
  double diff;
  cv::RotatedRect rRect;
  std::vector<cv::Point2f> Pos;
  ARMORTYPE armor_type;

  ArmorPos() = default;
  ArmorPos(double difference,
           cv::RotatedRect r,
           std::vector<cv::Point2f> position) : diff(difference), rRect(r), Pos(position) {}
};

class ArmorDetector
{
  friend std::vector<cv::Point2f> chooseTarget(cv::Mat &, ArmorDetector &);

public:
  ArmorDetector();
  cv::Mat hsvReg(const cv::Mat &srcImage, bool is_red);
  cv::Scalar getMSSIM(cv::Mat &i1);
  cv::RotatedRect adjustRRect(const cv::RotatedRect &);
  cv::RotatedRect boundingRRect(const cv::RotatedRect &, const cv::RotatedRect &);
  bool makeBoxSafe(cv::Point2f *);
  void getTarget2dPoinstion(const cv::RotatedRect &rect, std::vector<cv::Point> &target2d);
  void perspectiveTransformation(cv::Mat, cv::Mat &, cv::Point2f *, cv::Size);
  void updatePosInfo(const ArmorPos &);
  void rRect_resize(cv::RotatedRect &rRect, double width_rate, double height_rate);

private:
  cv::Mat grayImage;
  cv::Mat _binary_small_template; //MSSIM
  cv::Mat _binary_large_template;

public:
  bool too_far;
  bool enemy_is_red;
  // record last detected board, and times 1.25
  cv::Rect lastboard_rect;
  size_t loss_count;
  cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(1, 3));
};

#endif //ARMOR_DETECTOR_H