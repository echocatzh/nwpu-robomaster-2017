#include <opencv2/opencv.hpp>
#include <iostream>

using namespace std;
using namespace cv;

int Hmax = 255, Hmin = 50, Smax = 255, Smin = 0, Vmax = 255, Vmin = 0;

Mat Hsv = imread("../test.jpg");

cv::Mat findColor(const cv::Mat &inputBGRimage, int Hmax, int Hmin, int Smax, int Smin, int Vmax, int Vmin)
{
    cv::Mat input = inputBGRimage.clone();
    cv::Mat imageHSV;     //(input.rows, input.cols, CV_8UC3);
    cv::Mat imgThreshold; //(input.rows, input.cols, CV_8UC1);
    cv::cvtColor(input, imageHSV, cv::COLOR_BGR2HSV);
    cv::inRange(imageHSV, cv::Scalar(Hmin, Smin, Vmin, 0), cv::Scalar(Hmax, Smax, Vmax, 0), imgThreshold);

    return imgThreshold;
}

void on_track(int, void *)
{
    Mat tmp = findColor(Hsv, Hmax, Hmin, Smax, Smin, Vmax, Vmin);
    Mat kernel = cv::getStructuringElement(MORPH_RECT, cv::Size(17, 17));
    cv::erode(tmp, tmp, kernel);
    cv::dilate(tmp, tmp, kernel);
    vector<vector<cv::Point>> contours;
    vector<cv::Vec4i> hierarchy;
	cv::findContours(tmp, contours, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
    sort(contours.begin(),contours.end(),[](vector<Point> x, vector<Point>y) {return contourArea(x)>contourArea(y);});

    drawContours(Hsv,contours,0,cv::Scalar(255,255,0),20);
    
    imshow("HSVTest", tmp);
    imshow("HSVSrc", Hsv);
}

int main()
{
    // to dis r and g
    namedWindow("HSVTest", CV_WINDOW_NORMAL);
    namedWindow("HSVSrc", CV_WINDOW_NORMAL);
    // use this program to define what value we should use
    createTrackbar("Hmin", "HSVTest", &Hmin, 255, on_track);
    createTrackbar("Hmax", "HSVTest", &Hmax, 255, on_track);
    createTrackbar("Smin", "HSVTest", &Smin, 255, on_track);
    createTrackbar("Smax", "HSVTest", &Smax, 255, on_track);
    createTrackbar("Vmin", "HSVTest", &Vmin, 255, on_track);
    createTrackbar("Vmax", "HSVTest", &Vmax, 255, on_track);

    waitKey(0);
}