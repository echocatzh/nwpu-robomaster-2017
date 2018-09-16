#include "transform.h"
#include "RMVideoCapture.hpp"
#include "ArmorDetector.h"
#include "SerialInterface.h"
#include <string>
#include <iostream>
#include <vector>

int main(int argc, char **argv)
{
    RMVideoCapture stream("/dev/video1", 3);
    stream.setVideoFormat(640, 480, 1);
    stream.setExposureTime(false, 100);
    stream.startStream();
    // cv::VideoCapture stream("/home/shirlim/new_rmvision/rmvision/20180322141116.avi");
    // cv::VideoCapture stream(0);

    cv::Mat srcImage;
    double yaw, pitch;
    auto start = cv::getTickCount();
    ArmorDetector detector;
    while (cv::waitKey(1) != 27)
    {
        stream >> srcImage;
        if (srcImage.data)
        {
            cv::resize(srcImage, srcImage, cv::Size(IMAGE_WIDTH, IMAGE_HEIGHT));
            chooseTarget(srcImage, detector);
            cv::imshow("src", srcImage);
        }
        else{
            std::cout << "break" << std::endl;
            continue;
        }
    }
    std::cout << (cv::getTickCount() - start) / cv::getTickFrequency() << " sec" << std::endl;
    stream.closeStream();
    return 0;
}
