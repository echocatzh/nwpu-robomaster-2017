#include <opencv2/opencv.hpp>
#include <RMVideoCapture.hpp>
#include <iostream>
#include <string>
#include <sstream>

using namespace std;
using namespace cv;

int main()
{
    RMVideoCapture stream("/dev/video1", 3);
    stream.setVideoFormat(640, 480, 1);
    stream.setExposureTime(true, 5000);
    stream.startStream();
    cv::Mat srcImage;
    int i = 0;
    string format = "640_480_";
    ostringstream name;
    while (1)
    {
        stream >> srcImage;
        imshow("src", srcImage);
        if(waitKey(10)==' ')
        {
            name << format << i++;
            imwrite(name.str()+".jpg", srcImage);
            name.str("");
            cout << "saved :" << format << i << endl;
        }else if (waitKey(10)==27)break;
    }
    stream.closeStream();
    return 0;
}
