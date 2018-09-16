#include <iostream>
#include <vector>
#include <memory>
#include "transform.h"
#include "SerialInterface.h"
#include "RMVideoCapture.hpp"
#include "ArmorDetector.h"

using namespace std;
using namespace cv;

int main()
{
    auto start = cv::getTickCount();
    // RMVideoCapture stream("/dev/video1", 3);
    // stream.setVideoFormat(1280, 720, 1);
    // stream.setExposureTime(false, 10);
    // stream.startStream();
    // red
    VideoCapture stream("/home/shirlim/new_rmvision/rmvision/output.avi");
    // red
    // VideoCapture stream("/home/shirlim/new_rmvision/rmvision/20180322141116.avi");
    // blue
    // VideoCapture stream("/home/shirlim/new_rmvision/rmvision/20180322141426.avi");
    
    Mat srcImage;
    Mat grayImage;
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(1, 3));
    cv::Mat kernel_blue = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 7));

    ArmorDetector detector;
    vector<Point2f> board_position;
    board_position.clear();
    vector<vector<cv::Point>> contours;
    vector<cv::Vec4i> hierarchy;
    while (cv::waitKey(1) != 27)
    {
        stream >> srcImage;
        if (!srcImage.data)
            break;
        // std::cout << srcImage.size() << std::endl;
        cv::resize(srcImage, srcImage, cv::Size(IMAGE_WIDTH, IMAGE_HEIGHT));
        // throw std::runtime_error("No Image Data With func: chooseTarget(Mat &srcImage)!");

        cv::cvtColor(srcImage(detector.lastboard_rect), grayImage, CV_BGR2GRAY);
        // 这个阈值 最后 还得根据蓝色灯条的情况来决定，和HSV的做&，便可保证其稳定性
        grayImage = grayImage > 20;
        cv::Mat regImg = detector.hsvReg(srcImage(detector.lastboard_rect), detector.enemy_is_red);
        cv::bitwise_and(regImg, grayImage, regImg);
        if(detector.enemy_is_red == false)
        {
            cv::dilate(regImg, regImg, kernel_blue);
            cv::erode(regImg, regImg, kernel_blue);
        }
        
        vector<ArmorPos> armors_pos;
        vector<vector<cv::Point>> final_contours;

/**
 * 此处是为了判断因为目标过小而导致的无法识别，其实主要是识别的时候目标太小过不了第一轮筛选
 * 所以无法进行第二轮的判断。
 * 因为过小的时候轮廓几乎变成了圆的形状，而且轮廓也变得没有规则，
 * 所以要进行一些特征放大的操作。为了符合灯条的检测，最好使用类似椭圆的操作
 * 为了能够顺利进行第一轮的筛选，我采取的做法是面积小于1100的时候，进行morph运算，然后将之拉伸2倍，
 * 因为面积小于1200的时候，采用的是椭圆，所以resize更好一些
 **/
        // if (detector.lastboard_rect.area() < FARSIZE)
        // {
        //     detector.too_far = true;
        //     // dilate(regImg, regImg, kernel);
        //     // erode(regImg, regImg, kernel);
        //     // resize(regImg, regImg, cv::Size(), 1.0, 4.0);

        //     regImg = regImg > 1;
        // }

        cv::findContours(regImg, contours, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
        // 第一轮筛选
        for (int i = 0; i < contours.size() && contours.size() > 2; ++i)
        {
            //if contour.size<5, eillpse cannot be used
            if (contours[i].size() < 5)
                continue;

            //然后根据轮廓的角度，筛掉那些不满足垂直方向角度偏移
            RotatedRect minRect = fitEllipse(contours[i]);
            minRect = detector.adjustRRect(minRect);

            auto angle = minRect.angle;
            angle = 90 - angle;
            angle = angle < 0 ? angle + 180 : angle;
            if (std::abs(angle - 90) > 20)
                continue;
            final_contours.push_back(contours[i]);
        }

        if (contours.size() == 2)
            final_contours.push_back(contours[0]), final_contours.push_back(contours[1]);

        // 目前得到的contours是比较符合灯条形状的final_contours,接下来就是分别连接这些contour，判断形成的准装甲板是否符合一定条件
        for (int i = 0; i < final_contours.size(); ++i)
        {
            RotatedRect rRect1 = cv::minAreaRect(final_contours[i]);
            rRect1 = detector.adjustRRect(rRect1);

            vector<cv::Point> pos_1;
            detector.getTarget2dPoinstion(rRect1, pos_1);

            Point2f mid_u1, mid_d1;
            if (!detector.too_far)
            {
                mid_u1 = (pos_1[0] + pos_1[1]) / 2 + detector.lastboard_rect.tl();
                mid_d1 = (pos_1[2] + pos_1[3]) / 2 + detector.lastboard_rect.tl();
            }
            else
            {
                mid_u1 = Point(pos_1[0].x + pos_1[1].x, (pos_1[0].y + pos_1[1].y) / 4) / 2 + detector.lastboard_rect.tl();
                mid_d1 = Point(pos_1[2].x + pos_1[3].x, (pos_1[2].y + pos_1[3].y) / 4) / 2 + detector.lastboard_rect.tl();
            }

            auto angle_1 = atan2(mid_d1.y - mid_u1.y, mid_d1.x - mid_u1.x) * 180 / CV_PI;

            for (int j = i + 1; j < final_contours.size(); ++j)
            {

                RotatedRect rRect2 = cv::minAreaRect(final_contours[j]);
                vector<cv::Point> pos_2;
                rRect2 = detector.adjustRRect(rRect2);
                detector.getTarget2dPoinstion(rRect2, pos_2);
                Point2f mid_u2, mid_d2;
                if (!detector.too_far)
                {
                    mid_u2 = (pos_2[0] + pos_2[1]) / 2 + detector.lastboard_rect.tl();
                    mid_d2 = (pos_2[2] + pos_2[3]) / 2 + detector.lastboard_rect.tl();
                }
                else
                {
                    mid_u2 = Point(pos_2[0].x + pos_2[1].x, pos_2[0].y / 4 + pos_2[1].y / 4) / 2 + detector.lastboard_rect.tl();
                    mid_d2 = Point(pos_2[2].x + pos_2[3].x, pos_2[2].y / 4 + pos_2[3].y / 4) / 2 + detector.lastboard_rect.tl();
                }
                auto angle_2 = atan2(mid_d2.y - mid_u2.y, mid_d2.x - mid_u2.x) * 180 / CV_PI;
                if (std::abs(angle_1 - angle_2) > 3)
                    continue;

                // current board
                RotatedRect board = detector.boundingRRect(rRect1, rRect2);
                board.center += Point2f(detector.lastboard_rect.tl());
                vector<cv::Point> pos_board;
                detector.getTarget2dPoinstion(board, pos_board);

                // 角度筛选
                double angle = 0;
                if (pos_2[1].x != pos_1[2].x)
                    angle = std::atan2(pos_1[2].y - pos_2[1].y, pos_2[1].x - pos_1[2].x);
                if (angle > 5)
                    continue;

                //长宽筛选
                auto width = POINT_DIST(pos_board[0], pos_board[1]);
                auto height = POINT_DIST(pos_board[0], pos_board[2]);
                auto rate = width / height;
                // cout << rate << endl;
                if (rate > 3.5 || rate < 0.5)
                    continue;

                Mat dst;
                Point2f roi[4] = {pos_board[0], pos_board[1], pos_board[2], pos_board[3]};
                if (!detector.makeBoxSafe(roi))
                    continue;

                detector.perspectiveTransformation(srcImage, dst, roi, Size(105, 44));

                // BGR for the default, choose channel 2
                double dist = 0;
                dist = (detector.enemy_is_red ? (detector.getMSSIM(dst))[2] : (detector.getMSSIM(dst))[0]);

                board_position = {mid_u1, mid_d1, mid_u2, mid_d2};

                // for run_time value: this value ma be wrong, could not as mainly factor
                if (dist > 0.2)
                {
                    armors_pos.emplace_back(dist, board, board_position);
                }
            }
        }
        // use sort to find the best position
        sort(armors_pos.begin(), armors_pos.end(), [](ArmorPos a, ArmorPos b) { return a.diff > b.diff; });

        if (armors_pos.size() != 0)
        {
            ArmorPos final_board = armors_pos[0];
            detector.updatePosInfo(final_board);
            for (int t = 0; t < 4; ++t)
            {
                cv::line(srcImage, final_board.Pos[t], final_board.Pos[(t + 1) % 4], cv::Scalar(0, 255, 0));
            }
            detector.loss_count = 0;
            cv::rectangle(srcImage, detector.lastboard_rect, cv::Scalar(0, 0, 255));
        }
        else
        {
            // wait for 5 fps for normal armor
            if (detector.loss_count < 5)
            {
                ++detector.loss_count;
            }
            // quit waiting
            else
            {
                detector.loss_count = 0;
                detector.lastboard_rect = cv::Rect(0, 0, IMAGE_WIDTH, IMAGE_HEIGHT);
            }
        }
        imshow("gray", regImg);
        imshow("src", srcImage);
    }
    std::cout << (cv::getTickCount() - start) / cv::getTickFrequency() << std::endl;
    // stream.closeStream();
    return 0;
}