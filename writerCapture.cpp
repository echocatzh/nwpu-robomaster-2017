#include <opencv2/opencv.hpp>

using namespace cv;

int main()
{
    /** 打开输入视频文件 */
    cv::VideoCapture vc;
    vc.open( "/media/shirlim/Elements/20180123134810.avi" );
     
    if ( vc.isOpened() )
    {
        /** 打开输出视频文件 */
        cv::VideoWriter vw;
        vw.open( "output.avi", // 输出视频文件名
                (int)vc.get( CV_CAP_PROP_FOURCC ), // 也可设为CV_FOURCC_PROMPT，在运行时选取
                (double)vc.get(CV_CAP_PROP_FPS), // 视频帧率
                cv::Size( 640,480), // 视频大小
                true ); // 是否输出彩色视频
 
        /** 如果成功打开输出视频文件 */
        if ( vw.isOpened() )
        {
            cv::Mat frame;
            cv::Mat newFrame;
            while (true)
            {
                /** 读取当前视频帧 */

                vc >> frame;
 
                /** 若视频读取完毕，跳出循环 */
                if ( frame.empty() )
                {
                    break;
                }
                /** 将视频写入文件 */
                frame = frame(cv::Rect(0,0,1280,960));
                cv::resize(frame,frame,cv::Size(),0.5,0.5);
                vw << frame;
            }
        }
    }
 
    /** 手动释放视频捕获资源 */
    vc.release();
}