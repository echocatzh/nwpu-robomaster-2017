// this hpp contain all the method used for tf
#include "opencv2/opencv.hpp"
#include <iostream>
#include <string>
struct angle_limit{
    double x_min;
    double x_max;
    double y_min;
    double y_max;
    angle_limit(double _a, double _b, double _c, double _d):x_min(_a),x_max(_b),y_min(_c),y_max(_d){}
    bool check(double angle_x, double angle_y){ //限位
        // std::cout<<"the angle is "<<angle_x<<" "<<angle_y<<std::endl;
        return (angle_x>x_min && angle_x<x_max && angle_y>y_min && angle_y<y_max);
    }
};

class Tf_maker{
public:
    Tf_maker(){
      _limit = new angle_limit(-3.141592653589793,3.141592653589793,-0.7853981633974483,0.3490658503988659); //limit for yaw and pitch
    };
    ~Tf_maker(){
        delete _limit;
    };
    bool init(std::string config_name);
    cv::Mat Camera2Base(cv::Mat point_in_cam);
    void getTrans(const std::vector<cv::Point2f> & points2d, cv::Mat & trans);
    void getTarget2dPosition(const cv::RotatedRect & rect, std::vector<cv::Point2f> & target2d);

    bool calculateAngle(double &yaw, double &pitch, cv::RotatedRect rect);
// private:


    //用欧拉角变换方法描述相机坐标系和云台坐标系之间的变换
    //角的测量方法和欧拉角变换顺序有关
    //应该是按照yxz的欧拉角坐标系变换方法
    /**the param of the tf(camera and the gimbal) **/
    cv::Mat _CameraOrg;
    double _alpha;
    double _beta;
    double _gamma;
    //use for pnp tf
    double _width;
    double _height;
    double _offset_barrel;
    double _min_distance;
    double _max_distance;
    cv::Mat _camera_matrix;
    cv::Mat _distCo;
    cv::Mat _rot;
    angle_limit *_limit;
};