#include "transform.h"

bool Tf_maker::init(std::string config_name)
{
    cv::FileStorage camera_cfg(config_name, cv::FileStorage::READ);
    if (!camera_cfg.isOpened())
    {
        std::cout << "wrong config_file address" << std::endl;
        return false;
    }
    camera_cfg["camera_matrix"] >> _camera_matrix;
    camera_cfg["distortion_coefficients"] >> _distCo;
    camera_cfg["cameraOrg"] >> _CameraOrg;
    camera_cfg["alpha"] >> _alpha;
    camera_cfg["beta"] >> _beta;
    camera_cfg["gamma"] >> _gamma;
    camera_cfg["width"] >> _width;
    camera_cfg["height"] >> _height;
    camera_cfg["offset_barrel"] >> _offset_barrel;
    camera_cfg["min_distance"]>>_min_distance;
    camera_cfg["max_distance"]>>_max_distance;
    _rot = (cv::Mat_<double>(3, 3)
        <<  sin(_alpha) * sin(_beta) * sin(_gamma) + cos(_alpha)*cos(_gamma),   sin(_alpha) * sin(_beta) * cos(_gamma) - cos(_alpha)*sin(_gamma),   sin(_alpha) * cos(_beta),
            cos(_beta) * sin(_gamma),                                           cos(_beta) * cos(_gamma),                                           -sin(_beta),
            cos(_alpha) * sin(_beta) * sin(_gamma) - sin(_alpha) * cos(_gamma), cos(_alpha) * sin(_beta) * cos(_gamma) + sin(_alpha) * sin(_gamma), cos(_alpha) * cos(_beta));
    std::cout<<"the trans_matrix is:\n"<<_CameraOrg<<std::endl;
    std::cout<<"the rot_matrix is:\n"<<_rot<<std::endl;
    std::cout<<"the armor size is :"<<"\n\twidth "<<_width<<"\n\theight "<<_height<<"\n\toffset_barrel"<<_offset_barrel<<std::endl; 
    std::cout<<"the max distance is:"<<_max_distance<<" the min distance is: "<<_min_distance<<std::endl;
    return true;
}
cv::Mat Tf_maker::Camera2Base(cv::Mat point_in_cam)
{
    cv::Mat point_in_gim = _rot * point_in_cam + _CameraOrg;
    std::cout<<"the point in camera\n"<<point_in_cam<<"\nthe point in gim\n"<<point_in_gim<<std::endl;
    //point_in_gim.at<double>(0,0) = -point_in_gim.at<double>(0,0);
    return point_in_gim;

}
void Tf_maker::getTrans(const std::vector<cv::Point2f> &points2d, cv::Mat &trans)
{
    trans = (cv::Mat_<double>(3,1)<< 0.0,0.0,0.0);
    cv::Mat rot = (cv::Mat_<double>(3,1)<< 0.0,0.0,0.0);
    std::vector<cv::Point3d> point3d;
    double half_x = _width / 2.0;
    double half_y = _height / 2.0;

    point3d.push_back(cv::Point3f(-half_x, -half_y, 0));
    point3d.push_back(cv::Point3f(half_x, -half_y, 0));
    point3d.push_back(cv::Point3f(half_x, half_y, 0));
    point3d.push_back(cv::Point3f(-half_x, half_y, 0));

    cv::Mat r;
    cv::solvePnP(point3d, points2d, _camera_matrix, _distCo, r, trans);
}
void Tf_maker::getTarget2dPosition(const cv::RotatedRect &rect, std::vector<cv::Point2f> &target2d)
{
    cv::Point2f vertices[4];
    rect.points(vertices);
    cv::Point2f lu, ld, ru, rd;
    //按照x坐标从小到大排序
    std::sort(vertices, vertices + 4, [](const cv::Point2f &p1, const cv::Point2f &p2) { return p1.x  < p2.x; });
    if (vertices[0].y < vertices[1].y)
    {
        lu = vertices[0];
        ld = vertices[1];
    }
    else
    {
        lu = vertices[1];
        ld = vertices[0];
    }
    if (vertices[2].y < vertices[3].y)
    {
        ru = vertices[2];
        rd = vertices[3];
    }
    else
    {
        ru = vertices[3];
        rd = vertices[2];
    }
    target2d.clear();
    target2d.push_back(lu/2);
    target2d.push_back(ru/2);
    target2d.push_back(rd/2);
    target2d.push_back(ld/2);
}
// 全部改用弧度值，就不会出错了
bool Tf_maker::calculateAngle(double &yaw, double &pitch, cv::RotatedRect rect)
{
    //the trans in the camera_coordinate
    // rect.size.height = rect.size.width / wh_ratio;
    double hw_ratio = _height/_width; 
    rect.size.height = rect.size.width *hw_ratio;
    cv::Mat trans;
    std::vector<cv::Point2f> points2d;
    getTarget2dPosition(rect, points2d);
    getTrans(points2d, trans);
    //get the trans in the gimbal_coordinate
    cv::Mat _xyz = Camera2Base(trans);
    const double *xyz = (const double *)_xyz.data;
    // std::cout<<"the location is: "<<xyz[0]<<" "<<xyz[1]<<" "<<xyz[2]<<std::endl;
    if(xyz[2]>_max_distance || xyz[2]<_min_distance){
        std::cout<<"the length is out of range "<<xyz[2]<<std::endl;
        return false;
    }
    double alpha = 0.0, theta = 0.0;
    //根据技术手册，应该是用theta - alpha = pitch
    alpha = asin(_offset_barrel / sqrt(xyz[1] * xyz[1] + xyz[2] * xyz[2]+ xyz[0]*xyz[0]));
    double distance = xyz[2] * cos(pitch);
    // double distance = xyz[2];
    theta = atan2(xyz[1], xyz[2]);
    pitch += (-alpha + theta); // camera coordinate

    yaw += atan2(xyz[0], distance);
    return _limit->check(yaw, pitch);
}
