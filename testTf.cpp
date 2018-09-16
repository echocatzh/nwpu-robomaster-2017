#include <iostream>
#include "transform.h"

Tf_maker tf_maker;
bool test_tf(double& yaw,double& pitch,cv::Mat _xyz){

    const double *xyz = (const double *)_xyz.data;
    // std::cout<<"the location is: "<<xyz[0]<<" "<<xyz[1]<<" "<<xyz[2]<<std::endl;
    if(xyz[2]>tf_maker._max_distance || xyz[2]<tf_maker._min_distance){
        std::cout<<"the length is out of range "<<xyz[2]<<std::endl;
        return false;
    }
    double alpha = 0.0, theta = 0.0;
    //根据技术手册，应该是用theta - alpha = pitch
    alpha = asin(tf_maker._offset_barrel / sqrt(xyz[1] * xyz[1] + xyz[2] * xyz[2]));
    double distance = xyz[2] * cos(pitch);
    if (xyz[1] < 0)
    {
        theta = atan(-xyz[1] / xyz[2]);
        pitch += -(alpha + theta) ; // up<0,down>0
    }
    else if (xyz[1] < tf_maker._offset_barrel)
    {
        theta = atan(xyz[1] / xyz[2]);
        pitch += -(alpha - theta) ; // camera coordinate
    }
    else
    {
        theta = atan(xyz[1] / xyz[2]);
        pitch += (theta - alpha) ; // camera coordinate
    }

    yaw += atan2(xyz[0], distance);
    // std::cout<<"the angles are yaw\t"<<yaw<<" pitch\t"<<pitch<<std::endl;
    return tf_maker._limit->check(yaw, pitch);
}

//使用一个较小的连续角度测试
int main(){
    cv::Mat pos_in_cam = (cv::Mat_<double>(3,1)
                    << 3.0,-1.0,370.0);
    tf_maker.init("../camera.yml");
    cv::Mat pos_in_gim = tf_maker.Camera2Base(pos_in_cam);
    std::cout<<"the pos_in_gim is:\n"<<pos_in_gim<<std::endl;
    double yaw=0,pitch=0;
    while(test_tf(yaw,pitch,pos_in_gim)){
        std::cout<<"set value is yaw\t"<<yaw*180/3.141592653589793<<" pitch\t"<<pitch*180/3.141592653589793<<std::endl;
    }
    return 0;
    

}