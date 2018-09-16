#ifndef ROBOT_ABSTRACT_H
#define ROBOT_ABSTRACT_H




//MSG struct is the  unit of communication , also is the unit of robot abstract

struct MSGPose{
    float  pitch;
    float  yaw;
};

struct RobotAbstract
{
    RobotAbstract()
    {
        measure_gimbal_pose.pitch = 0;
        measure_gimbal_pose.yaw = 0;
        expect_gimbal_pose.pitch = 0;
        expect_gimbal_pose.yaw = 0;
        is_shooting = false;
    }

    MSGPose measure_gimbal_pose;
    MSGPose expect_gimbal_pose;
    bool    is_shooting;
   
};

#endif // ROBOT_ABSTRACT_H

