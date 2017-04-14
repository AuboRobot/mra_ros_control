#ifndef USERCONTROLONCAN_H
#define USERCONTROLONCAN_H

#include "driveroncan.h"

class UserControlOnCan
{
public:
    UserControlOnCan();
    ~UserControlOnCan();
    bool Init(const char * deviceName);
    bool Close(const char* deviceName);

    bool updateJointCurPos(uint32_t theID);
    float readJointCurPos(uint32_t theID, int cmd = JOINT_RADIAN);
    bool setJointTagPos(uint32_t theID, float angle, int cmd = JOINT_RADIAN);
    bool setJointAutoUpdateCurPos(uint32_t theID, bool enabled);

    bool updateJointCurSpd(uint32_t theID);
    float readJointCurSpd(uint32_t theID);
    bool setJointTagSpd(uint32_t theID, float speed);
    bool setJointAutoUpdateCurSpd(uint32_t theID, bool enabled);

    bool updateJointCurI(uint32_t theID);
    float readJointCurI(uint32_t theID);
    bool setJointTagI(uint32_t theID, int curI);
    bool setJointAutoUpdateCurI(uint32_t theID, bool enabled);

    bool setGripperTagOpenStatus(uint32_t theID, int mode);
    DriverOnCan controller;

private:
    Joint *findJointID(uint32_t theID);
    Gripper *findGripperID(uint32_t theID);
};

#endif // USERCONTROLONCAN_H
