// driveroncan.h
//
// ~~~~~~~~~~~~
//
// API
//
// ~~~~~~~~~~~~
//
// ------------------------------------------------------------------
// Author :
// Last change: 2017-3-23
//
// Language:
// ------------------------------------------------------------------
//
// Copyright (C) 2017
//
#ifndef DRIVERONCAN_H
#define DRIVERONCAN_H

#include "driver.h"
#include "joint.h"
#include "gripper.h"
#include <pthread.h>
#include <vector>

#define SEARCH_MAX_ID           40      // 搜索ID时的上限

#define CAN_CMD                 0
#define CAN_INDEX               1
#define CAN_DATA                2

#define BROADCAST_ID        0xFF        // Broadcast Identifier

class DriverOnCan : public Driver {
public:
    DriverOnCan();
    virtual bool BusInit(const char* deviceName);
    virtual bool BusClose(const char* deviceName);
    virtual bool SendMsg(uint16_t ID, uint8_t commandType, uint8_t addr, uint8_t *data, uint8_t dataLength);
    virtual bool ReceiveMsg();
    virtual bool ThreadStart();
    virtual bool GetValueInTable(uint16_t ID, uint8_t addr, uint16_t &data);

    /// @brief 读线程
    void * thread_read();

    /// @brief 更新总线上的设备
    /// @return bool Indication of successful
    bool updateAllID();

    std::vector<Joint> allJoint;        // 关节
    std::vector<Gripper> allGripper;    // 手爪
protected:
    void searchIDandInitMemoryTable();  // 搜索ID并初始化模块的内存控制表
    void * m_hCan;                      // Handle of CAN bus
    pthread_mutex_t hMutex;             // mutex signal of CAN writing
    pthread_t receive_thread;
};

#endif // DRIVERONCAN_H
