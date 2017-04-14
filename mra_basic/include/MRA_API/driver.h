// driver.h
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
#ifndef DRIVER_H
#define DRIVER_H

#include <stdint.h>

// Macro definition of command type
#define CMDTYPE_RD          0x01    // Reading command type
#define CMDTYPE_WR          0x02    // Writing command type
#define CMDTYPE_WR_NR       0x03    // Writing command type. No reutrn
#define CMDTYPE_WR_REG      0x04    // Asynchronous writing command type (Reserved)
#define CMDTYPE_SCP         0x05    // Command for data return of oscilloscope (Reserved)
#define CMDTYPE_RD_HP       0x08    // 读控制表指令，高优先级


class Driver
{
public:
    Driver();

    /// @brief 初始化总线
    /// @param deviceName 设备名称
    /// @return bool Indication of successful
    virtual bool BusInit(const char* deviceName) = 0;

    /// @brief 关闭总线
    /// @param deviceName 设备名称
    /// @return bool Indication of successful
    virtual bool BusClose(const char* deviceName) = 0;

    /// @brief 总线上发送数据包
    /// @param ID 设备ID
    /// @param commandType 命令类型
    /// @param addr 地址
    /// @param data 数据
    /// @param dataLength 数据长度
    /// @return bool Indication of successful
    virtual bool SendMsg(uint16_t ID,
                         uint8_t commandType,
                         uint8_t addr,
                         uint8_t *data,
                         uint8_t dataLength) = 0;

    /// @brief 接收数据包
    /// @return bool Indication of successful
    virtual bool ReceiveMsg() = 0;

    /// @brief 开始工作
    /// @return bool Indication of successful
    virtual bool ThreadStart() = 0;

    /// @brief 获取模块内存控制表数据
    /// @param ID 设备ID
    /// @param addr 地址
    /// @param data 数据
    /// @return bool Indication of successful
    virtual bool GetValueInTable(uint16_t ID,
                                 uint8_t addr,
                                 uint16_t &data) = 0;
};

#endif // DRIVER_H
