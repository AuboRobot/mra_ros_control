// gripper.h
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
#ifndef GRIPPER_H
#define GRIPPER_H

#include "driver.h"

#define CMDLENGTH_FINGER 0x44 // 内存控制表长度（单位：字节）

//系统状态相关
#define UPDATE_LEFT_SPD_FINGER      0x01
#define UPDATE_LEFT_POS_FINGER      0x02
#define UPDATE_LEFT_TMP_FINGER      0x04
#define UPDATE_LEFT_TRQ_FINGER      0x08
#define UPDATE_RIGHT_SPD_FINGER     0x10
#define UPDATE_RIGHT_POS_FINGER     0x20
#define UPDATE_RIGHT_TMP_FINGER     0x40
#define UPDATE_RIGHT_TRQ_FINGER     0x80

#define SYS_ID_FINGER               0x01    // 手抓ID
#define SYS_MODEL_TYPE_FINGER       0x02    // 驱动器型号：0x10-M14，0x20-M17,0x21-M17E,0x30-M20，0x80-Gripper
#define SYS_FW_VERSION_FINGER       0x03    // 固件版本
#define SYS_ERROR_FINGER            0x04    // 错误代码
#define SYS_VOLTAGE_FINGER          0x05    // 系统电压（0.01V）,暂不可用
#define SYS_TEMP_FINGER             0x06    // 系统温度（0.1℃）,暂不可用
#define SYS_SERVO_LED_FINGER        0x07    // 舵机LED:高8位-舵机ID，低8位-亮灭
#define SYS_BAUDRATE_485_FINGER     0x08    // RS485总线波特率(串口波特率)
#define SYS_BAUDRATE_CAN_FINGER     0x09    // CAN总线波特率
#define SYS_ENABLE_TORQUE_FINGER    0x0A    // 使能舵机力矩输出
#define SYS_ENABLE_ON_POWER_FINGER  0x0B    // 上电时使能舵机力矩输出
#define SYS_SAVE_TO_FLASH_FINGER    0x0C    // 保存数据到FLASH标志（自动清零）
#define SYS_SET_ZERO_POS_FINGER     0x0E    // 设置零点
#define SYS_CLEAR_ERROR_FINGER      0x0F    // 清除错误标志

#define SYS_GET_STATE_MASK_FINGER   0x10    // 更新舵机的信息，相应位置写1，自动清除
#define SYS_POSITION_LEFT_FINGER    0x11    // 左舵机当前位置
#define SYS_POSITION_RIGHT_FINGER   0x12    // 右舵机当前位置
#define SYS_SPEED_LEFT_FINGER       0x13    // 左舵机当前速度
#define SYS_SPEED_RIGHT_FINGER      0x14    // 右舵机当前速度
#define SYS_LOAD_LEFT_FINGER        0x15    // 左舵机当前负载
#define SYS_LOAD_RIGHT_FINGER       0x16    // 右舵机当前负载
#define SYS_VOLTAGE_LEFT_FINGER     0x17    // 左舵机当前电压（0.1V）
#define SYS_VOLTAGE_RIGHT_FINGER    0x18    // 右舵机当前电压（0.1V）
#define SYS_TEMP_LEFT_FINGER        0x19    // 左舵机当前温度（1℃）
#define SYS_TEMP_RIGHT_FINGER       0x1A    // 右舵机当前温度（1℃）
#define SYS_ZERO_LEFT_FINGER        0x1B    // 左舵机零点
#define SYS_ZERO_RIGHT_FINGER       0x1C    // 右舵机零点

#define TAG_WORK_MODE_FINGER        0x20    // 工作模式：1-开合，2-位置
#define TAG_OPEN_ANGLE_FINGER       0x21    // 开合大小
#define TAG_OPEN_STA_FINGER         0x22    // 手指开合状态
#define TAG_POSITION_LEFT_FINGER    0x23    // 左舵机目标位置
#define TAG_POSITION_RIGHT_FINGER   0x24    // 右舵机目标位置

#define MAX_TORQUE_FINGER           0x30    // 最大限制转矩(舵机ROM)
#define LIMIT_TORQUE_FINGER         0x31    // 最大限制转矩(舵机RAM)
#define MOVING_SPEED_FINGER         0x32    // 转动速度
#define COMPLIANCE_MARGIN_FINGER    0x33    // 位置误差
#define COMPLIANCE_SLOPE_FINGER     0x34    // 加速度
#define PUNCH_FINGER                0x35
#define RETURN_DELAY_FINGER         0x36    // Status返回延迟
#define CW_ANGLE_LEFT_FINGER        0x37    // 左舵机顺时针角度限制
#define CCW_ANGLE_LEFT_FINGER       0x38    // 左舵机逆时针角度限制
#define CW_ANGLE_RIGHT_FINGER       0x39    // 右舵机顺时针角度限制
#define CCW_ANGLE_RIGHT_FINGER      0x3A    // 右舵机逆时针角度限制

#define ANY_CMD_FINGER              0x40    // 舵机任意读写命令
#define ANY_ID_FINGER               0x41    // 舵机ID
#define ANY_ADD_FINGER              0x42    // 舵机读写地址
#define ANY_WRITE_WORD_FINGER       0x43    // 舵机写入字内容
#define ANY_READ_WORD_FINGER        0x44    // 舵机读取字内容

//错误字节MASK定义
#define ERROR_MASK_OVER_VOLTAGE_FINGER  0x0001      // 过压
#define ERROR_MASK_UNDER_VOLTAGE_FINGER 0x0002      // 欠压
#define ERROR_MASK_OVER_TEMP_FINGER     0x0004      // 过温
#define ERROR_MASK_RX28_FINGER          0x0008      // 舵机错误

class Gripper
{
public:
    Gripper(uint32_t ID, Driver * d);

    /// @brief Read the Finger left motor position.
    /// @return float Finger left motor position.
    float getFingerCurPosLeft();

    /// @brief Send read msg of the current position to finger left motor .
    /// @return bool Indication of successful
    bool readFingerCurPosLeft();

    /// @brief read the Finger right motor position.
    /// @return float Finger right motor position.
    float getFingerCurPosRight();

    /// @brief Send read msg of the current position to finger right motor .
    /// @return bool Indication of successful
    bool readFingerCurPosRight();

    /// @brief read the Finger left motor speed.
    /// @return Finger left motor speed.
    float getFingerCurSpeedLeft();

    /// @brief Send read msg of the current speed to finger left motor .
    /// @return bool Indication of successful
    bool readFingerCurSpeedLeft();

    /// @brief read the Finger right motor speed.
    /// @return Finger right motor speed.
    float getFingerCurSpeedRight();

    /// @brief Send read msg of the current speed to finger right motor .
    /// @return bool Indication of successful
    bool readFingerCurSpeedRight();

    /// @brief read the Finger left motor load.
    /// @return Finger left motor load.
    float getFingerCurLoadLeft();

    /// @brief Send read msg of the current load to finger left motor .
    /// @return bool Indication of successful
    bool readFingerCurLoadLeft();

    /// @brief read the Finger right motor load.
    /// @return Finger right motor load.
    float getFingerCurLoadRight();

    /// @brief Send read msg of the current load to finger right motor .
    /// @return bool Indication of successful
    bool readFingerCurLoadRight();

    /// @brief read the Finger left motor voltage.
    /// @return Finger left motor voltage.
    float getFingerCurVoltLeft();

    /// @brief Send read msg of the current volt to finger left motor .
    /// @return bool Indication of successful
    bool readFingerCurVoltLeft();

    /// @brief read the Finger right motor voltage.
    /// @return Finger right motor voltage.
    float getFingerCurVoltRight();

    /// @brief Send read msg of the current volt to finger right motor .
    /// @return bool Indication of successful
    bool readFingerCurVoltRight();

    /// @brief read the Finger left motor temperature.
    /// @return Finger left motor temperature.
    int getFingerCurTempLeft();

    /// @brief Send read msg of the current temperature to finger left motor .
    /// @return bool Indication of successful
    bool readFingerCurTempLeft();

    /// @brief read the Finger right motor temperature.
    /// @return Finger right motor temperature.
    int getFingerCurTempRight();

    /// @brief Send read msg of the current temperature to finger right motor .
    /// @return  bool Indication of successful
    bool readFingerCurTempRight();

    /// @brief get the Finger target speed.
    /// @return Finger target speed.
    float getFingerTagSpeed();

    /// @brief Send read msg of the target speed to finger .
    /// @return bool Indication of successful
    bool readFingerTagSpeed();

    /// @brief set the Finger target speed.
    /// @param speed speed
    /// @return bool Indication of successful
    bool setFingerTagSpeed(float speed);

    /// @brief Read the error code of joint.
    void getFingerError();

    /// @brief Send read msg of the error status to finger .
    /// @return bool Indication of successful
    bool readFingerError();

    /// @brief Update finger motor info .
    /// @param type
    /// @return bool Indication of successful
    bool FingerMotorInfoUpdate(int type);

    /// @brief Clear finger error info .
    /// @return bool Indication of successful
    bool FingerClearErr();

    /// @brief Set finger current position as zero .
    /// @return bool Indication of successful
    bool setFingerCurPosAsZero();

    /// @brief Set finger id.
    /// @param newid New finger ID
    /// @return bool Indication of successful
    bool setFingerID(int newid);

    /// @brief Send read msg of the model type to finger .
    /// @return bool Indication of successful
    bool readFingerModelType();

    /// @brief Get the Finger model type.
    /// @return bool Indication of successful
    int getFingerModelType();

    /// @brief Send read msg of the firmware version to finger .
    /// @return bool Indication of successful
    bool readFingerFirmwareVer();

    /// @brief Send read msg of the limit torque to finger .
    /// @return bool Indication of successful
    bool readFingerLimTorque();

    /// @brief Get the Finger limit torque.
    /// @return int Finger limit torque.
    int getFingerLimTorque();

    /// @brief Set finger limit torque.
    /// @param way Limit torque
    /// @return bool Indication of successful
    bool setFingerLimTorque(int way);

    /// @brief Send read msg of the enable status to finger .
    /// @return bool Indication of successful
    bool readFingerEnableStatus();

    /// @brief Get the Finger enable status.
    /// @return bool Indication of successful
    int getFingerEnableStatus();

    /// @brief Set finger enable status.
    /// @param status 0:disable;1:enable
    /// @return bool Indication of successful
    bool setFingerEnableStatus(int status);

    /// @brief Set finger enable on power.
    /// @param status 0:disable;1:enable
    /// @return bool Indication of successful
    bool setFingerEnableOnPower(int status);

    /// @brief Send read msg of the target open angle to finger. Unit:rad
    /// @return bool Indication of successful
    bool readFingerTagOpenRadio();

    /// @brief Get the Finger target open angle radio.
    /// @return float Finger target open angle. Unit:rad.
    float getFingerTagOpenRadio();

    /// @brief Set finger open angel.
    /// @param ang Open angel. Unit:rad
    /// @return bool Indication of successful
    bool setFingerTagOpenRadio(float ang);

    /// @brief Send read msg of the target open angle to finger. Unit:degree
    /// @return bool Indication of successful
    bool readFingerTagOpenAng();

    /// @brief Get the Finger target open angle.
    /// @return float Finger target open angle. Unit:degree.
    float getFingerTagOpenAng();

    /// @brief Set finger open angle.
    /// @param ang Open angle. Unit:degree
    /// @return bool Indication of successful
    bool setFingerTagOpenAng(float ang);

    /// @brief
    /// @return bool Indication of successful
    bool readFingerTagWorkMode();

    /// @brief Get the Finger target work mode.
    /// @return int Finger target work mode.
    int getFingerTagWorkMode();

    /// @brief Set finger work mode.
    /// @param mode Work mode 1:开合模式 2:位置模式
    /// @return bool Indication of successful
    bool setFingerTagWorkMode(int mode);

    /// @brief Send read msg of the target open status to finger.
    /// @return bool Indication of successful
    bool readFingerTagOpenStatus();

    /// @brief Get the Finger target open status.
    /// @return float Finger target open status.
    int getFingerTagOpenStatus();

    /// @brief Set finger open status.
    /// @param mode open status 0:关 1:开
    /// @return bool Indication of successful
    bool setFingerTagOpenStatus(int mode);

    /// @brief Finger save to flash.
    /// @return bool Indication of successful
    bool FingerSysSaveToFlash();

    /// @brief 发送读内存控制表的命令.
    void MemoryTableInit();

    uint32_t ID;
    Driver * busDriver;
    uint16_t MemoryControlTable[CMDLENGTH_FINGER];
protected:
};

#endif // GRIPPER_H
