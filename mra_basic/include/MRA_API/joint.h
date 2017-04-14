// joint.h
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
#ifndef JOINT_H
#define JOINT_H

#include "driver.h"

#define CMDLENGTH           160 // 内存控制表长度（单位：字节）

/// 模块类型宏定义
#define MODEL_TYPE_M14E     0x011
#define MODEL_TYPE_LIFT     0x040
#define MODEL_TYPE_M14      0x010
#define MODEL_TYPE_M17      0x002
#define MODEL_TYPE_M17E     0x021
#define MODEL_TYPE_M20      0x030

/// 模块减速比宏定义
#define GEAR_RATIO_LIFT     160
#define GEAR_RATIO_M14E     100
#define GEAR_RATIO_M14      100
#define GEAR_RATIO_M17      100
#define GEAR_RATIO_M17E     120
#define GEAR_RATIO_M20      160

/// 内存控制表宏定义
#define CMDMAP_LEN          160 /**< Total length of memory control table */
#define CMDMAP_INDLEN       10  /**< Index number of memory control table */
#define CMDMAP_SUBLEN       16  /**< Subindex number of memory control table */

/// 运动控制方式宏定义
#define MODE_OPEN           0   /**< Open-loop mode */
#define MODE_CURRENT        1   /**< Current mode */
#define MODE_SPEED          2   /**< Speed mode */
#define MODE_POSITION       3   /**< Position mode */

/// 位置显示格式
#define JOINT_ANGLE             0x01    /**< Use degree */
#define JOINT_RADIAN            0x02    /**< Use radian */

/// System state
#define SYS_ID              0x01    /**< Driver identifier */
#define SYS_MODEL_TYPE      0x02    /**< Driver type */
#define SYS_FW_VERSION      0x03    /**< Fireware version */
#define SYS_ERROR           0x04    /**< Error code */
#define SYS_VOLTAGE         0x05    /**< System voltage */
#define SYS_TEMP            0x06    /**< System temprature */
#define SYS_REDU_RATIO      0x07    /**< Reduction ratio */

#define SYS_BAUDRATE_CAN        0x09    /**< Baudrate of CAN Bus */
#define SYS_ENABLE_DRIVER       0x0a    /**< Enabled flag of driver */
#define SYS_ENABLE_ON_POWER     0x0b    /**< Enabled on power flag of driver */
#define SYS_SAVE_TO_FLASH       0x0c    /**< Flag of data saving */

#define SYS_SET_ZERO_POS        0x0e    /**< Set current position to zero */
#define SYS_CLEAR_ERROR         0x0f    /**< Clear error flag */

#define SYS_CURRENT_L           0x10    /**< Lower 16 bits of current (mA) */
#define SYS_CURRENT_H           0x11    /**< Higher 16 bits of current (mA) */
#define SYS_SPEED_L             0x12    /**< Lower 16 bits of current speed (units/s) */
#define SYS_SPEED_H             0x13    /**< Higher 16 bits of current speed (units/s) */
#define SYS_POSITION_L          0x14    /**< Lower 16 bits of current position (units) */
#define SYS_POSITION_H          0x15    /**< Higher 16 bits of current position (units) */
#define SYS_POTEN_VALUE         0x16    /**< Reading of potentiometer */
#define SYS_ZERO_POS_OFFSET_L   0x17    /**< Lower 16 bits of zero position offset (units) */
#define SYS_ZERO_POS_OFFSET_H   0x18    /**< Higher 16 bits of zero position offset (units) */

/// Motor information
#define MOT_RES                 0x20    /**< Internal resistance of motor */
#define MOT_INDUC               0x21    /**< Inductance of motor */
#define MOT_RATED_VOL           0x22    /**< Rated voltage of motor */
#define MOT_RATED_CUR           0x23    /**< Rated current of motor */
#define MOT_ENC_LINES           0x400   /**< Lines of encoder */

#define MOT_ST_DAT              0x26    /// 绝对编码器单圈数据
#define MOT_MT_DAT              0x27    /// 绝对编码器多圈数据
#define MOT_ENC_STA             0x28    /// 绝对编码器状态寄存器


/// Control target
#define TAG_WORK_MODE           0x30    /**< Work mode. 0: open-loop mode; 1: current mode; 2: speed mode; 3: position mode */
#define TAG_OPEN_PWM            0x31    /**< PWM in open-loop mode (0~100) */
#define TAG_CURRENT_L           0x32    /**< Lower 16 bits of target current (mA) */
#define TAG_CURRENT_H           0x33    /**< Higher 16 bits of target current (mA) */
#define TAG_SPEED_L             0x34    /**< Lower 16 bits of target speed (units/s) */
#define TAG_SPEED_H             0x35    /**< Higher 16 bits of target speed (units/s) */
#define TAG_POSITION_L          0x36    /**< Lower 16 bits of target poistion (units) */
#define TAG_POSITION_H          0x37    /**< Higher 16 bits of target poistion (units) */

/// Control threshold
#define LIT_MAX_CURRENT         0x40    /**< Maximal current (mA) */
#define LIT_MAX_SPEED           0x41    /**< Maximal speed (rmp) */
#define LIT_MAX_ACC             0x42    /**< Maximal acceleration (rpm/s) */
#define LIT_MIN_POSITION_L      0x43    /**< Lower 16 bits of minimal position */
#define LIT_MIN_POSITION_H      0x44    /**< Higher 16 bits of minimal position */
#define LIT_MAX_POSITION_L      0x45    /**< Lower 16 bits of maximal position */
#define LIT_MAX_POSITION_H      0x46    /**< Higher 16 bits of maximal position */

/// Closed loop
#define SEV_PARAME_LOCKED       0x50    /**< Flag of parameter lock in three closed loops */
#define SEV_CURRENT_P           0x51    /**< Parameter P in current loop */
#define SEV_CURRENT_I           0x52    /**< Parameter I in current loop */
#define SEV_CURRENT_D           0x53    /**< Parameter D in current loop */
#define SEV_SPEED_P             0x54    /**< Parameter P in speed loop */
#define SEV_SPEED_I             0x55    /**< Parameter I in speed loop */
#define SEV_SPEED_D             0x56    /**< Parameter D in speed loop */
#define SEV_SPEED_DS            0x57    /**< Dead zone in speed loop */
#define SEV_POSITION_P          0x58    /**< Parameter P in position loop */
#define SEV_POSITION_I          0x59    /**< Parameter I in position loop */
#define SEV_POSITION_D          0x5a    /**< Parameter D in position loop */
#define SEV_POSITION_DS         0x5b    /**< Dead zone in position loop */

#define M_CURRENT_P             0x61    /// 电流环P参数
#define M_CURRENT_I             0x62    /// 电流环I参数
#define M_CURRENT_D             0x63    /// 电流环D参数
#define M_SPEED_P               0x64    /// 速度环P参数
#define M_SPEED_I               0x65    /// 速度环I参数
#define M_SPEED_D               0x66    /// 速度环D参数
#define M_SPEED_DS              0x67    /// 速度P死区
#define M_POSITION_P            0x68    /// 位置环P参数
#define M_POSITION_I            0x69    /// 位置环I参数
#define M_POSITION_D            0x6A    /// 位置环D参数
#define M_POSITION_DS           0x6B    /// 位置P死区

#define L_CURRENT_P             0x71    /// 电流环P参数
#define L_CURRENT_I             0x72    /// 电流环I参数
#define L_CURRENT_D             0x73    /// 电流环D参数
#define L_SPEED_P               0x74    /// 速度环P参数
#define L_SPEED_I               0x75    /// 速度环I参数
#define L_SPEED_D               0x76    /// 速度环D参数
#define L_SPEED_DS              0x77    /// 速度P死区
#define L_POSITION_P            0x78    /// 位置环P参数
#define L_POSITION_I            0x79    /// 位置环I参数
#define L_POSITION_D            0x7A    /// 位置环D参数
#define L_POSITION_DS           0x7B    /// 位置P死区

/// 刹车控制命令
#define BRAKE_RELEASE_CMD       0x80    /// 刹车释放命令，0-保持制动，1-释放刹车
#define BRAKE_STATE             0x81    /// 刹车状态，0-保持制动，1-释放刹车

/// 示波器模块子索引地址定义
#define SCP_MASK        0x90    /// 记录对象标志MASK
#define SCP_REC_TIM     0x91    /// 记录时间间隔（对10kHZ的分频值）
#define SCP_TAGCUR_L    0x92    /// 目标电流数据集
#define SCP_TAGCUR_H    0x93    /// 目标电流数据集
#define SCP_MEACUR_L    0x94    /// 实际电流数据集
#define SCP_MEACUR_H    0x95    /// 实际电流数据集
#define SCP_TAGSPD_L    0x96    /// 目标速度数据集
#define SCP_TAGSPD_H    0x97    /// 目标速度数据集
#define SCP_MEASPD_L    0x98    /// 实际速度数据集
#define SCP_MEASPD_H    0x99    /// 实际速度数据集
#define SCP_TAGPOS_L    0x9A    /// 目标位置数据集
#define SCP_TAGPOS_H    0x9B    /// 目标位置数据集
#define SCP_MEAPOS_L    0x9C    /// 实际位置数据集
#define SCP_MEAPOS_H    0x9D    /// 实际位置数据集

#define MASK_TAGCUR     0x01    /// 记录目标电流MASK
#define MASK_MEACUR     0x02    /// 记录实际电流MASK
#define MASK_TAGSPD     0x04    /// 记录目标速度MASK
#define MASK_MEASPD     0x08    /// 记录实际速度MASK
#define MASK_TAGPOS     0x10    /// 记录目标位置MASK
#define MASK_MEAPOS     0x20    /// 记录实际位置MASK

/// MASK definition of errors
#define ERROR_MASK_OVER_CURRENT     0x0001      /**< Error code of over current */
#define ERROR_MASK_OVER_VOLTAGE     0x0002      /**< Error code of over voltage */
#define ERROR_MASK_UNDER_VOLTAGE    0x0004      /**< Error code of under voltage */
#define ERROR_MASK_OVER_TEMP        0x0008      /**< Error code of over temprature */
#define ERROR_MASK_HALL             0x0010      /**< Error code of HALL  */
#define ERROR_MASK_ENCODER          0x0020      /**< Error code of encoder */
#define ERROR_MASK_POTEN            0x0040      /**< Error code of potentiometer */
#define ERROR_MASK_CURRENT_INIT     0x0080      /**< Error code of current detection */
#define ERROR_MASK_FUSE             0x0100      /**< Error code of fuse */

class Joint
{
public:
    /// @brief 构造函数
    /// @param ID 关节ID
    /// @param d 该关节所在的总线
    Joint(uint32_t ID, Driver * d);

    /// @brief Save to flash.
    /// @return bool Indication of successful
    bool setSaveToFlash();

    /// @brief Set joint enable.
    /// @param enabled true - 使能, false - 失能
    /// @return bool Indication of successful
    bool setEnable(bool enabled);

    /// @brief Update joint enable.
    /// @return bool Indication of successful
    bool updateEnable();

    /// @brief Get joint enable status.
    /// @return bool enable status
    bool getEnableStatus();

    /// @brief Clear error.
    /// @return bool Indication of successful
    bool clearErrStatus();

    /// @brief Update joint error status.
    /// @return bool Indication of successful
    bool updateErrStatus();

    /// @brief Get joint error status.
    /// @return uint8_t Error code
    uint8_t getErrStatus();

    /// @brief Set joint work mode.
    /// @param workMode 0:开环 1:电流 2:速度 3:位置
    /// @return bool Indication of successful
    bool setWorkMode(uint8_t workMode);

    /// @brief Update joint work mode.
    /// @return bool Indication of successful
    bool updateWorkMode();

    /// @brief Get joint work mode.
    /// @return uint8_t 0:开环 1:电流 2:速度 3:位置
    uint8_t getWorkMode();

    /// @brief Set joint zero position.
    /// @return bool Indication of successful
    bool setZeroPos();

    /// @brief Set joint ID.
    /// @param newid 新的ID号
    /// @return bool Indication of successful
    bool setID(uint16_t newid);

    /// @brief Update the current I of joint.
    /// @return bool Indication of successful
    bool updateCurI();

    /// @brief Obtain the current I of joint.
    /// @return int I
    int getCurI();

    /// @brief Update the current speed of joint.
    /// @return bool Indication of successful
    bool updateCurSpeed();

    /// @brief Obtain the current speed of joint.
    /// @return float speed (Unit: rpm)
    float getCurSpeed();

    /// @brief Update the current position of joint.
    /// @return bool Indication of successful
    bool updateCurPos();

    /// @brief Obtain the current position.
    /// @param cmd JOINT_ANGLE or JOINT_RADIAN
    /// @return float position
    float getCurPos(int cmd);

    /// @brief Set the target Open PWM of joint.
    /// @param pwm Motor PWM (Unit: %)
    /// @return bool Indication of successful
    bool setTagPWM(uint8_t pwm);

    /// @brief Update the target PWM of joint.
    /// @return bool Indication of successful
    bool updateTagPWM();

    /// @brief Obtain the target Open PWM of joint.
    /// @return int Motor PWM (Unit: %)
    uint8_t getTagPWM();

    /// @brief Set the target I of joint.
    /// @param curI Joint I
    /// @return bool Indication of successful
    bool setTagI(int curI);

    /// @brief Update the target I of joint.
    /// @return bool Indication of successful
    bool updateTagI();

    /// @brief Obtain the target I of joint.
    /// @return int I
    int getTagI();

    /// @brief Set the target speed of joint.
    /// @param speed Joint speed (Unit: rpm)
    /// @return bool Indication of successful
    bool setTagSpeed(float speed);

    /// @brief Update the target speed of joint.
    /// @return bool Indication of successful
    bool updateTagSpeed();

    /// @brief Obtain the target speed of joint.
    /// @return float speed (Unit: rpm)
    float getTagSpeed();

    /// @brief Set the target position of joint.
    /// @param angle position
    /// @param cmd JOINT_ANGLE or JOINT_RADIAN
    /// @return bool Indication of successful
    bool setTagPos(float angle, int cmd);

    /// @brief Update the target position of joint.
    /// @return bool Indication of successful
    bool updateTagPos();

    /// @brief Obtain the target position.
    /// @param cmd JOINT_ANGLE or JOINT_RADIAN
    /// @return float position
    float getTagPos(int cmd);

    /// @brief Update the gear radio type.
    /// @return bool Indication of successful
    bool updateGearRadioType();

    /// @brief Obtain the gear radio type.
    /// @return uint16_t gear radio type
    uint16_t getGearRadioType();

    /// @brief Update the model type.
    /// @return bool Indication of successful
    bool updateModelType();

    /// @brief Obtain the model type.
    /// @return uint16_t model type
    uint16_t getModelType();

    /// @brief 发送读内存控制表的命令.
    void MemoryTableInit();

    /// @brief
    /// @param
    /// @return bool Indication of successful
    bool setScpMask(uint16_t mask);

    /// @brief
    /// @return bool Indication of successful
    bool updateScpMask();

    /// @brief
    /// @return int
    uint16_t getScpMask();

    uint32_t ID; // 模块类型
    Driver * busDriver; // 模块所在的总线
    uint16_t MemoryControlTable[CMDLENGTH]; // 模块内存控制表
protected:

};

#endif // JOINT_H
