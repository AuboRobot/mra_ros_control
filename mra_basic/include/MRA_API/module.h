#ifndef MODULE_H
#define MODULE_H

#include "driver.h"

class Module
{
public:
    /// @brief 构造函数
    /// @param ID 关节ID
    /// @param d 该关节所在的总线
    Module(uint32_t ID, Driver * d);
    virtual ~Module();

    /// @brief 发送读内存控制表的命令.
    virtual void MemoryTableInit() = 0;

    uint32_t ID;        // 模块类型
    Driver * busDriver; // 模块所在的总线
};

#endif // MODULE_H
