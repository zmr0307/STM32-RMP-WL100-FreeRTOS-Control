/**
 ****************************************************************************************************
 * @file        chassis_protocol.h
 * @author      RMP-WL100底盘驱动
 * @version     V1.0
 * @date        2026-02-27
 * @brief       RMP-WL100四驱四转向底盘CAN协议定义
 ****************************************************************************************************
 * @attention
 *
 * 实验平台:正点原子 探索者 F407开发板
 * 底盘型号:RMP-WL100 四驱四转向轮式底盘
 * CAN协议:标准帧, 500Kbps
 *
 ****************************************************************************************************
 */

#ifndef __CHASSIS_PROTOCOL_H
#define __CHASSIS_PROTOCOL_H

#include "./SYSTEM/sys/sys.h"

/******************************************************************************************/
/* CAN ID定义 */

/* 控制消息ID (STM32 -> 底盘) */
#define CAN_ID_CTRL_ENABLE          0x400   /* 控制模式使能 */
#define CAN_ID_MOTION_MODE          0x401   /* 运动模式选择 */
#define CAN_ID_MOTION_CTRL          0x402   /* 运动控制(最常用) */
#define CAN_ID_WHEEL_SPEED_CTRL     0x403   /* 独立轮速控制 */
#define CAN_ID_WHEEL_ANGLE_CTRL     0x404   /* 独立轮角控制 */
#define CAN_ID_DO_CTRL              0x405   /* 外设输出控制 */

/* 反馈消息ID (底盘 -> STM32) */
#define CAN_ID_SYSTEM_STATUS        0x100   /* 系统状态(100ms) */
#define CAN_ID_WHEEL_SPEED          0x101   /* 四轮速度(30ms) */
#define CAN_ID_WHEEL_ANGLE          0x102   /* 四轮角度(30ms) */
#define CAN_ID_FAULT_FEEDBACK       0x103   /* 故障定位(1000ms) */
#define CAN_ID_MOTION_FEEDBACK      0x104   /* 运动状态(20ms) */

/******************************************************************************************/
/* 枚举类型定义 */

/**
 * @brief 控制模式枚举
 */
typedef enum
{
    CTRL_MODE_CAN = 0x01,           /* CAN控制模式 */
    CTRL_MODE_REMOTE = 0x02         /* 遥控器模式 */
} chassis_ctrl_mode_e;

/**
 * @brief 运动模式枚举
 */
typedef enum
{
    MOTION_MODE_FORWARD = 0x00,     /* 正向模式(阿克曼转向) */
    MOTION_MODE_REVERSE = 0x01,     /* 反向模式(横向平移) */
    MOTION_MODE_SPIN = 0x02,        /* 自旋转模式(原地旋转) */
    MOTION_MODE_USER = 0x03         /* 用户自主控制模式 */
} chassis_motion_mode_e;

/******************************************************************************************/
/* 控制消息结构体 (STM32 -> 底盘) */

/**
 * @brief 0x400: 控制模式使能
 * @note  必须先发送此消息使能CAN控制,否则底盘不响应
 */
typedef struct
{
    uint8_t ctrl_mode;              /* 控制模式: 0x01=CAN, 0x02=遥控器 */
    uint8_t reserved[7];            /* 保留字节 */
} chassis_ctrl_enable_t;

/**
 * @brief 0x401: 运动模式选择
 */
typedef struct
{
    uint8_t motion_mode;            /* 运动模式: 0x00~0x03 */
    uint8_t reserved[7];            /* 保留字节 */
} chassis_motion_mode_t;

/**
 * @brief 0x402: 运动控制(最常用)
 * @note  发送周期: 100ms
 *        速度单位: Vx/Vy = mm/s, Vz = rad/s
 *        协议编码: Vx×1000, Vy×1000, Vz×1000 (已修正为1000)
 */
typedef struct
{
    float vx;                       /* X方向速度 (m/s) */
    float vy;                       /* Y方向速度 (m/s) */
    float vz;                       /* 旋转角速度 (rad/s) */
} chassis_motion_ctrl_t;

/******************************************************************************************/
/* 反馈消息结构体 (底盘 -> STM32) */

/**
 * @brief 0x100: 系统状态反馈
 * @note  反馈周期: 1000ms
 */
typedef struct
{
    uint8_t  system_state;          /* 系统状态 (0x00正常, 0x01低电量) */
    uint8_t  ctrl_mode;             /* 控制模式 (0x00待机, 0x01遥控, 0x02指令) */
    uint8_t  peripheral_state;      /* 外设状态 (bit0:急停, bit1:避障, bit2:接近开关, bit3:安全触边) */
    uint8_t  soc;                   /* 电池电量百分比 (0~100) */
    uint16_t battery_voltage;       /* 电池总电压 (10mV) */
    int16_t  battery_current;       /* 电池充放电电流 (10mA) */
} chassis_system_status_t;

/**
 * @brief 0x101: 四轮速度反馈
 * @note  反馈周期: 30ms
 *        速度单位: mm/s
 */
typedef struct
{
    int16_t wheel1_speed;           /* 1号轮速度 */
    int16_t wheel2_speed;           /* 2号轮速度 */
    int16_t wheel3_speed;           /* 3号轮速度 */
    int16_t wheel4_speed;           /* 4号轮速度 */
} chassis_wheel_speed_t;

/**
 * @brief 0x102: 四轮角度反馈
 * @note  反馈周期: 30ms
 *        角度单位: 0.01度
 */
typedef struct
{
    int16_t wheel1_angle;           /* 1号轮角度 */
    int16_t wheel2_angle;           /* 2号轮角度 */
    int16_t wheel3_angle;           /* 3号轮角度 */
    int16_t wheel4_angle;           /* 4号轮角度 */
} chassis_wheel_angle_t;

/**
 * @brief 0x104: 运动状态反馈
 * @note  反馈周期: 20ms
 *        速度单位: Vx/Vy = mm/s, Vz = rad/s
 */
typedef struct
{
    int16_t vx_fb;                  /* 实际X方向速度 */
    int16_t vy_fb;                  /* 实际Y方向速度 */
    int16_t vz_fb;                  /* 实际旋转角速度 */
} chassis_motion_feedback_t;

/**
 * @brief 0x103: 故障定位回馈
 * @note  反馈周期: 1000ms
 */
typedef struct
{
    uint8_t  fault_flag;            /* 驱动器故障标志位 (0:正常, 1:出现故障) */
    uint8_t  walk_fault;            /* 行走驱动器故障报警 (bit0-bit3对应该4号电机) */
    uint8_t  steer_fault;           /* 转向驱动器故障报警 (bit0-bit3对应该5-8号电机) */
    uint8_t  walk_comm_fault;       /* 行走驱动器通讯报警 (bit0-bit3对应该4号电机) */
    uint8_t  steer_comm_fault;      /* 转向驱动器通讯报警 (bit0-bit3对应该5-8号电机) */
} chassis_fault_status_t;

/******************************************************************************************/

#endif /* __CHASSIS_PROTOCOL_H */
