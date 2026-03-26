/**
 ****************************************************************************************************
 * @file        chassis_driver.h
 * @author      RMP-WL100底盘驱动
 * @version     V1.0
 * @date        2026-02-28
 * @brief       RMP-WL100底盘驱动层接口
 ****************************************************************************************************
 * @attention
 *
 * 实验平台:正点原子 探索者 F407开发板
 * 底盘型号:RMP-WL100 四驱四转向轮式底盘
 *
 ****************************************************************************************************
 */

#ifndef __CHASSIS_DRIVER_H
#define __CHASSIS_DRIVER_H

#include "./SYSTEM/sys/sys.h"
#include "chassis_protocol.h"
#include "../CAN/can.h"

/******************************************************************************************/
/* 错误码定义 */

typedef enum
{
    CHASSIS_OK = 0,              /* 成功 */
    CHASSIS_ERR_PARAM,           /* 参数错误 */
    CHASSIS_ERR_TIMEOUT,         /* 超时 */
    CHASSIS_ERR_CAN              /* CAN发送失败 */
} chassis_err_e;

/******************************************************************************************/
/* 底盘状态结构体 */

typedef struct
{
    chassis_system_status_t   system_status;    /* 系统状态 */
    chassis_motion_feedback_t motion_feedback;  /* 运动反馈 */
    chassis_wheel_speed_t     wheel_speed;      /* 轮速反馈 */
    chassis_wheel_angle_t     wheel_angle;      /* 轮角反馈 */
    chassis_fault_status_t    fault_status;     /* 故障定位 */

    uint32_t last_update_tick;                  /* 最后更新时间戳 */
    uint8_t  is_online;                         /* 在线状态 */
} chassis_state_t;

/******************************************************************************************/
/* 函数声明 */

/* 初始化和状态管理 */
chassis_err_e chassis_driver_init(uint8_t test_mode);
chassis_state_t* chassis_get_state(void);
uint8_t chassis_is_online(void);

/* 控制消息发送 */
chassis_err_e chassis_send_ctrl_enable(chassis_ctrl_mode_e mode);
chassis_err_e chassis_send_motion_mode(chassis_motion_mode_e mode);
chassis_err_e chassis_send_motion_ctrl(chassis_motion_ctrl_t *ctrl);


/* 安全状态查询 */
uint8_t chassis_has_fault(void);     /* 检查驱动器是否故障 (0x103 fault_flag) */
uint8_t chassis_is_estop(void);      /* 检查急停开关是否触发 (0x100 peripheral_state bit0) */

/* 反馈消息处理 */
void chassis_process_feedback(uint32_t can_id, uint8_t *data, uint8_t len);

/******************************************************************************************/

#endif /* __CHASSIS_DRIVER_H */
