/**
 ****************************************************************************************************
 * @file        chassis_driver.c
 * @author      RMP-WL100底盘驱动
 * @version     V1.0
 * @date        2026-02-28
 * @brief       RMP-WL100底盘驱动层实现
 ****************************************************************************************************
 * @attention
 *
 * 实验平台:正点原子 探索者 F407开发板
 * 底盘型号:RMP-WL100 四驱四转向轮式底盘
 *
 ****************************************************************************************************
 */

#include "chassis_driver.h"
#include "string.h"

/******************************************************************************************/
/* 全局变量 */

static chassis_state_t g_chassis_state;  /* 底盘状态 */

/******************************************************************************************/
/* 内部函数声明 */

static void encode_ctrl_enable(chassis_ctrl_mode_e mode, uint8_t *data);
static void encode_motion_mode(chassis_motion_mode_e mode, uint8_t *data);
static void encode_motion_ctrl(chassis_motion_ctrl_t *ctrl, uint8_t *data);


static void decode_system_status(uint8_t *data, chassis_system_status_t *status);
static void decode_wheel_speed(uint8_t *data, chassis_wheel_speed_t *speed);
static void decode_wheel_angle(uint8_t *data, chassis_wheel_angle_t *angle);
static void decode_motion_feedback(uint8_t *data, chassis_motion_feedback_t *feedback);

/******************************************************************************************/
/* 编码函数实现 */

/**
 * @brief       编码控制模式使能消息(0x400)
 * @param       mode: 控制模式
 * @param       data: 输出的8字节数据
 * @retval      无
 */
static void encode_ctrl_enable(chassis_ctrl_mode_e mode, uint8_t *data)
{
    memset(data, 0, 8);
    data[0] = (uint8_t)mode;
}

/**
 * @brief       编码运动模式选择消息(0x401)
 * @param       mode: 运动模式
 * @param       data: 输出的8字节数据
 * @retval      无
 */
static void encode_motion_mode(chassis_motion_mode_e mode, uint8_t *data)
{
    memset(data, 0, 8);
    data[0] = (uint8_t)mode;
}

/**
 * @brief       编码运动控制消息(0x402)
 * @param       ctrl: 运动控制结构体
 * @param       data: 输出的8字节数据
 * @retval      无
 */
static void encode_motion_ctrl(chassis_motion_ctrl_t *ctrl, uint8_t *data)
{
    int16_t vx_encoded, vy_encoded, vz_encoded;
    
    /* 防飞车安全限幅 (绝对最大 2.0 m/s 或 2.0 rad/s) */
    float safe_vx = ctrl->vx;
    float safe_vy = ctrl->vy;
    float safe_vz = ctrl->vz;
    
    if (safe_vx > 2.0f) safe_vx = 2.0f; else if (safe_vx < -2.0f) safe_vx = -2.0f;
    if (safe_vy > 2.0f) safe_vy = 2.0f; else if (safe_vy < -2.0f) safe_vy = -2.0f;
    if (safe_vz > 2.0f) safe_vz = 2.0f; else if (safe_vz < -2.0f) safe_vz = -2.0f;

    /* 浮点转整数(协议编码) */
    vx_encoded = (int16_t)(safe_vx * 1000.0f);  /* m/s -> 协议值 */
    vy_encoded = (int16_t)(safe_vy * 1000.0f);
    vz_encoded = (int16_t)(safe_vz * 1000.0f);  /* rad/s -> 协议值. 官方必须是*1000 */

    /* 拆分成字节(高字节在前) */
    data[0] = (vx_encoded >> 8) & 0xFF;
    data[1] = vx_encoded & 0xFF;

    data[2] = (vy_encoded >> 8) & 0xFF;
    data[3] = vy_encoded & 0xFF;

    data[4] = (vz_encoded >> 8) & 0xFF;
    data[5] = vz_encoded & 0xFF;

    data[6] = 0;  /* 保留 */
    data[7] = 0;  /* 保留 */
}



/******************************************************************************************/
/* 解码函数实现 */

/**
 * @brief       解码系统状态消息(0x100)
 * @param       data: 8字节CAN数据
 * @param       status: 输出的系统状态结构体
 * @retval      无
 */
static void decode_system_status(uint8_t *data, chassis_system_status_t *status)
{
    status->system_state    = data[0];                        /* Byte[0]: 系统状态 */
    status->ctrl_mode       = data[1];                        /* Byte[1]: 控制模式 */
    status->peripheral_state = data[2];                       /* Byte[2]: 外设状态 */
    status->soc             = data[3];                        /* Byte[3]: SOC */
    status->battery_voltage = (data[4] << 8) | data[5];       /* Byte[4-5]: 电压(10mV) */
    status->battery_current = (int16_t)((data[6] << 8) | data[7]); /* Byte[6-7]: 电流(10mA) */
}

/**
 * @brief       解码四轮速度消息(0x101)
 * @param       data: 8字节CAN数据
 * @param       speed: 输出的轮速结构体
 * @retval      无
 */
static void decode_wheel_speed(uint8_t *data, chassis_wheel_speed_t *speed)
{
    speed->wheel1_speed = (data[0] << 8) | data[1];
    speed->wheel2_speed = (data[2] << 8) | data[3];
    speed->wheel3_speed = (data[4] << 8) | data[5];
    speed->wheel4_speed = (data[6] << 8) | data[7];
}

/**
 * @brief       解码四轮角度消息(0x102)
 * @param       data: 8字节CAN数据
 * @param       angle: 输出的轮角结构体
 * @retval      无
 */
static void decode_wheel_angle(uint8_t *data, chassis_wheel_angle_t *angle)
{
    angle->wheel1_angle = (data[0] << 8) | data[1];
    angle->wheel2_angle = (data[2] << 8) | data[3];
    angle->wheel3_angle = (data[4] << 8) | data[5];
    angle->wheel4_angle = (data[6] << 8) | data[7];
}

/**
 * @brief       解码运动状态反馈消息(0x104)
 * @param       data: 8字节CAN数据
 * @param       feedback: 输出的运动反馈结构体
 * @retval      无
 */
static void decode_motion_feedback(uint8_t *data, chassis_motion_feedback_t *feedback)
{
    feedback->vx_fb = (int16_t)((data[0] << 8) | data[1]);
    feedback->vy_fb = (int16_t)((data[2] << 8) | data[3]);
    feedback->vz_fb = (int16_t)((data[4] << 8) | data[5]);
}

/**
 * @brief       解码故障定位反馈消息(0x103)
 * @param       data: 8字节CAN数据
 * @param       fault: 输出的故障状态结构体
 * @retval      无
 */
static void decode_fault_status(uint8_t *data, chassis_fault_status_t *fault)
{
    fault->fault_flag       = data[0];
    fault->walk_fault       = data[1];
    fault->steer_fault      = data[2];
    fault->walk_comm_fault  = data[3];
    fault->steer_comm_fault = data[4];
}

/******************************************************************************************/
/* 外部接口函数实现 */

/**
 * @brief       初始化底盘驱动
 * @param       test_mode: 0=正常模式(连接底盘), 1=回环测试模式(自测)
 * @retval      CHASSIS_OK: 成功
 */
chassis_err_e chassis_driver_init(uint8_t test_mode)
{
    /* 清空底盘状态 */
    memset(&g_chassis_state, 0, sizeof(chassis_state_t));

    /* 初始化CAN */
    if (test_mode)
    {
        /* 回环测试模式(自己发自己收,用于调试) */
        /* 波特率=42M/(6*(1+8+5))=500Kbps */
        can_init(CAN_SJW_1TQ, CAN_BS2_5TQ, CAN_BS1_8TQ, 6, CAN_MODE_LOOPBACK);
    }
    else
    {
        /* 正常模式(连接底盘) */
        /* 波特率=42M/(6*(1+8+5))=500Kbps */
        can_init(CAN_SJW_1TQ, CAN_BS2_5TQ, CAN_BS1_8TQ, 6, CAN_MODE_NORMAL);
    }

    return CHASSIS_OK;
}

/**
 * @brief       获取底盘状态指针
 * @param       无
 * @retval      底盘状态结构体指针
 */
chassis_state_t* chassis_get_state(void)
{
    return &g_chassis_state;
}

/**
 * @brief       检查底盘是否在线
 * @param       无
 * @retval      1: 在线, 0: 离线
 */
uint8_t chassis_is_online(void)
{
    uint32_t timeout = 500;  /* 超时时间500ms */

    /* 检查是否超时 */
    if (HAL_GetTick() - g_chassis_state.last_update_tick > timeout)
    {
        g_chassis_state.is_online = 0;  /* 超时,设为离线 */
    }

    return g_chassis_state.is_online;
}

/**
 * @brief       发送控制模式使能消息(0x400)
 * @param       mode: 控制模式
 * @retval      CHASSIS_OK: 成功, CHASSIS_ERR_CAN: CAN发送失败
 */
chassis_err_e chassis_send_ctrl_enable(chassis_ctrl_mode_e mode)
{
    uint8_t data[8];

    encode_ctrl_enable(mode, data);

    if (can_send_msg(CAN_ID_CTRL_ENABLE, data, 8) != 0)
    {
        return CHASSIS_ERR_CAN;
    }

    return CHASSIS_OK;
}

/**
 * @brief       发送运动模式选择消息(0x401)
 * @param       mode: 运动模式
 * @retval      CHASSIS_OK: 成功, CHASSIS_ERR_CAN: CAN发送失败
 */
chassis_err_e chassis_send_motion_mode(chassis_motion_mode_e mode)
{
    uint8_t data[8];

    encode_motion_mode(mode, data);

    if (can_send_msg(CAN_ID_MOTION_MODE, data, 8) != 0)
    {
        return CHASSIS_ERR_CAN;
    }

    return CHASSIS_OK;
}

/**
 * @brief       发送运动控制消息(0x402)
 * @param       ctrl: 运动控制结构体指针
 * @retval      CHASSIS_OK: 成功, CHASSIS_ERR_PARAM: 参数错误, CHASSIS_ERR_CAN: CAN发送失败
 */
chassis_err_e chassis_send_motion_ctrl(chassis_motion_ctrl_t *ctrl)
{
    uint8_t data[8];

    /* 参数检查 */
    if (ctrl == NULL)
    {
        return CHASSIS_ERR_PARAM;
    }

    encode_motion_ctrl(ctrl, data);

    if (can_send_msg(CAN_ID_MOTION_CTRL, data, 8) != 0)
    {
        return CHASSIS_ERR_CAN;
    }

    return CHASSIS_OK;
}


/**
 * @brief       处理底盘反馈消息
 * @param       can_id: CAN消息ID
 * @param       data: 8字节CAN数据
 * @param       len: 数据长度
 * @retval      无
 */
void chassis_process_feedback(uint32_t can_id, uint8_t *data, uint8_t len)
{
    if (len != 8) return;  /* 数据长度检查 */

    switch (can_id)
    {
        case CAN_ID_SYSTEM_STATUS:
            decode_system_status(data, &g_chassis_state.system_status);
            break;

        case CAN_ID_WHEEL_SPEED:
            decode_wheel_speed(data, &g_chassis_state.wheel_speed);
            break;

        case CAN_ID_WHEEL_ANGLE:
            decode_wheel_angle(data, &g_chassis_state.wheel_angle);
            break;

        case CAN_ID_FAULT_FEEDBACK:
            decode_fault_status(data, &g_chassis_state.fault_status);
            break;

        case CAN_ID_MOTION_FEEDBACK:
            decode_motion_feedback(data, &g_chassis_state.motion_feedback);
            break;

        default:
            return;
    }

    /* 更新时间戳和在线状态 */
    g_chassis_state.last_update_tick = HAL_GetTick();
    g_chassis_state.is_online = 1;
}

/******************************************************************************************/
