#ifndef MANDA_CAN_CONTROL_
#define MANDA_CAN_CONTROL_

#define CMD_ACK_FINISH                      0x00
#define CMD_ACK_FAIL                        0x01

#include <rclcpp/rclcpp.hpp>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <thread>

#include "manda_can_control/msg/battery_fb.hpp"
#include "manda_can_control/msg/control_mode_ctrl.hpp"
#include "manda_can_control/msg/fault_fb.hpp"
#include "manda_can_control/msg/motion_ctrl.hpp"
#include "manda_can_control/msg/motion_fb.hpp"
#include "manda_can_control/msg/speed_ctrl.hpp"
#include "manda_can_control/msg/speed_fb.hpp"
#include "manda_can_control/msg/steer_ctrl.hpp"
#include "manda_can_control/msg/steer_fb.hpp"
#include "manda_can_control/msg/systemstate_fb.hpp"
#include "manda_can_control/srv/do_ctrl.hpp"
#include "manda_can_control/srv/motion_mode.hpp"

class MandaCanControl : public rclcpp::Node
{
public:
    MandaCanControl();
    ~MandaCanControl();
    void Run();

private:
    int InitRosNode();
    int InitCanSocket();
    static void CanReceiveThread(void* ptr);
    void DecodeCanFrameData(const struct can_frame* frame);

    void MotionModeCallback(
        const std::shared_ptr<manda_can_control::srv::MotionMode::Request> req,
        std::shared_ptr<manda_can_control::srv::MotionMode::Response> res);
    void MotionControlCallback(const manda_can_control::msg::MotionCtrl::SharedPtr motion);
    void SpeedControlCallback(const manda_can_control::msg::SpeedCtrl::SharedPtr speed);
    void SteerControlCallback(const manda_can_control::msg::SteerCtrl::SharedPtr angle);
    void DoCallback(
        const std::shared_ptr<manda_can_control::srv::DoCtrl::Request> req,
        std::shared_ptr<manda_can_control::srv::DoCtrl::Response> res);

    int SendCanFrame(canid_t id, uint8_t *data, uint32_t frame_num);


private:
    int                     canSocketFd_;
    struct sockaddr_can     addr_;
    struct ifreq            ifr_;
    const char              *strCanDev_ = "can0";
    bool                    isCanRecvThreadRunning_;

    rclcpp::Publisher<manda_can_control::msg::SpeedFb>::SharedPtr wheel_fb_pub_;
    rclcpp::Publisher<manda_can_control::msg::SteerFb>::SharedPtr steer_fb_pub_;
    rclcpp::Publisher<manda_can_control::msg::BatteryFb>::SharedPtr battery_fb_pub_;
    rclcpp::Publisher<manda_can_control::msg::SystemstateFb>::SharedPtr system_state_fb_pub_;
    rclcpp::Publisher<manda_can_control::msg::MotionFb>::SharedPtr motion_fb_pb_;
    rclcpp::Publisher<manda_can_control::msg::FaultFb>::SharedPtr fault_fb_pb_;

    rclcpp::Subscription<manda_can_control::msg::MotionCtrl>::SharedPtr ctrl_cmd_sub_;
    rclcpp::Subscription<manda_can_control::msg::SpeedCtrl>::SharedPtr ctrl_speed_sub_;
    rclcpp::Subscription<manda_can_control::msg::SteerCtrl>::SharedPtr ctrl_steer_sub_;

    rclcpp::Service<manda_can_control::srv::DoCtrl>::SharedPtr srv_do_sub_;
    rclcpp::Service<manda_can_control::srv::MotionMode>::SharedPtr srv_motion_mode_;

};

#endif
