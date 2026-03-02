#include "manda_can_control/manda_can_control.h"
#include <syslog.h>
#include <signal.h>
#include <execinfo.h>
 #include <cerrno>
 #include <cstring>
 #include <functional>
 #include <unistd.h>


MandaCanControl::MandaCanControl()
    : rclcpp::Node("manda_can_control_node")
{

}

MandaCanControl::~MandaCanControl()
{
    
}

void MandaCanControl::Run()
{
    if(InitRosNode() < 0 || InitCanSocket() < 0)
    {
        rclcpp::shutdown();
        return;
    }
    std::thread t(MandaCanControl::CanReceiveThread, this);
	t.detach();
}

int MandaCanControl::InitCanSocket()
{
    int nEnable = 1;
    // Open the CAN socket
    if ((canSocketFd_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
        RCLCPP_ERROR(this->get_logger(), "CAN socket open error");
        return -1;
    }
    // Allow CAN FD frames (default:off)
    if ((::setsockopt(canSocketFd_, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &nEnable, sizeof(nEnable)) < 0)) {
        RCLCPP_ERROR(this->get_logger(), "Setsockopt CAN FD error");
        return -2;
    }
    // Set the can "can0"
    std::strncpy(ifr_.ifr_name, strCanDev_, IFNAMSIZ);
    ifr_.ifr_ifindex = if_nametoindex(ifr_.ifr_name);
    if (!ifr_.ifr_ifindex) {
        RCLCPP_ERROR(this->get_logger(), "If_nametoindex CAN name error");
        return -3;
    }
    // name -> if_index mapping
    if (::ioctl(canSocketFd_, SIOCGIFINDEX, &ifr_) < 0) {
        RCLCPP_ERROR(this->get_logger(), "Ioctl CAN if_index mapping error");
        return -4;
    }
    // Bind the socket to the network interface
    memset(&addr_, 0, sizeof(addr_));
    addr_.can_family     = AF_CAN;
    addr_.can_ifindex    = ifr_.ifr_ifindex;
    if (::bind(canSocketFd_, reinterpret_cast<struct sockaddr *>(&addr_), sizeof(addr_)) < 0) {
        RCLCPP_ERROR(this->get_logger(), "Bind CAN socket error");
        return -5;
    }
    RCLCPP_INFO(this->get_logger(), "Init Socket CAN success.");
    return 0;
}

int MandaCanControl::InitRosNode()
{
    wheel_fb_pub_ = this->create_publisher<manda_can_control::msg::SpeedFb>("/speed_fb", rclcpp::QoS(5));
    steer_fb_pub_ = this->create_publisher<manda_can_control::msg::SteerFb>("/steer_fb", rclcpp::QoS(5));
    battery_fb_pub_ = this->create_publisher<manda_can_control::msg::BatteryFb>("/battery_fb", rclcpp::QoS(5));
    system_state_fb_pub_ = this->create_publisher<manda_can_control::msg::SystemstateFb>("/system_state_fb", rclcpp::QoS(5));
    fault_fb_pb_ = this->create_publisher<manda_can_control::msg::FaultFb>("/fault_fb", rclcpp::QoS(5));
    motion_fb_pb_ = this->create_publisher<manda_can_control::msg::MotionFb>("/motion_fb", rclcpp::QoS(5));

    ctrl_cmd_sub_ = this->create_subscription<manda_can_control::msg::MotionCtrl>(
        "/motion_control", rclcpp::QoS(3),
        std::bind(&MandaCanControl::MotionControlCallback, this, std::placeholders::_1));
    ctrl_speed_sub_ = this->create_subscription<manda_can_control::msg::SpeedCtrl>(
        "/speed_ctrl", rclcpp::QoS(3),
        std::bind(&MandaCanControl::SpeedControlCallback, this, std::placeholders::_1));
    ctrl_steer_sub_ = this->create_subscription<manda_can_control::msg::SteerCtrl>(
        "/steer_ctrl", rclcpp::QoS(3),
        std::bind(&MandaCanControl::SteerControlCallback, this, std::placeholders::_1));

    srv_motion_mode_ = this->create_service<manda_can_control::srv::MotionMode>(
        "/motion_mode",
        std::bind(&MandaCanControl::MotionModeCallback, this, std::placeholders::_1, std::placeholders::_2));
    srv_do_sub_ = this->create_service<manda_can_control::srv::DoCtrl>(
        "/do_ctrl",
        std::bind(&MandaCanControl::DoCallback, this, std::placeholders::_1, std::placeholders::_2));
    return 0;
}

void MandaCanControl::CanReceiveThread(void* ptr)
{
    fd_set fds;
    struct timeval timeout;
    struct can_frame frame;
    int nBytes = 0;
    MandaCanControl* p = (MandaCanControl*)ptr;
    while(rclcpp::ok())
    {
		FD_ZERO(&fds);
        FD_SET(p->canSocketFd_, &fds);
		timeout.tv_sec = 0;      // sec
        timeout.tv_usec = 20000; // usec
        switch (select(p->canSocketFd_+1, &fds, NULL, NULL, &timeout))
        {
        case -1:
            break;
        case 0:
            continue;
        default:
            // no data arrive
            if (!FD_ISSET(p->canSocketFd_, &fds))
                continue;
            // have new data arrive
            nBytes = read(p->canSocketFd_, &frame, CAN_MTU);
            switch (nBytes)
            {

            case CAN_MTU:
                p->DecodeCanFrameData(&frame);
                break;

            case CANFD_MTU:
                // TODO: Should make an example for CAN FD
                break;

            case -1:
                // Check the signal value on interrupt
                if (EINTR == errno)
                    continue;
                RCLCPP_ERROR(p->get_logger(), "CAN read error");

            default:
                continue;
            }
            break;
        }
    }
}

void MandaCanControl::DecodeCanFrameData(const struct can_frame* frame)
{
    switch(frame->can_id)
    {
        case 0x100:
        {
            manda_can_control::msg::SystemstateFb msg;
            manda_can_control::msg::BatteryFb msg_battery;
            msg.system_mode = frame->data[0];
            msg.control_mode = frame->data[1];
            int bits[4];
            for(int i = 0; i < 4; i++)
            {
                bits[i] = (frame->data[2] >> i) & 1; 
            }
            msg.emergency_mode = bits[0];
            msg.obstacle_mode = bits[1];
            msg.proximity_switch_mode = bits[2];
            msg.secure_edge_mode = bits[3];

            msg_battery.battery_soc = frame->data[3];
            msg_battery.battery_voltage = (frame->data[4] << 8 | frame->data[5]);
			uint8_t buf[2] = {frame->data[7], frame->data[6]};
			int16_t battery_current;
			memcpy(&battery_current, buf, 2);
			
            msg_battery.battery_current = battery_current;

            if(system_state_fb_pub_->get_subscription_count() > 0)
            {
                system_state_fb_pub_->publish(msg);
            }
            if(battery_fb_pub_->get_subscription_count() > 0)
            {
                battery_fb_pub_->publish(msg_battery);
            }
            
            break;
        }
        case 0x101:
        {
            manda_can_control::msg::SpeedFb msg;
			uint8_t buf[8] = {frame->data[1], frame->data[0], frame->data[3],frame->data[2],frame->data[5],frame->data[4],frame->data[7],frame->data[6]};
			int16_t lf_speed, rf_speed, lr_speed, rr_speed;
			memcpy(&lf_speed, buf, 2);
			msg.lf_speed = (float)(lf_speed / 1000.0);
			memcpy(&lr_speed, buf + 2, 2);
			msg.lr_speed = (float)(lr_speed / 1000.0);
			memcpy(&rf_speed, buf + 4, 2);
			msg.rf_speed = (float)(rf_speed / 1000.0);
			memcpy(&rr_speed, buf + 6, 2);
			msg.rr_speed = (float)(rr_speed / 1000.0);
            if(wheel_fb_pub_->get_subscription_count() > 0)
            {
                wheel_fb_pub_->publish(msg);
            }
            break;
        }
        case 0x102:
        {
            manda_can_control::msg::SteerFb msg;
			uint8_t buf[8] = {frame->data[1], frame->data[0], frame->data[3],frame->data[2],frame->data[5],frame->data[4],frame->data[7],frame->data[6]};
			int16_t lf_steer_angle, lr_steer_angle, rf_steer_angle, rr_steer_angle;
			memcpy(&lf_steer_angle, buf, 2);
            msg.lf_steer_angle = float(lf_steer_angle) / 100.0;
			memcpy(&lr_steer_angle, buf + 2, 2);
            msg.lr_steer_angle = float(lr_steer_angle) / 100.0;
			memcpy(&rf_steer_angle, buf + 4, 2);
            msg.rf_steer_angle = float(rf_steer_angle) / 100.0;
			memcpy(&rr_steer_angle, buf + 6, 2);
            msg.rr_steer_angle = float(rr_steer_angle) / 100.0;
            if(steer_fb_pub_->get_subscription_count() > 0)
            {
               steer_fb_pub_->publish(msg); 
            }
            break;
        }
        case 0x103:
        {
            manda_can_control::msg::FaultFb msg;
            int bits_motor_alarm[4];
            int bits_steer_alarm[4];
            int bits_motor_connect[4];
            int bits_steer_connect[4];
            msg.fault_code = frame->data[0];
            
            for(int i = 0; i < 4; i++)
            {
                bits_motor_alarm[i] = (frame->data[1] >> i) & 1; 
            }
            for(int i = 0; i < 4; i++)
            {
                bits_steer_alarm[i] = (frame->data[2] >> i) & 1; 
            }
            for(int i = 0; i < 4; i++)
            {
                bits_motor_connect[i] = (frame->data[3] >> i) & 1; 
            }
            for(int i = 0; i < 4; i++)
            {
                bits_steer_connect[i] = (frame->data[4] >> i) & 1; 
            }
            msg.lf_motor_fault = bits_motor_alarm[0];
            msg.lr_motor_fault = bits_motor_alarm[1];
            msg.rf_motor_fault = bits_motor_alarm[2];
            msg.rr_motor_fault = bits_motor_alarm[3];
            msg.lf_steer_fault = bits_steer_alarm[0];
            msg.lr_steer_fault = bits_steer_alarm[1];
            msg.rf_steer_fault = bits_steer_alarm[2];
            msg.rr_steer_fault = bits_steer_alarm[3];
            msg.lf_motor_disconnect = bits_motor_connect[0];
            msg.lr_motor_disconnect = bits_motor_connect[1];
            msg.rf_motor_disconnect = bits_motor_connect[2];
            msg.rr_motor_disconnect = bits_motor_connect[3];
            msg.lf_steer_disconnect = bits_steer_connect[0];
            msg.lr_steer_disconnect = bits_steer_connect[1];
            msg.rf_steer_disconnect = bits_steer_connect[2];
            msg.rr_steer_disconnect = bits_steer_connect[3];
            if(fault_fb_pb_->get_subscription_count() > 0)
            {
                fault_fb_pb_->publish(msg);
            }
            break;
        }
        case 0x104:
        {
            manda_can_control::msg::MotionFb msg;
			int16_t linear_x, linear_y, angular_z;
			uint8_t buf[6] = {frame->data[1], frame->data[0], frame->data[3],frame->data[2],frame->data[5],frame->data[4]};
			memcpy(&linear_x, buf, 2);
            msg.linear_x = float(linear_x) / 1000.0;
			memcpy(&linear_y, buf + 2, 2);
            msg.linear_y = float(linear_y) / 1000.0;
			memcpy(&angular_z, buf + 4, 2);
            msg.angular_z = float(angular_z) / 1000.0;
			msg.mode_type = frame->data[6];
            msg.mode_switch = frame->data[7];
            if(motion_fb_pb_->get_subscription_count() > 0)
            {
                motion_fb_pb_->publish(msg);
            }
            break;
        }
        default:
            break;
    }
}

int MandaCanControl::SendCanFrame(canid_t id, uint8_t *data, uint32_t frame_num)
{
    struct can_frame frame;
    memset(&frame, 0, sizeof(frame));
    frame.can_id = id;
    frame.can_dlc = 8;
    for (uint32_t i = 0; i < frame_num; i++) {
        for (uint32_t j = 0; j < frame.can_dlc; j++)
            frame.data[j] = data[(i * frame.can_dlc) + j];
        if (write(canSocketFd_, &frame, sizeof(frame)) != sizeof(frame)) {
            RCLCPP_INFO(this->get_logger(), "Can socket write error");
            return -1;
        }
    }
    return 0;
}

void MandaCanControl::MotionModeCallback(
    const std::shared_ptr<manda_can_control::srv::MotionMode::Request> req,
    std::shared_ptr<manda_can_control::srv::MotionMode::Response> res)
{
	uint8_t data = (uint8_t)req->cmd_ctl;
    uint8_t nData[1] = {data};
    SendCanFrame(0x401, nData, 1);
    res->cmd_ack = CMD_ACK_FINISH;
}

void MandaCanControl::MotionControlCallback(const manda_can_control::msg::MotionCtrl::SharedPtr motion)
{
    uint8_t nData[8];
    int16_t linear_x = int16_t(motion->linear_x * 1000);
    int16_t linear_y = int16_t(motion->linear_y * 1000);
    int16_t angular_z = int16_t(motion->angular_z * 1000);

    nData[0] = (linear_x >> 8) & 0xff;
    nData[1] = linear_x & 0xff;
    nData[2] = (linear_y >> 8) & 0xff;
    nData[3] = linear_y & 0xff;
    nData[4] = (angular_z >> 8) & 0xff;
    nData[5] = angular_z & 0xff;
    nData[6] = 0;
    nData[7] = 0;
    SendCanFrame(0x402, nData, 1);
}

void MandaCanControl::SpeedControlCallback(const manda_can_control::msg::SpeedCtrl::SharedPtr speed)
{
    uint8_t nData[8];  
    int16_t lf_speed = int16_t(speed->lf_speed * 1000);
    int16_t lr_speed = int16_t(speed->lr_speed * 1000);
    int16_t rf_speed = int16_t(speed->rf_speed * 1000);
    int16_t rr_speed = int16_t(speed->rr_speed * 1000);

    nData[0] = (lf_speed >> 8) & 0xff;
    nData[1] = lf_speed & 0xff;
    nData[2] = (lr_speed >> 8) & 0xff;
    nData[3] = lr_speed & 0xff;
    nData[4] = (rf_speed >> 8) & 0xff;
    nData[5] = rf_speed & 0xff;
    nData[6] = (rr_speed >> 8) & 0xff;
    nData[7] = rr_speed & 0xff;
    SendCanFrame(0x403, nData, 1);
}

void MandaCanControl::SteerControlCallback(const manda_can_control::msg::SteerCtrl::SharedPtr angle)
{
    uint8_t nData[8];
    int16_t lf_steer_angle = int16_t(angle->lf_steer_angle * 100);
    int16_t lr_steer_angle = int16_t(angle->lr_steer_angle * 100);
    int16_t rf_steer_angle = int16_t(angle->rf_steer_angle * 100);
    int16_t rr_steer_angle = int16_t(angle->rr_steer_angle * 100);
    nData[0] = (lf_steer_angle >> 8) & 0xff;
    nData[1] = lf_steer_angle & 0xff;
    nData[2] = (lr_steer_angle >> 8) & 0xff;
    nData[3] = lr_steer_angle & 0xff;
    nData[4] = (rf_steer_angle >> 8) & 0xff;
    nData[5] = rf_steer_angle & 0xff;
    nData[6] = (rr_steer_angle >> 8) & 0xff;
    nData[7] = rr_steer_angle & 0xff;
    SendCanFrame(0x404, nData, 1);
}

void MandaCanControl::DoCallback(
    const std::shared_ptr<manda_can_control::srv::DoCtrl::Request> req,
    std::shared_ptr<manda_can_control::srv::DoCtrl::Response> res)
{
	uint8_t data = (uint8_t)req->cmd_ctl;
	uint8_t nData[8] = {data, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    SendCanFrame(0x405, nData, 1);
    res->cmd_ack = CMD_ACK_FINISH;
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto cancontrol = std::make_shared<MandaCanControl>();
  cancontrol->Run();
  rclcpp::spin(cancontrol);
  rclcpp::shutdown();

  return 0;
}
