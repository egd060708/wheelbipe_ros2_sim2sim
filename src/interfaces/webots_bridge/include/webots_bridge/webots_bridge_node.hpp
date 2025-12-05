/* 重写了webots_ros2_control中的Ros2ControlSystem，提高实时性能和自定义传感器数据 */

#pragma once

// 标准库工具
#include <memory>
#include <string>
#include <vector>

// ros2ctrl硬件接口
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"

// ros2标准库
#include "rclcpp/macros.hpp"
#include "realtime_tools/realtime_publisher.hpp"

// webots接口
// #include "webots/Accelerometer.hpp"
// #include "webots/Gyro.hpp"
// #include "webots/InertialUnit.hpp"
// #include "webots/Motor.hpp"
// #include "webots/PositionSensor.hpp"
#include "webots/accelerometer.h"
#include "webots/gyro.h"
#include "webots/inertial_unit.h"
#include "webots/motor.h"
#include "webots/position_sensor.h"

// webots_ros2接口
#include "webots_ros2_control/Ros2ControlSystemInterface.hpp"
#include "webots_ros2_driver/PluginInterface.hpp"
#include "webots_ros2_driver/WebotsNode.hpp"

typedef double tFloat;

namespace template_webots_ros2_ctrl
{
    /* 关节控制数据结构 */
    typedef struct _Joint
    {
        tFloat position = 0.0;     // 关节位置
        tFloat velocity = 0.0;     // 关节速度
        tFloat effort = 0.0;       // 关节力矩
        tFloat acceleration = 0.0; // 关节加速度

        tFloat effortCommand = 0.0;   // 力矩命令
        tFloat positionCommand = 0.0; // 位置命令
        tFloat velocityCommand = 0.0; // 速度命令

        tFloat kp = 0.0; // 位置控制增益
        tFloat kd = 0.0; // 速度控制增益
        // tFloat ki = 0.0; // 位置积分增益

        std::string name; // 关节名称
        WbDeviceTag motor;
        WbDeviceTag sensor;
    } Joint;
    /* 惯导控制数据结构 */
    typedef struct _InertiaUnit
    {
        std::string name;
        WbDeviceTag inertialUnit;
        WbDeviceTag gyro;
        WbDeviceTag accelerometer;
        tFloat linear_acceleration[3];
        tFloat angular_velocity[3];
        tFloat orientation[4]; // x y z w
    } InertiaUnit;

    /* ros2ctrl接口单元 */
    class WebotsBridge : public webots_ros2_control::Ros2ControlSystemInterface
    {
    public:
        WebotsBridge();
        void init(
            webots_ros2_driver::WebotsNode *node, const hardware_interface::HardwareInfo &info) override;

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_init(
            const hardware_interface::HardwareInfo &info) override;
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(
            const rclcpp_lifecycle::State & /*previous_state*/) override;
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(
            const rclcpp_lifecycle::State & /*previous_state*/) override;

        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
        hardware_interface::return_type read(
            const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override;
        hardware_interface::return_type write(
            const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override;

    private:
        webots_ros2_driver::WebotsNode *mNode;
        std::vector<Joint> mJoints;
        InertiaUnit mImu;
    };

    // 通用角度归一化函数
    double normalize_angle(double angle, double mod_value) {
        // mod_value: 模数（2π, 4π等）
        double half_mod = mod_value / 2.0;
        
        // 取模
        angle = std::fmod(angle, mod_value);
        
        // 映射到[-half_mod, half_mod]
        if (angle > half_mod) {
            angle -= mod_value;
        } else if (angle < -half_mod) {
            angle += mod_value;
        }
        
        return angle;
    }

    // 通用角度差计算
    double angle_difference(double angle1, double angle2, double mod_value) {
        double diff = angle1 - angle2;
        double half_mod = mod_value / 2.0;
        
        if (diff > half_mod) {
            diff -= mod_value;
        } else if (diff < -half_mod) {
            diff += mod_value;
        }
        
        return diff;
    }
}