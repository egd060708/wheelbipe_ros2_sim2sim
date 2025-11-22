
/* 头文件 */
#include "webots_bridge/webots_bridge_node.hpp"

/* 标准库文件 */
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

/* ros2库文件 */
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

/* webots文件 */
#include "webots/device.h"
#include "webots/robot.h"


namespace template_webots_ros2_ctrl
{
    WebotsBridge::WebotsBridge()
    {
        mNode = NULL;
    }
    void WebotsBridge::init(webots_ros2_driver::WebotsNode *node, const hardware_interface::HardwareInfo &info)
    {
        // 保存webots节点
        mNode = node;
        // 关节设备初始化
        for (hardware_interface::ComponentInfo component : info.joints) {
            Joint joint;
            joint.name = component.name;
            
            // 获取设备节点
            WbDeviceTag device = wb_robot_get_device(joint.name.c_str());
            // 获取设备类型
            WbNodeType type = wb_device_get_node_type(device);
            // 获取关节电机
            joint.motor = (type == WB_NODE_LINEAR_MOTOR || type == WB_NODE_ROTATIONAL_MOTOR)
                            ? device
                            : wb_position_sensor_get_motor(device);

            // 如果配置了传感器则需要获取传感器
            device = (component.parameters.count("sensor") == 0)
                    ? wb_robot_get_device(joint.name.c_str())
                    : wb_robot_get_device(component.parameters.at("sensor").c_str());
            type = wb_device_get_node_type(device);
            joint.sensor =
            (type == WB_NODE_POSITION_SENSOR) ? device : wb_motor_get_position_sensor(device);

            // 启动传感器及其反馈
            if (joint.sensor) {
                wb_position_sensor_enable(joint.sensor, wb_robot_get_basic_time_step());
                wb_motor_enable_torque_feedback(joint.motor, wb_robot_get_basic_time_step());
            }
            if (!joint.sensor && !joint.motor) {
                throw std::runtime_error("Cannot find a Motor or PositionSensor with name " + joint.name);
            }

            mJoints.push_back(joint);
        }
        // 传感器设备初始化
        for (hardware_interface::ComponentInfo component : info.sensors) {
            std::string sensor_name = component.name;
            WbDeviceTag device = wb_robot_get_device(sensor_name.c_str());
            WbNodeType type = wb_device_get_node_type(device);
            if (type == WB_NODE_INERTIAL_UNIT) {
              mImu.name = sensor_name;
              mImu.inertialUnit = device;
              mImu.gyro = wb_robot_get_device("gyro");
              mImu.accelerometer = wb_robot_get_device("accelerometer");  // defalt name
              wb_inertial_unit_enable(mImu.inertialUnit, wb_robot_get_basic_time_step());
              if (mImu.gyro) wb_gyro_enable(mImu.gyro, wb_robot_get_basic_time_step());
              if (mImu.accelerometer)
                wb_accelerometer_enable(mImu.accelerometer, wb_robot_get_basic_time_step());
            }
            // std::cout << "\033[35m" << sensor_name << type << "\033[0m" << std::endl;
        }
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn WebotsBridge::on_init(
        const hardware_interface::HardwareInfo & info
    )
    {
        if (
            hardware_interface::SystemInterface::on_init(info) !=
            rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS) {
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
        }
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    /* 状态接口发布 */
    std::vector<hardware_interface::StateInterface> WebotsBridge::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> interfaces;
        for (Joint & joint : mJoints) {
            if (joint.sensor) {
            interfaces.emplace_back(hardware_interface::StateInterface(
                joint.name, hardware_interface::HW_IF_POSITION, &(joint.position)));
            interfaces.emplace_back(hardware_interface::StateInterface(
                joint.name, hardware_interface::HW_IF_VELOCITY, &(joint.velocity)));
            interfaces.emplace_back(hardware_interface::StateInterface(
                joint.name, hardware_interface::HW_IF_EFFORT, &(joint.effort)));
            }
        }

        for (hardware_interface::ComponentInfo component : info_.sensors) {
            if (component.name == mImu.name) {
                for (uint i = 0; i < 4; i++) {
                    interfaces.emplace_back(hardware_interface::StateInterface(
                    component.name, component.state_interfaces[i].name, &mImu.orientation[i]));
                }
                for (uint i = 0; i < 3; i++) {
                    interfaces.emplace_back(hardware_interface::StateInterface(
                    component.name, component.state_interfaces[i + 4].name, &mImu.angular_velocity[i]));
                }
                for (uint i = 0; i < 3; i++) {
                    interfaces.emplace_back(hardware_interface::StateInterface(
                    component.name, component.state_interfaces[i + 7].name, &mImu.linear_acceleration[i]));
                }
            }
        }
        return interfaces;
    }

    /* 命令接口发布 */
    std::vector<hardware_interface::CommandInterface> WebotsBridge::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> interfaces;
        for (Joint & joint : mJoints) {
            if (joint.motor) {
            interfaces.emplace_back(hardware_interface::CommandInterface(
                joint.name, hardware_interface::HW_IF_POSITION, &(joint.positionCommand)));

            interfaces.emplace_back(hardware_interface::CommandInterface(
                joint.name, hardware_interface::HW_IF_EFFORT, &(joint.effortCommand)));

            interfaces.emplace_back(hardware_interface::CommandInterface(
                joint.name, hardware_interface::HW_IF_VELOCITY, &(joint.velocityCommand)));

            interfaces.emplace_back(hardware_interface::CommandInterface(joint.name, "kp", &(joint.kp)));

            interfaces.emplace_back(hardware_interface::CommandInterface(joint.name, "kd", &(joint.kd)));
            }
        }
        return interfaces;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn WebotsBridge::on_activate(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }
    
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    WebotsBridge::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
    {
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type WebotsBridge::read(
        const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    {
        // 计算时间数据
        static double lastReadTime = 0;
        const double deltaTime = wb_robot_get_time() - lastReadTime;
        lastReadTime = wb_robot_get_time();
        // 读取并初步处理关节数据
        size_t index = 0;
        for (Joint & joint : mJoints) {
            if (joint.sensor) {
            const double position = wb_position_sensor_get_value(joint.sensor);
            auto sin = std::sin(position);
            auto cos = std::cos(position);
            double position_modify = std::atan2(sin, cos);
            double velocity;
            if (std::abs(position_modify - joint.position) > M_PI) {
                auto sign = position_modify > joint.position ? 1 : -1;
                velocity = (position_modify - 2 * M_PI * sign - joint.position) / deltaTime;
            } else {
                velocity = (position_modify - joint.position) / deltaTime;
            }
        
            if (!std::isnan(joint.velocity)) {
                joint.acceleration = static_cast<tFloat>((joint.velocity - velocity) / deltaTime);
            }
            joint.velocity = static_cast<tFloat>(velocity);
            joint.position = static_cast<tFloat>(position_modify);
            joint.effort = static_cast<tFloat>(wb_motor_get_torque_feedback(joint.motor) + joint.effortCommand);
            }
            // if (index == 1 || index == 5) {
            //   if (joint.position < -2.5) {
            //     joint.position += 2 * M_PI;
            //   }
            // }
            index++;
        }
        
        if (mImu.linear_acceleration) {
            const double * values = wb_accelerometer_get_values(mImu.accelerometer);
            mImu.linear_acceleration[0] = static_cast<tFloat>(values[0]);
            mImu.linear_acceleration[1] = static_cast<tFloat>(values[1]);
            mImu.linear_acceleration[2] = static_cast<tFloat>(values[2]);
        }
        
        if (mImu.angular_velocity) {
            const double * values = wb_gyro_get_values(mImu.gyro);
            mImu.angular_velocity[0] = static_cast<tFloat>(values[0]);
            mImu.angular_velocity[1] = static_cast<tFloat>(values[1]);
            mImu.angular_velocity[2] = static_cast<tFloat>(values[2]);
        }
        
        if (mImu.orientation) {
            const double * values = wb_inertial_unit_get_quaternion(mImu.inertialUnit);
            mImu.orientation[0] = static_cast<tFloat>(values[0]);
            mImu.orientation[1] = static_cast<tFloat>(values[1]);
            mImu.orientation[2] = static_cast<tFloat>(values[2]);
            mImu.orientation[3] = static_cast<tFloat>(values[3]);
        }
        
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type WebotsBridge::write(
        const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    {
        // 直接调用力矩接口，按照mit模式实现控制量
        for (Joint & joint : mJoints) {
            if (joint.motor) {
            auto effort = joint.kp * (joint.positionCommand - joint.position) +
                            joint.kd * (joint.velocityCommand - joint.velocity) + joint.effortCommand;
            // joint.effortCommand = effort;
            wb_motor_set_torque(joint.motor, effort);
            }
        }
        return hardware_interface::return_type::OK;
    }
} // namespace template_webots_ros2_ctrl

PLUGINLIB_EXPORT_CLASS(
    template_webots_ros2_ctrl::WebotsBridge, webots_ros2_control::Ros2ControlSystemInterface)