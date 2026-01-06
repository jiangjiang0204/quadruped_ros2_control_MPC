//
// Created by tlab-uav on 24-9-19.
//

#include "leg_pd_controller/LegPdController.h"

namespace leg_pd_controller {
    using config_type = controller_interface::interface_configuration_type;

    controller_interface::CallbackReturn LegPdController::on_init() {
        try {
            joint_names_ = auto_declare<std::vector<std::string> >("joints", joint_names_);//从参数服务器（YAML文件）读取关节名称列表
            reference_interface_types_ =
                    auto_declare<std::vector<std::string> >("reference_interfaces", reference_interface_types_);//从参数服务器读取参考接口类型列表
            state_interface_types_ = auto_declare<std::vector<
                std::string> >("state_interfaces", state_interface_types_);//从参数服务器读取状态接口类型列表（通常是 position 和 velocity）
        } catch (const std::exception &e) {
            fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
            return controller_interface::CallbackReturn::ERROR;//如果读取参数失败，打印错误并返回 ERROR
        }

        //根据关节数量，初始化内部数据容器的大小
        const size_t joint_num = joint_names_.size();
        joint_effort_command_.assign(joint_num, 0);
        joint_position_command_.assign(joint_num, 0);
        joint_velocities_command_.assign(joint_num, 0);
        joint_kp_command_.assign(joint_num, 0);
        joint_kd_command_.assign(joint_num, 0);

        return CallbackReturn::SUCCESS;
    }

    //配置硬件接口：告诉ros2control我们需要哪些接口
    //命令接口（输出）
    controller_interface::InterfaceConfiguration LegPdController::command_interface_configuration() const {
        controller_interface::InterfaceConfiguration conf = {config_type::INDIVIDUAL, {}};

        conf.names.reserve(joint_names_.size());
        for (const auto &joint_name: joint_names_) {
            // 声明我要控制每个关节的 "effort"（力矩/力）
            conf.names.push_back(joint_name + "/effort");
        }

        return conf;
    }

    //状态接口（输入）
    controller_interface::InterfaceConfiguration LegPdController::state_interface_configuration() const {
        controller_interface::InterfaceConfiguration conf = {config_type::INDIVIDUAL, {}};
        conf.names.reserve(joint_names_.size() * state_interface_types_.size());
        for (const auto &joint_name: joint_names_) {
            for (const auto &interface_type: state_interface_types_) {
                // 声明我要读取每个关节的状态（例如 "position" 和 "velocity"）
                conf.names.push_back(joint_name + "/" += interface_type);
            }
        }
        return conf;
    }

    //生命周期管理
    controller_interface::CallbackReturn LegPdController::on_configure(
        const rclcpp_lifecycle::State & /*previous_state*/) {
        reference_interfaces_.resize(joint_names_.size() * 5, std::numeric_limits<double>::quiet_NaN());
        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn LegPdController::on_activate(
        const rclcpp_lifecycle::State & /*previous_state*/) {
        joint_effort_command_interface_.clear();
        joint_position_state_interface_.clear();
        joint_velocity_state_interface_.clear();

        // assign effort command interface
        for (auto &interface: command_interfaces_) {
            joint_effort_command_interface_.emplace_back(interface);
        }

        // assign state interfaces
        for (auto &interface: state_interfaces_) {
            state_interface_map_[interface.get_interface_name()]->push_back(interface);
        }

        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn LegPdController::on_deactivate(
        const rclcpp_lifecycle::State & /*previous_state*/) {
        release_interfaces();
        return CallbackReturn::SUCCESS;
    }

    bool LegPdController::on_set_chained_mode(bool /*chained_mode*/) {
        return true;
    }

    //控制循环的主要更新函数，以很高的频率运行
    controller_interface::return_type LegPdController::update_and_write_commands(
        const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
        // --- 安全检查 ---
        // 检查所有的数据容器大小是否一致。如果不一致，说明初始化出了大问题，直接报错。
        // 这是为了防止数组越界访问导致程序崩溃（段错误）。
        if (joint_names_.size() != joint_effort_command_.size() ||
            joint_names_.size() != joint_kp_command_.size() ||
            joint_names_.size() != joint_position_command_.size() ||
            joint_names_.size() != joint_position_state_interface_.size() ||
            joint_names_.size() != joint_velocity_state_interface_.size() ||
            joint_names_.size() != joint_effort_command_interface_.size()) {
            std::cout << "joint_names_.size() = " << joint_names_.size() << std::endl;
            std::cout << "joint_effort_command_.size() = " << joint_effort_command_.size() << std::endl;
            std::cout << "joint_kp_command_.size() = " << joint_kp_command_.size() << std::endl;
            std::cout << "joint_position_command_.size() = " << joint_position_command_.size() << std::endl;
            std::cout << "joint_position_state_interface_.size() = " << joint_position_state_interface_.size() <<
                    std::endl;
            std::cout << "joint_velocity_state_interface_.size() = " << joint_velocity_state_interface_.size() <<
                    std::endl;
            std::cout << "joint_effort_command_interface_.size() = " << joint_effort_command_interface_.size() <<
                    std::endl;

            throw std::runtime_error("Mismatch in vector sizes in update_and_write_commands");
        }

        // --- 控制律计算 ---
        for (size_t i = 0; i < joint_names_.size(); ++i) {
            // PD Controller
            // Torque = 前馈力矩 + Kp * (目标位置 - 当前位置) + Kd * (目标速度 - 当前速度)
            const double torque = joint_effort_command_[i] + joint_kp_command_[i] * (
                                      joint_position_command_[i] - joint_position_state_interface_[i].get().get_value())
                                  +
                                  joint_kd_command_[i] * (
                                      joint_velocities_command_[i] - joint_velocity_state_interface_[i].get().
                                      get_value());
            // 将计算出的力矩写入硬件接口
            joint_effort_command_interface_[i].get().set_value(torque);
        }

        return controller_interface::return_type::OK;
    }

        //暴露接口给上层
    std::vector<hardware_interface::CommandInterface> LegPdController::on_export_reference_interfaces() {
        std::vector<hardware_interface::CommandInterface> reference_interfaces;

        int ind = 0;
        std::string controller_name = get_node()->get_name();
        for (const auto &joint_name: joint_names_) {
            std::cout << joint_name << std::endl;
            reference_interfaces.emplace_back(controller_name, joint_name + "/position", &joint_position_command_[ind]);
            reference_interfaces.emplace_back(controller_name, joint_name + "/velocity",
                                              &joint_velocities_command_[ind]);
            reference_interfaces.emplace_back(controller_name, joint_name + "/effort", &joint_effort_command_[ind]);
            reference_interfaces.emplace_back(controller_name, joint_name + "/kp", &joint_kp_command_[ind]);
            reference_interfaces.emplace_back(controller_name, joint_name + "/kd", &joint_kd_command_[ind]);
            ind++;
        }

        return reference_interfaces;
    }

#ifdef ROS2_CONTROL_VERSION_LT_3
    controller_interface::return_type LegPdController::update_reference_from_subscribers() {
        return controller_interface::return_type::OK;
    }
#else
    controller_interface::return_type LegPdController::update_reference_from_subscribers(
        const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
        return controller_interface::return_type::OK;
    }
#endif
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(leg_pd_controller::LegPdController, controller_interface::ChainableControllerInterface);
