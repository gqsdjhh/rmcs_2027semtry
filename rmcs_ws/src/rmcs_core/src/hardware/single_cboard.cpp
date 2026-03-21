#include <memory>

#include <librmcs/agent/c_board.hpp>
// #include <librmcs/data/datas.hpp>
#include <rclcpp/node.hpp>
// #include <rclcpp/node_options.hpp>
#include <rmcs_executor/component.hpp>

#include "hardware/device/can_packet.hpp"
#include "hardware/device/dji_motor.hpp"
#include "hardware/device/lk_motor.hpp"

namespace rmcs_core::hardware {

class Sentry
    : public rmcs_executor::Component
    , public rclcpp::Node
    , private librmcs::agent::CBoard {
public:
    Sentry()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , librmcs::agent::CBoard{get_parameter("board_serial").as_string()}
        , command_component_(
              create_partner_component<CommandComponent>(get_component_name() + "_command", *this))
        , chassis_wheel_motors_(
              {*this, *command_component_, "/chassis/left_front_wheel"},
              {*this, *command_component_, "/chassis/right_front_wheel"},
              {*this, *command_component_, "/chassis/right_back_wheel"},
              {*this, *command_component_, "/chassis/left_back_wheel"}) {
        for (auto& motor : chassis_wheel_motors_) {
            motor.configure(
                device::DjiMotor::Config{device::DjiMotor::Type::kM3508}
                    .set_reversed()
                    .set_reduction_ratio(13.0)
                    .enable_multi_turn_angle());
        }
    }

    Sentry(const Sentry&) = delete;
    Sentry& operator=(const Sentry&) = delete;
    Sentry(Sentry&&) = delete;
    Sentry& operator=(Sentry&&) = delete;

    ~Sentry() override = default;

    void update() override {
        for (auto& motor : chassis_wheel_motors_)
            motor.update_status();
    }

    void command_update() {
        auto builder = start_transmit();

        builder.can1_transmit({
            .can_id = 0x200,
            .can_data =
                device::CanPacket8{
                                   chassis_wheel_motors_[0].generate_command(),
                                   chassis_wheel_motors_[1].generate_command(),
                                   chassis_wheel_motors_[2].generate_command(),
                                   chassis_wheel_motors_[3].generate_command(),
                                   }
                    .as_bytes(),
        });

        // builder.can2_transmit({
        //     .can_id = 0x200,
        //     .can_data =
        //         device::CanPacket8{
        //                            device::CanPacket8::PaddingQuarter{},
        //                            gimbal_bullet_feeder_.generate_command(),
        //                            gimbal_left_friction_.generate_command(),
        //                            gimbal_right_friction_.generate_command(),
        //                            }
        //             .as_bytes(),
        // });
    }

private:
    void can1_receive_callback(const librmcs::data::CanDataView& data) override {
        if (data.is_extended_can_id || data.is_remote_transmission) [[unlikely]]
            return;

        switch (data.can_id) {
        case 0x201: chassis_wheel_motors_[0].store_status(data.can_data); break;
        case 0x202: chassis_wheel_motors_[1].store_status(data.can_data); break;
        case 0x203: chassis_wheel_motors_[2].store_status(data.can_data); break;
        case 0x204: chassis_wheel_motors_[3].store_status(data.can_data); break;
        default: break;
        }
    }

    // void can2_receive_callback(const librmcs::data::CanDataView& data) override {
    //     if (data.is_extended_can_id || data.is_remote_transmission) [[unlikely]]
    //         return;

    //     auto can_id = data.can_id;
    //     if (can_id == 0x142) {
    //         gimbal_pitch_motor_.store_status(data.can_data);
    //     } else if (can_id == 0x202) {
    //         gimbal_bullet_feeder_.store_status(data.can_data);
    //     } else if (can_id == 0x203) {
    //         gimbal_left_friction_.store_status(data.can_data);
    //     } else if (can_id == 0x204) {
    //         gimbal_right_friction_.store_status(data.can_data);
    //     }
    // }

private:
    class CommandComponent : public rmcs_executor::Component {
    public:
        explicit CommandComponent(Sentry& hardware)
            : hardware_(hardware) {}

        void update() override { hardware_.command_update(); }

    private:
        Sentry& hardware_;
    };

    std::shared_ptr<CommandComponent> command_component_;
    device::DjiMotor chassis_wheel_motors_[4];
};

} // namespace rmcs_core::hardware

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::hardware::Sentry, rmcs_executor::Component)
