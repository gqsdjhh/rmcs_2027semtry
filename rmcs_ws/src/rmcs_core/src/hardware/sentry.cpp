#include <memory>

#include <librmcs/agent/c_board.hpp>
// #include <librmcs/data/datas.hpp>
#include <rclcpp/node.hpp>
// #include <rclcpp/node_options.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/serial_interface.hpp>
#include <rmcs_utility/ring_buffer.hpp>
#include <rmcs_description/tf_description.hpp>
#include <std_msgs/msg/int32.hpp>

#include "hardware/device/bmi088.hpp"
#include "hardware/device/can_packet.hpp"
#include "hardware/device/dji_motor.hpp"
#include "hardware/device/dr16.hpp"
#include "hardware/device/supercap.hpp"

namespace rmcs_core::hardware {

class Sentry
    : public rmcs_executor::Component
    , public rclcpp::Node
    , private librmcs::agent::CBoard {
public:
    explicit Sentry()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , librmcs::agent::CBoard{get_parameter("board_serial").as_string()}
        , sentry_command(
              create_partner_component<CommandComponent>(get_component_name() + "_command", *this))
        , chassis_wheel_motors_(
              {*this, *sentry_command, "/chassis/left_front_wheel"},
              {*this, *sentry_command, "/chassis/right_front_wheel"},
              {*this, *sentry_command, "/chassis/right_back_wheel"},
              {*this, *sentry_command, "/chassis/left_back_wheel"})
        , chassis_steer_motors_(
              {*this, *sentry_command, "/chassis/left_front_steering"},
              {*this, *sentry_command, "/chassis/left_back_steering"},
              {*this, *sentry_command, "/chassis/right_back_steering"},
              {*this, *sentry_command, "/chassis/right_front_steering"})
        , supercap_(*this, *sentry_command)          
        , dr16_{*this}
        , imu_(1000, 0.2, 0.0){

        this->register_output("/referee/serial", referee_serial_);
        this->register_output("/tf", tf_);

        this->register_output("/chassis/yaw/velocity_imu", chassis_yaw_velocity_imu_, 0);

        steers_calibrate_subscription_ = create_subscription<std_msgs::msg::Int32>(
        "/steers/calibrate", rclcpp::QoS{0}, [this](std_msgs::msg::Int32::UniquePtr&& msg) {
            steers_calibrate_subscription_callback(std::move(msg));
        });

        for (auto& motor : chassis_wheel_motors_) {
            motor.configure(
                device::DjiMotor::Config{device::DjiMotor::Type::kM3508}
                    .set_reversed()
                    .set_reduction_ratio(13.0)
                    .enable_multi_turn_angle());
        }

        referee_serial_->read = [this](std::byte* buffer, size_t size) {
            return referee_ring_buffer_receive_.pop_front_n(
                [&buffer](std::byte byte) noexcept { *buffer++ = byte; }, size);
        };
        referee_serial_->write = [this](const std::byte* buffer, size_t size) {
            start_transmit().uart1_transmit(
                {.uart_data = std::span<const std::byte>{buffer, size}});
            return size;
        };

        chassis_steer_motors_[0].configure(
                device::DjiMotor::Config{device::DjiMotor::Type::kGM6020}
                    .set_reversed()
                    .set_encoder_zero_point(
                        static_cast<int>(
                            this->get_parameter("left_front_zero_point").as_int()))
                    .enable_multi_turn_angle());
        chassis_steer_motors_[1].configure(
            device::DjiMotor::Config{device::DjiMotor::Type::kGM6020}
                .set_reversed()
                    .set_encoder_zero_point(
                        static_cast<int>(
                            this->get_parameter("left_back_zero_point").as_int()))
                    .enable_multi_turn_angle());
        chassis_steer_motors_[2].configure(
            device::DjiMotor::Config{device::DjiMotor::Type::kGM6020}
                .set_reversed()
                    .set_encoder_zero_point(
                        static_cast<int>(
                            this->get_parameter("right_back_zero_point").as_int()))
                    .enable_multi_turn_angle());
        chassis_steer_motors_[3].configure(
            device::DjiMotor::Config{device::DjiMotor::Type::kGM6020}
                .set_reversed()
                    .set_encoder_zero_point(
                        static_cast<int>(
                            this->get_parameter("right_front_zero_point").as_int()))
                    .enable_multi_turn_angle());
    }

    Sentry(const Sentry&) = delete;
    Sentry& operator=(const Sentry&) = delete;
    Sentry(Sentry&&) = delete;
    Sentry& operator=(Sentry&&) = delete;

    ~Sentry() override = default;

    void update() override {
        imu_.update_status();
        *chassis_yaw_velocity_imu_ = imu_.gz();
        supercap_.update_status();

        for (auto& motor : chassis_wheel_motors_)
            motor.update_status();
        for (auto& motor : chassis_steer_motors_)
            motor.update_status();

        dr16_.update_status();
    }

    void command_update() {
            auto builder = start_transmit();
            // builder.can1_transmit({
            //     .can_id = 0x141,
            //     .can_data = gimbal_bottom_yaw_motor_.generate_command().as_bytes(),
            // });

            if (can_transmission_mode_) {
                builder
                    .can1_transmit({
                        .can_id = 0x200,
                        .can_data =
                            device::CanPacket8{
                                chassis_wheel_motors_[1].generate_command(),
                                chassis_wheel_motors_[0].generate_command(),
                                device::CanPacket8::PaddingQuarter{},
                                device::CanPacket8::PaddingQuarter{},
                            }
                                .as_bytes(),
                    })
                    .can2_transmit({
                        .can_id = 0x200,
                        .can_data =
                            device::CanPacket8{
                                device::CanPacket8::PaddingQuarter{},
                                chassis_wheel_motors_[2].generate_command(),
                                device::CanPacket8::PaddingQuarter{},
                                chassis_wheel_motors_[3].generate_command(),
                            }
                                .as_bytes(),
                    });
            } else {
                builder
                    .can1_transmit({
                        .can_id = 0x1FE,
                        .can_data =
                            device::CanPacket8{
                                chassis_steer_motors_[1].generate_command(),
                                chassis_steer_motors_[0].generate_command(),
                                device::CanPacket8::PaddingQuarter{},
                                device::CanPacket8::PaddingQuarter{},
                            }
                                .as_bytes(),
                    })
                    .can2_transmit({
                        .can_id = 0x1FE,
                        .can_data =
                            device::CanPacket8{
                                chassis_steer_motors_[2].generate_command(),
                                chassis_steer_motors_[3].generate_command(),
                                device::CanPacket8::PaddingQuarter{},
                                supercap_.generate_command(),
                            }
                                .as_bytes(),
                    });
            }
            can_transmission_mode_ = !can_transmission_mode_;
    }

private:
    void can1_receive_callback(const librmcs::data::CanDataView& data) override {
        if (data.is_extended_can_id || data.is_remote_transmission) [[unlikely]]
            return;
        auto can_id = data.can_id;
        if (can_id == 0x201)
            chassis_wheel_motors_[1].store_status(data.can_data);
        else if (can_id == 0x202)
            chassis_wheel_motors_[0].store_status(data.can_data);
        else if (can_id == 0x205)
            chassis_steer_motors_[1].store_status(data.can_data);
        else if (can_id == 0x206)
            chassis_steer_motors_[0].store_status(data.can_data);
        // else if (can_id == 0x141)
        //     gimbal_bottom_yaw_motor_.store_status(data.can_data);
    }

    void can2_receive_callback(const librmcs::data::CanDataView& data) override {
        if (data.is_extended_can_id || data.is_remote_transmission) [[unlikely]]
            return;
        auto can_id = data.can_id;
        if (can_id == 0x202)
            chassis_wheel_motors_[2].store_status(data.can_data);
        else if (can_id == 0x204)
            chassis_wheel_motors_[3].store_status(data.can_data);
        else if (can_id == 0x205)
            chassis_steer_motors_[2].store_status(data.can_data);
        else if (can_id == 0x206)
            chassis_steer_motors_[3].store_status(data.can_data);
        else if (can_id == 0x300)
            supercap_.store_status(data.can_data);
    }

    void uart1_receive_callback(const librmcs::data::UartDataView& data) override {
        const auto* uart_data = data.uart_data.data();
        referee_ring_buffer_receive_.emplace_back_n(
            [&uart_data](std::byte* storage) noexcept { *storage = *uart_data++; },
            data.uart_data.size());
    }

    void dbus_receive_callback(const librmcs::data::UartDataView& data) override {
        dr16_.store_status(data.uart_data.data(), data.uart_data.size());
    }

    void accelerometer_receive_callback(
        const librmcs::data::AccelerometerDataView& data) override {
        imu_.store_accelerometer_status(data.x, data.y, data.z);
    }

    void gyroscope_receive_callback(const librmcs::data::GyroscopeDataView& data) override {
        imu_.store_gyroscope_status(data.x, data.y, data.z);
    }

    void steers_calibrate_subscription_callback(std_msgs::msg::Int32::UniquePtr) {
        RCLCPP_INFO(
            get_logger(), "[steer calibration] New left front offset: %d",
            this->chassis_steer_motors_[0].calibrate_zero_point());
        RCLCPP_INFO(
            get_logger(), "[steer calibration] New left back offset: %d",
            this->chassis_steer_motors_[1].calibrate_zero_point());
        RCLCPP_INFO(
            get_logger(), "[steer calibration] New right back offset: %d",
            this->chassis_steer_motors_[2].calibrate_zero_point());
        RCLCPP_INFO(
            get_logger(), "[steer calibration] New right front offset: %d",
            this->chassis_steer_motors_[3].calibrate_zero_point());
    }

private:
    class CommandComponent : public rmcs_executor::Component {
    public:
        explicit CommandComponent(Sentry& hardware)
            : hardware_(hardware) {}

        void update() override { hardware_.command_update(); }

    private:
        Sentry& hardware_;
    };
    std::shared_ptr<CommandComponent> sentry_command;

    bool can_transmission_mode_ = true;

    device::DjiMotor chassis_wheel_motors_[4];
    device::DjiMotor chassis_steer_motors_[4];
    device::Supercap supercap_;

    device::Dr16 dr16_;

    rmcs_utility::RingBuffer<std::byte> referee_ring_buffer_receive_{256};
    OutputInterface<rmcs_msgs::SerialInterface> referee_serial_;


    device::Bmi088 imu_;
    OutputInterface<rmcs_description::Tf> tf_;
    OutputInterface<double> chassis_yaw_velocity_imu_;

    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr steers_calibrate_subscription_;
};

} // namespace rmcs_core::hardware

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::hardware::Sentry, rmcs_executor::Component)
