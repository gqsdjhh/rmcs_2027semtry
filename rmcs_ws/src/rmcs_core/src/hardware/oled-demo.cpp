#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>

#include <librmcs/agent/c_board.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rmcs_executor/component.hpp>

#include "hardware/device/oled_runtime.hpp"

namespace rmcs_core::hardware {

class OledDemo
    : public rmcs_executor::Component
    , public rclcpp::Node
    , private librmcs::agent::CBoard {
public:
    OledDemo()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , librmcs::agent::CBoard{
              get_parameter("board_serial").as_string(),
              librmcs::agent::AdvancedOptions{
                  .dangerously_skip_version_checks =
                      get_parameter("skip_version_checks").as_bool()}}
        , command_component_(
              create_partner_component<CommandComponent>(get_component_name() + "_command", *this))
        , oled_runtime_(
              *this,
              [this](std::uint8_t slave_address, std::span<const std::byte> payload) {
                  write_i2c0(slave_address, payload);
              },
              static_cast<std::uint8_t>(get_parameter("oled_i2c_address").as_int()),
              static_cast<unsigned int>(get_parameter("oled_refresh_period").as_int()),
              get_logger()) {
        if (get_parameter("skip_version_checks").as_bool()) {
            RCLCPP_WARN(
                get_logger(),
                "Skipping board firmware version checks for OledDemo. Use only for temporary "
                "debugging.");
        }
    }

    OledDemo(const OledDemo&) = delete;
    OledDemo& operator=(const OledDemo&) = delete;
    OledDemo(OledDemo&&) = delete;
    OledDemo& operator=(OledDemo&&) = delete;

    ~OledDemo() override = default;

    void update() override {
        oled_runtime_.update_status();
        oled_runtime_.set_printf(
            "OLED TEST\nupd=%04llu\ncmd=%04llu\ni2c=0x%02X\nerr=%s",
            static_cast<unsigned long long>(oled_runtime_.update_count()),
            static_cast<unsigned long long>(oled_runtime_.command_count()),
            static_cast<unsigned int>(oled_runtime_.i2c_address()),
            oled_runtime_.has_i2c_error() ? "yes" : "no");
    }

    void command_update() {
        auto packet_builder = start_transmit();
        active_packet_builder_ = &packet_builder;

        struct PacketBuilderGuard {
            OledDemo& demo;

            ~PacketBuilderGuard() { demo.active_packet_builder_ = nullptr; }
        } guard{*this};

        oled_runtime_.command_update();
    }

private:
    class CommandComponent : public rmcs_executor::Component {
    public:
        explicit CommandComponent(OledDemo& demo)
            : demo_(demo) {}

        void update() override { demo_.command_update(); }

    private:
        OledDemo& demo_;
    };

    void write_i2c0(std::uint8_t slave_address, std::span<const std::byte> payload) {
        if (payload.empty())
            return;

        const librmcs::data::I2cDataView data{
            .slave_address = slave_address,
            .payload = payload,
        };
        if (active_packet_builder_) {
            active_packet_builder_->i2c0_write(data);
            return;
        }

        start_transmit().i2c0_write(data);
    }

    void i2c0_error_callback(std::uint8_t slave_address) override {
        oled_runtime_.notify_i2c_error(slave_address);
    }

    std::shared_ptr<CommandComponent> command_component_;
    device::OledRuntime oled_runtime_;
    librmcs::agent::CBoard::PacketBuilder* active_packet_builder_ = nullptr;
};

} // namespace rmcs_core::hardware

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::OledDemo, rmcs_executor::Component)
