#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>
#include <string>
#include <string_view>
#include <utility>

#include <librmcs/agent/c_board.hpp>
#include <librmcs/data/datas.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rmcs_executor/component.hpp>

#include "hardware/device/oled_runtime.hpp"

namespace rmcs_core::hardware {

class OledMenuDemo
    : public rmcs_executor::Component
    , public rclcpp::Node
    , private librmcs::agent::CBoard {
public:
    OledMenuDemo()
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
              get_logger(),
              "/demo/oled")
        , button_gpio_channel_(static_cast<std::uint8_t>(get_parameter("button_gpio_channel").as_int()))
        , button_read_period_ms_(
              static_cast<std::uint16_t>(get_parameter("button_read_period_ms").as_int()))
        , button_output_prefix_(get_parameter("button_output_prefix").as_string())
        , test_value_(get_parameter("test_value_initial").as_int()) {
        register_output(button_output_prefix_ + "/raw_pressed", raw_button_pressed_output_, false);
        register_output("/oled_menu_demo/test_value", test_value_output_, test_value_);

        if (get_parameter("skip_version_checks").as_bool()) {
            RCLCPP_WARN(
                get_logger(),
                "Skipping board firmware version checks for OledMenuDemo. Use only for temporary "
                "debugging.");
        }
    }

    OledMenuDemo(const OledMenuDemo&) = delete;
    OledMenuDemo& operator=(const OledMenuDemo&) = delete;
    OledMenuDemo(OledMenuDemo&&) = delete;
    OledMenuDemo& operator=(OledMenuDemo&&) = delete;

    ~OledMenuDemo() override = default;

    void update() override {
        oled_runtime_.update_status();
        *raw_button_pressed_output_ = raw_button_pressed_.load(std::memory_order_acquire);
        *test_value_output_ = test_value_;
    }

    void apply_display(std::string_view text, std::uint8_t inverted_line) {
        oled_runtime_.set_text(text);
        oled_runtime_.set_inverted_line(0, false);

        for (std::uint8_t line = 1; line < device::OledRuntime::kVisibleTextLineCount; ++line)
            oled_runtime_.set_inverted_line(line, line == inverted_line);
    }

    void command_update() {
        auto packet_builder = start_transmit();
        active_packet_builder_ = &packet_builder;

        struct PacketBuilderGuard {
            OledMenuDemo& demo;

            ~PacketBuilderGuard() { demo.active_packet_builder_ = nullptr; }
        } guard{*this};

        if (!button_read_configured_) {
            packet_builder.gpio_digital_read({
                .channel = button_gpio_channel_,
                .period_ms = button_read_period_ms_,
                .asap = true,
                .rising_edge = false,
                .falling_edge = false,
                .pull = librmcs::data::GpioPull::kUp,
            });
            button_read_configured_ = true;
        }

        oled_runtime_.command_update();
    }

private:
    class CommandComponent : public rmcs_executor::Component {
    public:
        explicit CommandComponent(OledMenuDemo& demo)
            : demo_(demo) {
            register_input("/oled/menu/text", text_);
            register_input("/oled/menu/inverted_line", inverted_line_);
            register_input("/oled/menu/test_value", test_value_);
        }

        void update() override {
            demo_.test_value_ = *test_value_;
            demo_.apply_display(*text_, *inverted_line_);
            demo_.command_update();
        }

    private:
        OledMenuDemo& demo_;
        InputInterface<std::string> text_;
        InputInterface<std::uint8_t> inverted_line_;
        InputInterface<std::int64_t> test_value_;
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

    void gpio_digital_read_result_callback(const librmcs::data::GpioDigitalDataView& data) override {
        if (data.channel != button_gpio_channel_)
            return;

        raw_button_pressed_.store(!data.high, std::memory_order_release);
    }

    void i2c0_error_callback(std::uint8_t slave_address) override {
        oled_runtime_.notify_i2c_error(slave_address);
    }

    std::shared_ptr<CommandComponent> command_component_;
    device::OledRuntime oled_runtime_;
    librmcs::agent::CBoard::PacketBuilder* active_packet_builder_ = nullptr;

    std::atomic_bool raw_button_pressed_ = false;
    bool button_read_configured_ = false;
    const std::uint8_t button_gpio_channel_;
    const std::uint16_t button_read_period_ms_;
    const std::string button_output_prefix_;
    std::int64_t test_value_;

    OutputInterface<bool> raw_button_pressed_output_;
    OutputInterface<std::int64_t> test_value_output_;
};

} // namespace rmcs_core::hardware

PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::OledMenuDemo, rmcs_executor::Component)
