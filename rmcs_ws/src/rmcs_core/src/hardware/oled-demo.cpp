#include <algorithm>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>

#include <librmcs/agent/c_board.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rmcs_executor/component.hpp>

#include "hardware/device/oled.hpp"
#include "hardware/device/oled_i2c.hpp"

namespace rmcs_core::hardware {

namespace {

std::uint8_t make_oled_i2c_address(rclcpp::Node& node) {
    if (!node.has_parameter("oled_i2c_address")) {
        node.declare_parameter<int64_t>(
            "oled_i2c_address", device::OledI2c::kDefaultI2cAddress);
    }

    const auto configured_i2c_address = node.get_parameter("oled_i2c_address").as_int();
    if (configured_i2c_address < 0 || configured_i2c_address > 0x7F) {
        RCLCPP_WARN(
            node.get_logger(), "Invalid OLED I2C address %ld. Falling back to 0x%02X.",
            configured_i2c_address,
            static_cast<unsigned int>(device::OledI2c::kDefaultI2cAddress));
        return device::OledI2c::kDefaultI2cAddress;
    }

    return static_cast<std::uint8_t>(configured_i2c_address);
}

unsigned int make_oled_refresh_period(rclcpp::Node& node) {
    if (!node.has_parameter("oled_refresh_period"))
        node.declare_parameter<int64_t>("oled_refresh_period", 20);

    return static_cast<unsigned int>(
        std::max<int64_t>(1, node.get_parameter("oled_refresh_period").as_int()));
}

librmcs::agent::AdvancedOptions make_c_board_options(rclcpp::Node& node) {
    librmcs::agent::AdvancedOptions options{};
    if (node.has_parameter("skip_version_checks")) {
        options.dangerously_skip_version_checks =
            node.get_parameter("skip_version_checks").as_bool();
    }
    return options;
}

} // namespace

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
              get_parameter("board_serial").as_string(), make_c_board_options(*this)}
        , command_component_(
              create_partner_component<CommandComponent>(get_component_name() + "_command", *this))
        , oled_i2c_(
              [this](std::uint8_t slave_address, std::span<const std::byte> payload) {
                  write_i2c0(slave_address, payload);
              },
              make_oled_i2c_address(*this))
        , oled_(oled_i2c_.backend())
        , oled_refresh_period_(make_oled_refresh_period(*this)) {
        register_output("/demo/oled/update_count", update_count_, std::uint64_t{0});
        register_output("/demo/oled/command_count", command_count_, std::uint64_t{0});
        register_output("/demo/oled/initialized", initialized_output_, false);

        if (!has_parameter("skip_version_checks"))
            declare_parameter<bool>("skip_version_checks", false);

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

    void update() override { ++(*update_count_); }

    void command_update() {
        ++(*command_count_);

        if (oled_refresh_countdown_ > 0) {
            --oled_refresh_countdown_;
            *initialized_output_ = initialized_;
            return;
        }

        oled_refresh_countdown_ = oled_refresh_period_;
        initialized_ = render_test_screen_batched()
                    && !i2c_error_seen_.load(std::memory_order_acquire);
        *initialized_output_ = initialized_;
    }

private:
    using Font = device::Oled::FontSize;

    class CommandComponent : public rmcs_executor::Component {
    public:
        explicit CommandComponent(OledDemo& demo)
            : demo_(demo) {}

        void update() override { demo_.command_update(); }

    private:
        OledDemo& demo_;
    };

    bool render_test_screen_batched() {
        auto packet_builder = start_transmit();
        active_packet_builder_ = &packet_builder;

        struct PacketBuilderGuard {
            OledDemo& demo;

            ~PacketBuilderGuard() { demo.active_packet_builder_ = nullptr; }
        } guard{*this};

        return render_test_screen();
    }

    bool render_test_screen() {
        const auto i2c_error = i2c_error_seen_.load(std::memory_order_acquire);
        return oled_.display_printf(
            Font::k6x8, "OLED TEST\nupd=%04llu\ncmd=%04llu\ni2c=0x%02X\nerr=%s",
            static_cast<unsigned long long>(*update_count_),
            static_cast<unsigned long long>(*command_count_),
            static_cast<unsigned int>(oled_i2c_.i2c_address()), i2c_error ? "yes" : "no");
    }

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
        i2c_error_seen_.store(true, std::memory_order_release);
        RCLCPP_WARN(
            get_logger(), "OLED I2C0 write failed for slave address 0x%02X.",
            static_cast<unsigned int>(slave_address));
    }

    std::shared_ptr<CommandComponent> command_component_;
    device::OledI2c oled_i2c_;
    device::Oled oled_;

    std::atomic_bool i2c_error_seen_ = false;
    bool initialized_ = false;
    librmcs::agent::CBoard::PacketBuilder* active_packet_builder_ = nullptr;
    unsigned int oled_refresh_period_ = 20;
    unsigned int oled_refresh_countdown_ = 0;

    OutputInterface<std::uint64_t> update_count_;
    OutputInterface<std::uint64_t> command_count_;
    OutputInterface<bool> initialized_output_;
};

} // namespace rmcs_core::hardware

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::OledDemo, rmcs_executor::Component)
