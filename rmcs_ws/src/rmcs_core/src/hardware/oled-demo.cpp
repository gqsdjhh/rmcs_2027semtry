#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>
#include <string>
#include <string_view>

#include <librmcs/agent/c_board.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_utility/tick_timer.hpp>

#include "hardware/device/oled.hpp"
#include "hardware/device/oled_i2c.hpp"

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
        , librmcs::agent::CBoard{get_parameter("board_serial").as_string()}
        , command_component_(
              create_partner_component<CommandComponent>(get_component_name() + "_command", *this))
        , oled_i2c_([this](std::uint8_t slave_address, std::span<const std::byte> payload) {
            write_i2c0(slave_address, payload);
        })
        , oled_(
              *command_component_, "/oled",
              oled_i2c_.backend()) {
        register_output("/demo/oled/update_count", update_count_, std::uint64_t{0});
        register_output("/demo/oled/command_count", command_count_, std::uint64_t{0});
        register_output("/demo/oled/initialized", initialized_output_, false);
        register_output("/demo/oled/scene_index", scene_index_output_, std::uint8_t{0});

        if (!has_parameter("display_message"))
            declare_parameter<std::string>("display_message", "RMCS OLED demo");
        if (!has_parameter("oled_i2c_address"))
            declare_parameter<int64_t>("oled_i2c_address", device::OledConfig{}.i2c_address);
        if (!has_parameter("scene_period"))
            declare_parameter<int64_t>("scene_period", 150);

        auto oled_config = oled_.config();
        const auto configured_i2c_address = get_parameter("oled_i2c_address").as_int();
        if (configured_i2c_address < 0 || configured_i2c_address > 0x7F) {
            RCLCPP_WARN(
                get_logger(),
                "Invalid OLED I2C address %ld. Falling back to 0x%02X.",
                configured_i2c_address, static_cast<unsigned int>(oled_config.i2c_address));
        } else {
            oled_config.i2c_address = static_cast<std::uint8_t>(configured_i2c_address);
        }
        oled_i2c_.set_i2c_address(oled_config.i2c_address);
        oled_.set_config(oled_config);

        scene_period_ =
            static_cast<unsigned int>(std::max<int64_t>(1, get_parameter("scene_period").as_int()));
        scene_timer_.reset(scene_period_);
    }

    OledDemo(const OledDemo&) = delete;
    OledDemo& operator=(const OledDemo&) = delete;
    OledDemo(OledDemo&&) = delete;
    OledDemo& operator=(OledDemo&&) = delete;

    ~OledDemo() override = default;

    void update() override { ++(*update_count_); }

    void command_update() {
        ++(*command_count_);

        if (scene_timer_.tick()) {
            current_scene_ =
                static_cast<DemoScene>((static_cast<std::uint8_t>(current_scene_) + 1)
                                       % static_cast<std::uint8_t>(DemoScene::kCount));
            scene_tick_ = 0;
            scene_entered_ = true;
            scene_timer_.reset(scene_period_);
        }

        ++scene_tick_;
        *scene_index_output_ = static_cast<std::uint8_t>(current_scene_);

        initialized_ =
            render_scene(current_scene_) && !i2c_error_seen_.load(std::memory_order_acquire);
        *initialized_output_ = initialized_;
        scene_entered_ = false;
    }

private:
    using Font = device::Oled::FontSize;

    enum class DemoScene : std::uint8_t {
        kDisplayText = 0,
        kDisplayLineText,
        kDisplayAtText,
        kDisplayPrintf,
        kDisplayLinePrintf,
        kDisplayAtPrintf,
        kCount,
    };

    class CommandComponent : public rmcs_executor::Component {
    public:
        explicit CommandComponent(OledDemo& demo)
            : demo_(demo) {}

        void update() override { demo_.command_update(); }

    private:
        OledDemo& demo_;
    };

    bool render_scene(DemoScene scene) {
        switch (scene) {
        case DemoScene::kDisplayText: return render_display_text_scene();
        case DemoScene::kDisplayLineText: return render_display_line_text_scene();
        case DemoScene::kDisplayAtText: return render_display_at_text_scene();
        case DemoScene::kDisplayPrintf: return render_display_printf_scene();
        case DemoScene::kDisplayLinePrintf: return render_display_line_printf_scene();
        case DemoScene::kDisplayAtPrintf: return render_display_at_printf_scene();
        case DemoScene::kCount: return false;
        }
        return false;
    }

    bool render_display_text_scene() {
        auto message = get_parameter("display_message").as_string();
        if (message.size() > 16)
            message.resize(16);

        std::string content = "display_text\n";
        content += message;
        content += "\nwhole-screen\nstring_view";
        return oled_.display_text(content, Font::k6x8);
    }

    bool render_display_line_text_scene() {
        auto message = get_parameter("display_message").as_string();
        if (message.size() > 12)
            message.resize(12);

        bool success = true;
        success &= oled_.display_line_text(0, "display_line_text", Font::k6x8);
        success &= oled_.display_line_text(2, "message:", Font::k6x8);
        success &= oled_.display_line_text(3, message, Font::k6x8);
        success &= oled_.display_line_text(
            5, scene_tick_ % 2 == 0 ? "line update A" : "line update B", Font::k6x8);
        success &= oled_.display_line_text(7, "per-line flush", Font::k6x8);
        return success;
    }

    bool render_display_at_text_scene() {
        bool success = true;
        if (scene_entered_)
            success &= oled_.display_text("", Font::k6x8);

        success &= oled_.display_at_text(0, 0, "display_at_text", Font::k6x8);
        success &= oled_.display_at_text(0, 16, "x=0,y=16", Font::k6x8);
        success &= oled_.display_at_text(42, 32, "center", Font::k6x8);
        success &= oled_.display_at_text(
            0, 48, scene_tick_ % 2 == 0 ? "partial A" : "partial B", Font::k6x8);
        return success;
    }

    bool render_display_printf_scene() {
        return oled_.display_printf(
            Font::k6x8, "display_printf\nupd=%04llu\ncmd=%04llu\nratio=%+.2f\nptr=%p",
            static_cast<unsigned long long>(*update_count_),
            static_cast<unsigned long long>(*command_count_),
            static_cast<double>((scene_tick_ % 40) - 20) / 3.0, static_cast<void*>(this));
    }

    bool render_display_line_printf_scene() {
        bool success = true;
        success &= oled_.display_line_printf(0, Font::k6x8, "display_line_printf");
        success &= oled_.display_line_printf(
            2, Font::k6x8, "upd=%04llu", static_cast<unsigned long long>(*update_count_));
        success &= oled_.display_line_printf(
            3, Font::k6x8, "cmd=%04llu", static_cast<unsigned long long>(*command_count_));
        success &= oled_.display_line_printf(
            5, Font::k6x8, "hex=0x%04X", static_cast<unsigned int>(*command_count_ & 0xFFFF));
        success &= oled_.display_line_printf(
            7, Font::k6x8, "tick=%03u", static_cast<unsigned int>(scene_tick_ % 1000));
        return success;
    }

    bool render_display_at_printf_scene() {
        bool success = true;
        if (scene_entered_)
            success &= oled_.display_text("", Font::k6x8);

        success &= oled_.display_at_printf(0, 0, Font::k6x8, "display_at_printf");
        success &= oled_.display_at_printf(0, 16, Font::k6x8, "x=%d", 0);
        success &= oled_.display_at_printf(48, 16, Font::k6x8, "y=%d", 16);
        success &= oled_.display_at_printf(
            0, 32, Font::k6x8, "scene=%u", static_cast<unsigned int>(current_scene_));
        success &= oled_.display_at_printf(
            0, 48, Font::k6x8, "tick=%03u", static_cast<unsigned int>(scene_tick_ % 1000));
        return success;
    }

    void write_i2c0(std::uint8_t slave_address, std::span<const std::byte> payload) {
        if (payload.empty())
            return;
        start_transmit().i2c0_write({
            .slave_address = slave_address,
            .payload = payload,
        });
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
    bool scene_entered_ = true;
    unsigned int scene_period_ = 150;
    std::uint64_t scene_tick_ = 0;
    DemoScene current_scene_ = DemoScene::kDisplayText;
    rmcs_utility::TickTimer scene_timer_;

    OutputInterface<std::uint64_t> update_count_;
    OutputInterface<std::uint64_t> command_count_;
    OutputInterface<bool> initialized_output_;
    OutputInterface<std::uint8_t> scene_index_output_;
};

} // namespace rmcs_core::hardware

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::OledDemo, rmcs_executor::Component)
