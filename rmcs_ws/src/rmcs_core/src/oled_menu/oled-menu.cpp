#include <array>
#include <cstdint>
#include <string>

#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/chassis_mode.hpp>
#include <rmcs_msgs/gimbal_mode.hpp>
#include <rmcs_msgs/shoot_mode.hpp>

namespace rmcs_core::oled_menu {

class OledMenu
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    OledMenu()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , button_prefix_(get_parameter("button_prefix").as_string())
        , test_value_(get_parameter("test_value_initial").as_int()) {
        register_input("/oled/menu/enabled", menu_enabled_, false);
        register_input("/oled/menu/chassis_mode", chassis_mode_, false);
        register_input("/oled/menu/gimbal_mode", gimbal_mode_, false);
        register_input("/oled/menu/shoot_mode", shoot_mode_, false);
        register_input("/oled/menu/friction_enabled", friction_enabled_, false);

        register_input(button_prefix_ + "/click", click_, false);
        register_input(button_prefix_ + "/long_press", long_press_, false);

        register_output("/oled/menu/text", text_output_, std::string{});
        register_output("/oled/menu/inverted_line", inverted_line_output_, std::uint8_t{0});
        register_output("/oled/menu/test_value", test_value_output_, test_value_);
    }

    void before_updating() override {
        if (!menu_enabled_.ready())
            menu_enabled_.make_and_bind_directly(true);
        if (!chassis_mode_.ready())
            chassis_mode_.make_and_bind_directly(rmcs_msgs::ChassisMode::AUTO);
        if (!gimbal_mode_.ready())
            gimbal_mode_.make_and_bind_directly(rmcs_msgs::GimbalMode::IMU);
        if (!shoot_mode_.ready())
            shoot_mode_.make_and_bind_directly(rmcs_msgs::ShootMode::AUTOMATIC);
        if (!friction_enabled_.ready())
            friction_enabled_.make_and_bind_directly(false);

        if (!click_.ready())
            click_.make_and_bind_directly(false);
        if (!long_press_.ready())
            long_press_.make_and_bind_directly(false);
    }

    void update() override {
        if (*menu_enabled_ && *click_) {
            cursor_index_ =
                cursor_index_ >= kMenuItemCount ? 0 : (cursor_index_ + 1) % kMenuItemCount;
        }

        if (*menu_enabled_ && *long_press_ && cursor_index_ == kTestItemIndex)
            ++test_value_;

        render_menu_page();
    }

private:
    static constexpr std::size_t kMenuItemCount = 5;
    static constexpr std::size_t kVisibleItemCount = 3;
    static constexpr std::size_t kTestItemIndex = 4;

    void render_menu_page() {
        if (!*menu_enabled_) {
            *text_output_ = "MODE MENU\nDISABLED";
            *inverted_line_output_ = 0;
            return;
        }

        const auto items = build_menu_items();
        update_window_start();

        std::string text = "MODE MENU";
        const auto visible_end = std::min(kMenuItemCount, window_start_ + kVisibleItemCount);
        for (std::size_t index = window_start_; index < visible_end; ++index) {
            text += '\n';
            text += items[index];
        }

        *text_output_ = std::move(text);
        *test_value_output_ = test_value_;
        *inverted_line_output_ =
            cursor_index_ < kMenuItemCount
                ? static_cast<std::uint8_t>(cursor_index_ - window_start_ + 1)
                : static_cast<std::uint8_t>(0);
    }

    [[nodiscard]] std::array<std::string, kMenuItemCount> build_menu_items() const {
        return {
            std::string{"CHAS:"} + display_text(*chassis_mode_),
            std::string{"GMBL:"} + display_text(*gimbal_mode_),
            std::string{"SHOT:"} + display_text(*shoot_mode_),
            std::string{"FRIC:"} + display_text(*friction_enabled_),
            std::string{"TEST:"} + std::to_string(test_value_),
        };
    }

    void update_window_start() {
        if (cursor_index_ >= kMenuItemCount) {
            window_start_ = 0;
            return;
        }

        if (cursor_index_ < window_start_) {
            window_start_ = cursor_index_;
            return;
        }

        const auto window_end = window_start_ + kVisibleItemCount;
        if (cursor_index_ >= window_end)
            window_start_ = cursor_index_ - kVisibleItemCount + 1;
    }

    static const char* display_text(rmcs_msgs::ChassisMode mode) {
        using rmcs_msgs::ChassisMode;
        switch (mode) {
        case ChassisMode::AUTO: return "AUTO";
        case ChassisMode::SPIN: return "SPIN";
        case ChassisMode::STEP_DOWN: return "STEP";
        case ChassisMode::LAUNCH_RAMP: return "RAMP";
        }
        return "UNK";
    }

    static const char* display_text(rmcs_msgs::GimbalMode mode) {
        using rmcs_msgs::GimbalMode;
        switch (mode) {
        case GimbalMode::IMU: return "IMU";
        case GimbalMode::ENCODER: return "ENC";
        }
        return "UNK";
    }

    static const char* display_text(rmcs_msgs::ShootMode mode) {
        using rmcs_msgs::ShootMode;
        switch (mode) {
        case ShootMode::SINGLE: return "SINGLE";
        case ShootMode::AUTOMATIC: return "AUTO";
        case ShootMode::PRECISE: return "PRECISE";
        case ShootMode::LOW_LATENCY: return "LOWLAT";
        case ShootMode::OVERDRIVE: return "OVRDRV";
        }
        return "UNK";
    }

    static const char* display_text(bool enabled) { return enabled ? "ON" : "OFF"; }

    const std::string button_prefix_;
    std::int64_t test_value_;
    std::size_t cursor_index_ = kMenuItemCount;
    std::size_t window_start_ = 0;

    InputInterface<bool> menu_enabled_;
    InputInterface<rmcs_msgs::ChassisMode> chassis_mode_;
    InputInterface<rmcs_msgs::GimbalMode> gimbal_mode_;
    InputInterface<rmcs_msgs::ShootMode> shoot_mode_;
    InputInterface<bool> friction_enabled_;

    InputInterface<bool> click_;
    InputInterface<bool> long_press_;

    OutputInterface<std::string> text_output_;
    OutputInterface<std::uint8_t> inverted_line_output_;
    OutputInterface<std::int64_t> test_value_output_;
};

} // namespace rmcs_core::oled_menu

PLUGINLIB_EXPORT_CLASS(rmcs_core::oled_menu::OledMenu, rmcs_executor::Component)
