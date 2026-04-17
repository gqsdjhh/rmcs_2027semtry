#include <chrono>
#include <cstdint>
#include <string>

#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rmcs_executor/component.hpp>

namespace rmcs_core::input {

class ButtonGesture
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    ButtonGesture()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , raw_input_name_(get_parameter("raw_pressed_input").as_string())
        , output_prefix_(get_parameter("output_prefix").as_string())
        , debounce_duration_(std::chrono::milliseconds{get_parameter("debounce_ms").as_int()})
        , double_click_gap_duration_(
              std::chrono::milliseconds{get_parameter("double_click_gap_ms").as_int()})
        , long_press_duration_(std::chrono::milliseconds{get_parameter("long_press_ms").as_int()}) {
        register_input(raw_input_name_, raw_pressed_input_);
        register_input("/predefined/timestamp", timestamp_);

        register_output(output_prefix_ + "/pressed", pressed_, false);
        register_output(output_prefix_ + "/down", down_, false);
        register_output(output_prefix_ + "/up", up_, false);
        register_output(output_prefix_ + "/click", click_, false);
        register_output(output_prefix_ + "/double_click", double_click_, false);
        register_output(output_prefix_ + "/long_press", long_press_, false);
        register_output(output_prefix_ + "/long_pressing", long_pressing_, false);
    }

    void update() override {
        const auto now = *timestamp_;
        const bool double_click_enabled =
            double_click_gap_duration_ > std::chrono::milliseconds::zero();
        reset_pulses();

        if (!initialized_) {
            initialized_ = true;
            last_raw_pressed_ = stable_pressed_ = *raw_pressed_input_;
            last_raw_change_time_ = now;
            press_start_time_ = now;
            last_release_time_ = now;
            *pressed_ = stable_pressed_;
            *long_pressing_ = false;
            return;
        }

        const bool raw_pressed = *raw_pressed_input_;
        if (raw_pressed != last_raw_pressed_) {
            last_raw_pressed_ = raw_pressed;
            last_raw_change_time_ = now;
        }

        if (double_click_enabled && waiting_single_click_ && !stable_pressed_
            && now - last_release_time_ >= double_click_gap_duration_) {
            *click_ = true;
            waiting_single_click_ = false;
        }

        if (raw_pressed != stable_pressed_ && now - last_raw_change_time_ >= debounce_duration_) {
            stable_pressed_ = raw_pressed;
            *pressed_ = stable_pressed_;

            if (stable_pressed_) {
                *down_ = true;
                press_start_time_ = now;
                long_press_emitted_ = false;
            } else {
                *up_ = true;
                *long_pressing_ = false;

                if (long_press_emitted_) {
                    waiting_single_click_ = false;
                } else if (!double_click_enabled) {
                    *click_ = true;
                    waiting_single_click_ = false;
                } else if (waiting_single_click_
                           && now - last_release_time_ < double_click_gap_duration_) {
                    *double_click_ = true;
                    waiting_single_click_ = false;
                } else {
                    waiting_single_click_ = true;
                    last_release_time_ = now;
                }
            }
        }

        if (stable_pressed_ && !long_press_emitted_
            && now - press_start_time_ >= long_press_duration_) {
            *long_press_ = true;
            *long_pressing_ = true;
            long_press_emitted_ = true;
            waiting_single_click_ = false;
        } else if (stable_pressed_ && long_press_emitted_) {
            *long_pressing_ = true;
        } else {
            *long_pressing_ = false;
        }

        *pressed_ = stable_pressed_;
    }

private:
    using TimePoint = std::chrono::steady_clock::time_point;

    void reset_pulses() {
        *down_ = false;
        *up_ = false;
        *click_ = false;
        *double_click_ = false;
        *long_press_ = false;
    }

    const std::string raw_input_name_;
    const std::string output_prefix_;
    const std::chrono::milliseconds debounce_duration_;
    const std::chrono::milliseconds double_click_gap_duration_;
    const std::chrono::milliseconds long_press_duration_;

    bool initialized_ = false;
    bool last_raw_pressed_ = false;
    bool stable_pressed_ = false;
    bool long_press_emitted_ = false;
    bool waiting_single_click_ = false;

    TimePoint last_raw_change_time_{};
    TimePoint press_start_time_{};
    TimePoint last_release_time_{};

    InputInterface<bool> raw_pressed_input_;
    InputInterface<TimePoint> timestamp_;

    OutputInterface<bool> pressed_;
    OutputInterface<bool> down_;
    OutputInterface<bool> up_;
    OutputInterface<bool> click_;
    OutputInterface<bool> double_click_;
    OutputInterface<bool> long_press_;
    OutputInterface<bool> long_pressing_;
};

} // namespace rmcs_core::input

PLUGINLIB_EXPORT_CLASS(rmcs_core::input::ButtonGesture, rmcs_executor::Component)
