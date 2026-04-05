#include <cstdint>
#include <memory>
#include <span>

#include <librmcs/agent/c_board.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rmcs_executor/component.hpp>

#include "hardware/device/oled.hpp"

namespace rmcs_core::hardware {

class SingleBoardOledDemo
    : public rmcs_executor::Component
    , public rclcpp::Node
    , private librmcs::agent::CBoard {
public:
    SingleBoardOledDemo()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , librmcs::agent::CBoard{get_parameter("board_serial").as_string()}
        , command_component_(
              create_partner_component<CommandComponent>(get_component_name() + "_command", *this))
        , oled_(
              *command_component_, "/oled",
              {
                  .write_command = [this](std::uint8_t command) { write_oled_command(command); },
                  .write_data =
                      [this](std::span<const std::uint8_t> data) { write_oled_data(data); },
              }) {
        register_output("/demo/oled/update_count", update_count_, std::uint64_t{0});
        register_output("/demo/oled/command_count", command_count_, std::uint64_t{0});
        register_output("/demo/oled/initialized", initialized_output_, false);
    }

    SingleBoardOledDemo(const SingleBoardOledDemo&) = delete;
    SingleBoardOledDemo& operator=(const SingleBoardOledDemo&) = delete;
    SingleBoardOledDemo(SingleBoardOledDemo&&) = delete;
    SingleBoardOledDemo& operator=(SingleBoardOledDemo&&) = delete;

    ~SingleBoardOledDemo() override = default;

    void update() override { ++(*update_count_); }

    void command_update() {
        ++(*command_count_);

        // Parameter text is the default content, and `/oled/text` can override it dynamically.
        initialized_ = oled_.display_text(get_parameter("display_message").as_string(), Font::k6x8);
        *initialized_output_ = initialized_;
        if (!initialized_)
            return;

        oled_.update_command(Font::k6x8);
    }

private:
    using Font = device::Oled::FontSize;

    class CommandComponent : public rmcs_executor::Component {
    public:
        explicit CommandComponent(SingleBoardOledDemo& demo)
            : demo_(demo) {}

        void update() override { demo_.command_update(); }

    private:
        SingleBoardOledDemo& demo_;
    };

    void write_oled_command(std::uint8_t command) {
        //等待i2c适配即可
        (void)command;
    }

    void write_oled_data(std::span<const std::uint8_t> data) {
        //等待i2c适配即可
        (void)data;
    }

    std::shared_ptr<CommandComponent> command_component_;
    device::Oled oled_;

    bool initialized_ = false;

    OutputInterface<std::uint64_t> update_count_;
    OutputInterface<std::uint64_t> command_count_;
    OutputInterface<bool> initialized_output_;
};

} // namespace rmcs_core::hardware

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::SingleBoardOledDemo, rmcs_executor::Component)
