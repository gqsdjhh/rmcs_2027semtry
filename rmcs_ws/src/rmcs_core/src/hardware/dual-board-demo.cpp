#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <string_view>

#include <librmcs/agent/c_board.hpp>
#include <librmcs/data/datas.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rmcs_executor/component.hpp>

namespace rmcs_core::hardware {

class DualBoardDemo
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    DualBoardDemo()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , command_component_(
              create_partner_component<DualBoardDemoCommand>(
                  get_component_name() + "_command", *this)) {
        register_output("/demo/top/update_count", top_update_count_, std::uint64_t{0});
        register_output("/demo/top/command_count", top_command_count_, std::uint64_t{0});
        register_output("/demo/top/can1_rx_count", top_can1_rx_count_, std::uint64_t{0});
        register_output("/demo/top/uart1_rx_bytes", top_uart1_rx_bytes_, std::uint64_t{0});

        register_output("/demo/bottom/update_count", bottom_update_count_, std::uint64_t{0});
        register_output("/demo/bottom/command_count", bottom_command_count_, std::uint64_t{0});
        register_output("/demo/bottom/can1_rx_count", bottom_can1_rx_count_, std::uint64_t{0});
        register_output("/demo/bottom/uart1_rx_bytes", bottom_uart1_rx_bytes_, std::uint64_t{0});

        top_board_ = std::make_unique<TopBoard>(
            *this, *command_component_, get_parameter("board_serial_top_board").as_string(),
            top_update_count_, top_command_count_, top_can1_rx_count_, top_uart1_rx_bytes_);
        bottom_board_ = std::make_unique<BottomBoard>(
            *this, *command_component_, get_parameter("board_serial_bottom_board").as_string(),
            bottom_update_count_, bottom_command_count_, bottom_can1_rx_count_,
            bottom_uart1_rx_bytes_);
    }

    DualBoardDemo(const DualBoardDemo&) = delete;
    DualBoardDemo& operator=(const DualBoardDemo&) = delete;
    DualBoardDemo(DualBoardDemo&&) = delete;
    DualBoardDemo& operator=(DualBoardDemo&&) = delete;

    ~DualBoardDemo() override = default;

    void update() override {
        top_board_->update();
        bottom_board_->update();
    }

    void command_update() {
        top_board_->command_update();
        bottom_board_->command_update();
    }

private:
    class DualBoardDemoCommand : public rmcs_executor::Component {
    public:
        explicit DualBoardDemoCommand(DualBoardDemo& dual_board_demo)
            : dual_board_demo_(dual_board_demo) {}

        void update() override { dual_board_demo_.command_update(); }

    private:
        DualBoardDemo& dual_board_demo_;
    };

    class TopBoard final : private librmcs::agent::CBoard {
    public:
        explicit TopBoard(
            DualBoardDemo&, DualBoardDemoCommand&, std::string_view board_serial,
            OutputInterface<std::uint64_t>& update_count,
            OutputInterface<std::uint64_t>& command_count,
            OutputInterface<std::uint64_t>& can1_rx_count,
            OutputInterface<std::uint64_t>& uart1_rx_bytes)
            : librmcs::agent::CBoard(board_serial)
            , update_count_(update_count)
            , command_count_(command_count)
            , can1_rx_count_(can1_rx_count)
            , uart1_rx_bytes_(uart1_rx_bytes) {}

        TopBoard(const TopBoard&) = delete;
        TopBoard& operator=(const TopBoard&) = delete;
        TopBoard(TopBoard&&) = delete;
        TopBoard& operator=(TopBoard&&) = delete;

        ~TopBoard() override = default;

        void update() { ++(*update_count_); }

        void command_update() { ++(*command_count_); }

    private:
        void can1_receive_callback(const librmcs::data::CanDataView& data) override {
            if (data.is_extended_can_id || data.is_remote_transmission) [[unlikely]]
                return;
            ++(*can1_rx_count_);
        }

        void uart1_receive_callback(const librmcs::data::UartDataView& data) override {
            *uart1_rx_bytes_ += static_cast<std::uint64_t>(data.uart_data.size());
        }

        OutputInterface<std::uint64_t>& update_count_;
        OutputInterface<std::uint64_t>& command_count_;
        OutputInterface<std::uint64_t>& can1_rx_count_;
        OutputInterface<std::uint64_t>& uart1_rx_bytes_;
    };

    class BottomBoard final : private librmcs::agent::CBoard {
    public:
        explicit BottomBoard(
            DualBoardDemo&, DualBoardDemoCommand&, std::string_view board_serial,
            OutputInterface<std::uint64_t>& update_count,
            OutputInterface<std::uint64_t>& command_count,
            OutputInterface<std::uint64_t>& can1_rx_count,
            OutputInterface<std::uint64_t>& uart1_rx_bytes)
            : librmcs::agent::CBoard(board_serial)
            , update_count_(update_count)
            , command_count_(command_count)
            , can1_rx_count_(can1_rx_count)
            , uart1_rx_bytes_(uart1_rx_bytes) {}

        BottomBoard(const BottomBoard&) = delete;
        BottomBoard& operator=(const BottomBoard&) = delete;
        BottomBoard(BottomBoard&&) = delete;
        BottomBoard& operator=(BottomBoard&&) = delete;

        ~BottomBoard() override = default;

        void update() { ++(*update_count_); }

        void command_update() { ++(*command_count_); }

    private:
        void can1_receive_callback(const librmcs::data::CanDataView& data) override {
            if (data.is_extended_can_id || data.is_remote_transmission) [[unlikely]]
                return;
            ++(*can1_rx_count_);
        }

        void uart1_receive_callback(const librmcs::data::UartDataView& data) override {
            *uart1_rx_bytes_ += static_cast<std::uint64_t>(data.uart_data.size());
        }

        OutputInterface<std::uint64_t>& update_count_;
        OutputInterface<std::uint64_t>& command_count_;
        OutputInterface<std::uint64_t>& can1_rx_count_;
        OutputInterface<std::uint64_t>& uart1_rx_bytes_;
    };

    std::shared_ptr<DualBoardDemoCommand> command_component_;
    std::shared_ptr<TopBoard> top_board_;
    std::shared_ptr<BottomBoard> bottom_board_;

    OutputInterface<std::uint64_t> top_update_count_;
    OutputInterface<std::uint64_t> top_command_count_;
    OutputInterface<std::uint64_t> top_can1_rx_count_;
    OutputInterface<std::uint64_t> top_uart1_rx_bytes_;

    OutputInterface<std::uint64_t> bottom_update_count_;
    OutputInterface<std::uint64_t> bottom_command_count_;
    OutputInterface<std::uint64_t> bottom_can1_rx_count_;
    OutputInterface<std::uint64_t> bottom_uart1_rx_bytes_;
};

} // namespace rmcs_core::hardware

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::DualBoardDemo, rmcs_executor::Component)
