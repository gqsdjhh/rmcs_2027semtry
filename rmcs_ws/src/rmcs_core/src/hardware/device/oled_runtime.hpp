#pragma once

#include <algorithm>
#include <array>
#include <atomic>
#include <cstdarg>
#include <cstdint>
#include <exception>
#include <cstdio>
#include <string>
#include <string_view>
#include <utility>

#include <librmcs/agent/c_board.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rmcs_executor/component.hpp>

#include "hardware/device/oled.hpp"
#include "hardware/device/oled_i2c.hpp"

namespace rmcs_core::hardware::device {
using rmcs_executor::Component;

class OledRuntime {
public:
    static constexpr std::uint8_t kDefaultI2cAddress = OledI2c::kDefaultI2cAddress;
    using TransferWriter = OledI2c::TransferWriter;
    static constexpr Oled::FontSize kFont = Oled::FontSize::k8x16;
    static constexpr std::uint8_t kVisibleTextLineCount =
        static_cast<std::uint8_t>(Oled::kHeight / Oled::line_height(kFont));

    OledRuntime(
        Component& status_component, TransferWriter writer, std::uint8_t i2c_address,
        unsigned int refresh_period, rclcpp::Logger logger, std::string status_prefix = "/oled")
        : status_prefix_(std::move(status_prefix))
        , logger_(std::move(logger))
        , oled_i2c_(std::move(writer), i2c_address)
        , oled_(oled_i2c_.backend())
        , oled_refresh_period_(refresh_period) {
        status_component.register_output(
            status_prefix_ + "/update_count", update_count_, std::uint64_t{0});
        status_component.register_output(
            status_prefix_ + "/command_count", command_count_, std::uint64_t{0});
        status_component.register_output(status_prefix_ + "/initialized", initialized_output_, false);
    }

    void update_status() {
        ++(*update_count_);
        *initialized_output_ = initialized_ && !i2c_error_seen_.load(std::memory_order_acquire);
    }

    void set_text(std::string_view text) {
        const auto text_length = std::min(text.size(), pending_text_buffer_.size() - 1);
        if (text_length == pending_text_length_
            && std::equal(text.begin(), text.begin() + static_cast<std::ptrdiff_t>(text_length),
                pending_text_buffer_.begin())) {
            return;
        }

        std::copy_n(text.data(), text_length, pending_text_buffer_.data());
        pending_text_buffer_[text_length] = '\0';
        pending_text_length_ = text_length;
        text_dirty_ = true;
    }

    void set_printf(const char* format, ...) {
        std::va_list args;
        va_start(args, format);
        vset_printf(format, args);
        va_end(args);
    }

    void set_inverted_line(std::uint8_t line, bool inverted = true) {
        if (line >= inverted_lines_.size())
            return;

        if (inverted_lines_[line] == inverted)
            return;

        inverted_lines_[line] = inverted;
        text_dirty_ = true;
    }

    void command_update() {
        ++(*command_count_);

        try {
            if (i2c_error_seen_.load(std::memory_order_acquire)) {
                cancel_pending_frame(true);
            }

            if (!frame_pending_) {
                if (!text_dirty_ && oled_refresh_countdown_ > 0) {
                    --oled_refresh_countdown_;
                    return;
                }

                if (!prepare_frame(!text_dirty_)) {
                    initialized_ = false;
                    return;
                }

                oled_refresh_countdown_ = oled_refresh_period_;
                if (!frame_pending_)
                    return;
            }

            const auto page_begin = find_next_dirty_page(next_page_to_flush_);
            if (page_begin >= static_cast<std::uint8_t>(Oled::kPages)) {
                finalize_frame();
                return;
            }

            auto page_end = page_begin;
            while (page_end < static_cast<std::uint8_t>(Oled::kPages)
                   && dirty_pages_[page_end] && page_end - page_begin < kPagesPerCommandUpdate) {
                ++page_end;
            }

            if (!oled_.flush_pages(page_begin, page_end)) {
                cancel_pending_frame(true);
                oled_refresh_countdown_ = oled_refresh_period_;
                return;
            }

            for (std::uint8_t page = page_begin; page < page_end; ++page)
                dirty_pages_[page] = false;

            next_page_to_flush_ = page_end;
            if (find_next_dirty_page(next_page_to_flush_) >= static_cast<std::uint8_t>(Oled::kPages))
                finalize_frame();
        } catch (const std::exception& ex) {
            handle_local_send_failure(ex.what());
        } catch (...) {
            handle_local_send_failure("unknown exception");
        }
    }

    void notify_i2c_error(std::uint8_t slave_address) {
        i2c_error_seen_.store(true, std::memory_order_release);
        RCLCPP_WARN(
            logger_, "OLED I2C0 write failed for slave address 0x%02X.",
            static_cast<unsigned int>(slave_address));
    }

    std::uint64_t update_count() const { return *update_count_; }

    std::uint64_t command_count() const { return *command_count_; }

    std::uint8_t i2c_address() const noexcept { return oled_i2c_.i2c_address(); }

    bool has_i2c_error() const noexcept {
        return i2c_error_seen_.load(std::memory_order_acquire);
    }

private:
    static constexpr std::uint8_t kPagesPerCommandUpdate = 1;
    using DisplayBuffer = Oled::DisplayBuffer;

    void handle_local_send_failure(const char* detail) {
        cancel_pending_frame(true);
        oled_refresh_countdown_ = oled_refresh_period_;
        RCLCPP_WARN(logger_, "OLED local send failed: %s", detail);
    }

    void vset_printf(const char* format, std::va_list args) {
        std::array<char, kTextBufferSize> buffer{};
        std::va_list args_copy;
        va_copy(args_copy, args);
        const int written = std::vsnprintf(buffer.data(), buffer.size(), format, args_copy);
        va_end(args_copy);
        if (written < 0) {
            set_text({});
            return;
        }

        const auto text_length = std::min<std::size_t>(
            static_cast<std::size_t>(written), buffer.size() - 1);
        set_text(std::string_view{buffer.data(), text_length});
    }

    void cancel_pending_frame(bool reinitialize_panel) {
        frame_pending_ = false;
        next_page_to_flush_ = 0;
        dirty_pages_.fill(false);
        initialized_ = false;
        if (reinitialize_panel)
            panel_initialized_ = false;
    }

    void finalize_frame() {
        frame_pending_ = false;
        next_page_to_flush_ = 0;
        dirty_pages_.fill(false);
        last_flushed_buffer_ = oled_.display_buffer();
        last_flushed_buffer_valid_ = true;
        initialized_ = !i2c_error_seen_.load(std::memory_order_acquire);
    }

    std::uint8_t find_next_dirty_page(std::uint8_t page_begin) const {
        for (std::uint8_t page = page_begin; page < static_cast<std::uint8_t>(Oled::kPages); ++page) {
            if (dirty_pages_[page])
                return page;
        }
        return static_cast<std::uint8_t>(Oled::kPages);
    }

    void update_dirty_pages(bool force_full_refresh) {
        if (force_full_refresh || !last_flushed_buffer_valid_) {
            dirty_pages_.fill(true);
            return;
        }

        const auto& current = oled_.display_buffer();
        for (std::uint8_t page = 0; page < static_cast<std::uint8_t>(Oled::kPages); ++page)
            dirty_pages_[page] = current[page] != last_flushed_buffer_[page];
    }

    bool prepare_frame(bool force_full_refresh) {
        if (!panel_initialized_) {
            panel_initialized_ = oled_.initialize_without_flush();
            if (!panel_initialized_)
                return false;

            force_full_refresh = true;
        }

        i2c_error_seen_.store(false, std::memory_order_release);
        initialized_ = false;
        oled_.clear();
        oled_.show_text(
            std::string_view{pending_text_buffer_.data(), pending_text_length_}, kFont);
        apply_inverted_lines();
        update_dirty_pages(force_full_refresh);
        frame_pending_ = true;
        next_page_to_flush_ = 0;
        text_dirty_ = false;
        if (find_next_dirty_page(0) >= static_cast<std::uint8_t>(Oled::kPages))
            finalize_frame();
        return true;
    }

    void apply_inverted_lines() {
        constexpr auto line_height = Oled::line_height(kFont);
        for (std::size_t line = 0; line < inverted_lines_.size(); ++line) {
            if (!inverted_lines_[line])
                continue;

            oled_.invert_area(
                0, static_cast<std::int16_t>(line * line_height), static_cast<std::uint8_t>(Oled::kWidth),
                line_height);
        }
    }

    static constexpr std::size_t kTextBufferSize = 96;
    const std::string status_prefix_;
    rclcpp::Logger logger_;
    OledI2c oled_i2c_;
    Oled oled_;
    DisplayBuffer last_flushed_buffer_{};

    std::atomic_bool i2c_error_seen_ = false;
    bool initialized_ = false;
    bool panel_initialized_ = false;
    bool frame_pending_ = false;
    bool last_flushed_buffer_valid_ = false;
    bool text_dirty_ = true;
    unsigned int oled_refresh_period_ = 20;
    unsigned int oled_refresh_countdown_ = 0;
    std::uint8_t next_page_to_flush_ = 0;
    std::array<bool, Oled::kPages> dirty_pages_{};
    std::array<bool, kVisibleTextLineCount> inverted_lines_{};
    std::array<char, kTextBufferSize> pending_text_buffer_{};
    std::size_t pending_text_length_ = 0;

    Component::OutputInterface<std::uint64_t> update_count_;
    Component::OutputInterface<std::uint64_t> command_count_;
    Component::OutputInterface<bool> initialized_output_;
};

} // namespace rmcs_core::hardware::device
