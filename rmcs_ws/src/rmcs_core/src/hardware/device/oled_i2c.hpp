#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <span>
#include <utility>

#include "hardware/device/oled.hpp"

namespace rmcs_core::hardware::device {

class OledI2c {
public:
    static constexpr std::uint8_t kDefaultI2cAddress = 0x3C;

    using TransferWriter = std::function<void(std::uint8_t, std::span<const std::byte>)>;

    explicit OledI2c(
        TransferWriter writer = {}, std::uint8_t i2c_address = kDefaultI2cAddress) noexcept
        : writer_(std::move(writer))
        , i2c_address_(i2c_address) {}

    void configure(TransferWriter writer) { writer_ = std::move(writer); }

    void set_writer(TransferWriter writer) { configure(std::move(writer)); }

    void set_i2c_address(std::uint8_t i2c_address) noexcept { i2c_address_ = i2c_address; }

    [[nodiscard]] std::uint8_t i2c_address() const noexcept { return i2c_address_; }

    [[nodiscard]] bool has_writer() const noexcept { return static_cast<bool>(writer_); }

    [[nodiscard]] Oled::Backend backend() {
        return {
            .write_commands = [this](std::span<const std::uint8_t> commands) {
                write_commands(commands);
            },
            .write_data = [this](std::span<const std::uint8_t> data) { write_data(data); },
        };
    }

private:
    static constexpr std::uint8_t kCommandControl = 0x00;
    static constexpr std::uint8_t kDataControl = 0x40;
    static constexpr std::size_t kMaxTransferSize = Oled::kWidth;

    void write_commands(std::span<const std::uint8_t> commands) const {
        write_transfer(kCommandControl, commands);
    }

    void write_data(std::span<const std::uint8_t> data) const {
        write_transfer(kDataControl, data);
    }

    void write_transfer(std::uint8_t control, std::span<const std::uint8_t> data) const {
        if (!writer_ || data.empty())
            return;

        // Prefix each transfer with the SSD1306 control byte:
        // 0x00 for commands and 0x40 for display data.
        std::array<std::byte, kMaxTransferSize + 1> frame{};
        frame[0] = static_cast<std::byte>(control);

        for (std::size_t offset = 0; offset < data.size(); offset += kMaxTransferSize) {
            const auto chunk_size = std::min<std::size_t>(kMaxTransferSize, data.size() - offset);
            std::transform(
                data.begin() + static_cast<std::ptrdiff_t>(offset),
                data.begin() + static_cast<std::ptrdiff_t>(offset + chunk_size), frame.begin() + 1,
                [](std::uint8_t value) { return static_cast<std::byte>(value); });

            writer_(i2c_address_, {frame.data(), chunk_size + 1});
        }
    }

    TransferWriter writer_;
    std::uint8_t i2c_address_;
};

} // namespace rmcs_core::hardware::device
