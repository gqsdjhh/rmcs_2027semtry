#pragma once

#include <cstdint>

namespace rmcs_core::hardware::device {

struct OledConfig {
    static constexpr std::int16_t kSupportedWidth = 128;
    static constexpr std::int16_t kSupportedHeight = 64;
    static constexpr std::int16_t kSupportedPages = kSupportedHeight / 8;

    enum class Controller : std::uint8_t {
        kSsd1306,
    };

    enum class AddressingMode : std::uint8_t {
        kHorizontal = 0x00,
        kVertical = 0x01,
        kPage = 0x02,
    };

    [[nodiscard]] bool uses_supported_geometry() const {
        return width == kSupportedWidth && height == kSupportedHeight;
    }

    [[nodiscard]] bool uses_supported_transport_mode() const {
        return controller == Controller::kSsd1306 && addressing_mode == AddressingMode::kPage;
    }

    Controller controller = Controller::kSsd1306;
    AddressingMode addressing_mode = AddressingMode::kPage;

    std::int16_t width = kSupportedWidth;
    std::int16_t height = kSupportedHeight;

    std::uint8_t i2c_address = 0x3C;
    std::uint8_t display_offset = 0x00;
    std::uint8_t start_line = 0x00;
    std::uint8_t contrast = 0xCF;
    std::uint8_t oscillator_frequency = 0x80;
    std::uint8_t precharge_period = 0xF1;
    std::uint8_t vcomh_deselect_level = 0x30;
    std::uint8_t com_pins_hardware_config = 0x12;

    bool segment_remap = true;
    bool com_scan_reverse = true;
    bool inverse_display = false;
    bool entire_display_on = false;
    bool enable_charge_pump = true;
};

} // namespace rmcs_core::hardware::device
