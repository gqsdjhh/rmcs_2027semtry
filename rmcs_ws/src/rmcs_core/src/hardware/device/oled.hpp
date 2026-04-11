#pragma once

#include <algorithm>
#include <array>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <functional>
#include <span>
#include <string>
#include <string_view>
#include <utility>

#include "hardware/device/oled_data.hpp"

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

class OledShow {
public:
    static constexpr std::int16_t kWidth = OledConfig::kSupportedWidth;
    static constexpr std::int16_t kHeight = OledConfig::kSupportedHeight;
    static constexpr std::int16_t kPages = OledConfig::kSupportedPages;

    using DisplayBuffer = std::array<std::array<std::uint8_t, kWidth>, kPages>;

    enum class FontSize : std::uint8_t {
        k6x8 = 6,
        k8x16 = 8,
    };

    virtual ~OledShow() = default;

    static constexpr std::uint8_t line_height(FontSize font_size) {
        return font_size == FontSize::k8x16 ? 16 : 8;
    }

    void clear() {
        on_framebuffer_mutated();
        for (auto& page : display_buffer_)
            page.fill(0x00);
    }

    void show_char(std::int16_t x, std::int16_t y, char character, FontSize font_size) {
        if (character < ' ' || character > '~')
            character = '?';

        const auto index = static_cast<std::size_t>(character - ' ');
        if (font_size == FontSize::k8x16) {
            blit_image(x, y, 8, 16, detail::kFont8x16[index]);
        } else {
            blit_image(x, y, 6, 8, detail::kFont6x8[index]);
        }
    }

    void show_string(std::int16_t x, std::int16_t y, std::string_view string, FontSize font_size) {
        std::uint16_t x_offset = 0;

        for (std::size_t i = 0; i < string.size();) {
            const auto char_length = utf8_char_length(static_cast<std::uint8_t>(string[i]));
            if (char_length == 0) {
                ++i;
                continue;
            }
            if (i + char_length > string.size())
                break;

            const auto character = string.substr(i, char_length);
            i += char_length;

            if (char_length == 1) {
                show_char(x + x_offset, y, character.front(), font_size);
                x_offset += font_width(font_size);
                continue;
            }

            if (font_size != FontSize::k8x16) {
                show_char(x + x_offset, y, '?', FontSize::k6x8);
                x_offset += font_width(FontSize::k6x8);
                continue;
            }

            std::size_t glyph_index = 0;
            for (; detail::kChineseFont16x16[glyph_index].index[0] != '\0'; ++glyph_index) {
                if (character == detail::kChineseFont16x16[glyph_index].index)
                    break;
            }

            blit_image(x + x_offset, y, 16, 16, detail::kChineseFont16x16[glyph_index].data);
            x_offset += 16;
        }
    }

    void show_text(std::string_view text, FontSize font_size = FontSize::k6x8) {
        const std::int16_t line_height_value = line_height(font_size);
        std::int16_t y = 0;

        while (y < kHeight) {
            const auto newline = text.find('\n');
            const auto line = text.substr(0, newline);
            show_string(0, y, line, font_size);

            if (newline == std::string_view::npos)
                break;

            text.remove_prefix(newline + 1);
            y += line_height_value;
        }
    }

    [[nodiscard]] const DisplayBuffer& display_buffer() const { return display_buffer_; }

protected:
    [[nodiscard]] DisplayBuffer& mutable_display_buffer() { return display_buffer_; }

    static std::uint8_t utf8_char_length(std::uint8_t first_byte) {
        if ((first_byte & 0x80U) == 0x00U)
            return 1;
        if ((first_byte & 0xE0U) == 0xC0U)
            return 2;
        if ((first_byte & 0xF0U) == 0xE0U)
            return 3;
        if ((first_byte & 0xF8U) == 0xF0U)
            return 4;
        return 0;
    }

    static std::uint8_t font_width(FontSize font_size) {
        return static_cast<std::uint8_t>(font_size);
    }

    void clear_area(std::int16_t x, std::int16_t y, std::uint8_t width, std::uint8_t height) {
        for (std::int16_t j = y; j < y + height; ++j) {
            for (std::int16_t i = x; i < x + width; ++i) {
                if (i < 0 || i >= kWidth || j < 0 || j >= kHeight)
                    continue;

                display_buffer_[j / 8][i] &= static_cast<std::uint8_t>(~(0x01U << (j % 8)));
            }
        }
    }

    void blit_image(
        std::int16_t x, std::int16_t y, std::uint8_t width, std::uint8_t height,
        const std::uint8_t* image) {
        if (width == 0 || height == 0 || image == nullptr)
            return;

        on_framebuffer_mutated();
        clear_area(x, y, width, height);

        for (std::uint8_t page_index = 0; page_index < (height - 1) / 8 + 1; ++page_index) {
            for (std::uint8_t column = 0; column < width; ++column) {
                if (x + column < 0 || x + column >= kWidth)
                    continue;

                auto page = static_cast<std::int16_t>(y / 8);
                auto shift = static_cast<std::int16_t>(y % 8);
                if (y < 0) {
                    --page;
                    shift += 8;
                }

                if (page + page_index >= 0 && page + page_index < kPages) {
                    display_buffer_[page + page_index][x + column] |=
                        static_cast<std::uint8_t>(image[page_index * width + column] << shift);
                }
                if (page + page_index + 1 >= 0 && page + page_index + 1 < kPages) {
                    display_buffer_[page + page_index + 1][x + column] |= static_cast<std::uint8_t>(
                        image[page_index * width + column] >> (8 - shift));
                }
            }
        }
    }

    static std::uint32_t pow(std::uint32_t x, std::uint32_t y) {
        std::uint32_t result = 1;
        while (y-- != 0)
            result *= x;
        return result;
    }

    virtual void on_framebuffer_mutated() {}

private:
    DisplayBuffer display_buffer_{};
};

class Oled : public OledShow {
public:
    using DisplayBuffer = OledShow::DisplayBuffer;
    using FontSize = OledShow::FontSize;

    static constexpr std::int16_t kWidth = OledShow::kWidth;
    static constexpr std::int16_t kHeight = OledShow::kHeight;
    static constexpr std::int16_t kPages = OledShow::kPages;

    using CommandWriter = std::function<void(std::span<const std::uint8_t>)>;
    using DataWriter = std::function<void(std::span<const std::uint8_t>)>;

    struct Backend {
        CommandWriter write_commands;
        DataWriter write_data;
    };

    explicit Oled(Backend backend = {}, OledConfig config = {})
        : backend_(std::move(backend))
        , config_(config) {}

    void configure(Backend backend) { backend_ = std::move(backend); }
    void configure(const OledConfig& config) { config_ = config; }

    void configure(Backend backend, const OledConfig& config) {
        backend_ = std::move(backend);
        config_ = config;
    }

    void set_backend(Backend backend) { configure(std::move(backend)); }
    void set_config(const OledConfig& config) { configure(config); }

    [[nodiscard]] const OledConfig& config() const { return config_; }

    bool has_backend() const {
        return static_cast<bool>(backend_.write_commands) && static_cast<bool>(backend_.write_data);
    }

    bool initialize_without_flush() {
        if (initialized_)
            return true;

        clear();
        if (!has_backend() || !config_.uses_supported_geometry()
            || !config_.uses_supported_transport_mode())
            return false;

        const auto init_sequence = build_init_sequence();
        write_commands(init_sequence);
        initialized_ = true;
        return true;
    }

    bool initialize() {
        if (!initialize_without_flush())
            return false;

        return flush();
    }

    bool flush() {
        return flush_pages(0, static_cast<std::uint8_t>(kPages));
    }

    bool flush_pages(std::uint8_t page_begin, std::uint8_t page_end) {
        if (!has_backend())
            return false;

        page_begin = std::min<std::uint8_t>(page_begin, static_cast<std::uint8_t>(kPages));
        page_end = std::min<std::uint8_t>(page_end, static_cast<std::uint8_t>(kPages));
        if (page_begin >= page_end)
            return true;

        const auto& current = display_buffer();
        for (std::uint8_t page = page_begin; page < page_end; ++page) {
            set_cursor(page, 0);
            write_data({current[page].data(), current[page].size()});
        }
        return true;
    }

    bool flush_area(std::int16_t x, std::int16_t y, std::uint8_t width, std::uint8_t height) {
        if (!has_backend())
            return false;
        if (width == 0 || height == 0)
            return true;

        const auto page_begin = page_for_y(y);
        const auto page_end = static_cast<std::int16_t>(
            page_for_y(static_cast<std::int32_t>(y) + height - 1) + 1);

        const std::int16_t start_x = std::clamp<std::int16_t>(x, 0, kWidth);
        const std::int16_t end_x = std::clamp<std::int16_t>(x + width, 0, kWidth);
        if (start_x >= end_x)
            return true;

        const auto& current = display_buffer();
        for (std::int16_t page = page_begin; page < page_end; ++page) {
            if (page < 0 || page >= kPages)
                continue;

            set_cursor(static_cast<std::uint8_t>(page), static_cast<std::uint8_t>(start_x));
            write_data({
                current[page].data() + start_x,
                static_cast<std::size_t>(end_x - start_x),
            });
        }
        return true;
    }

    /// Clear the whole screen, render the text, then flush the full framebuffer.
    /// Supports preformatted textual data passed as `std::string_view`, such as `std::string`,
    /// string literals, and manually formatted numeric/status strings.
    bool display_text(std::string_view text, FontSize font_size = FontSize::k6x8) {
        if (!initialized_) {
            initialized_ = initialize();
            if (!initialized_)
                return false;
        }

        if (text_cache_valid_ && font_size == displayed_font_size_ && text == displayed_text_)
            return true;

        displayed_text_.assign(text.data(), text.size());
        displayed_font_size_ = font_size;
        clear();
        show_text(displayed_text_, font_size);
        text_cache_valid_ = true;
        return flush();
    }

    /// Clear a single text line, render new content at the line origin, then flush that region.
    /// Supports the same preformatted textual data types as `display_text`.
    bool display_line_text(
        std::uint8_t line, std::string_view text, FontSize font_size = FontSize::k6x8) {
        if (!initialized_) {
            initialized_ = initialize();
            if (!initialized_)
                return false;
        }

        const auto y = static_cast<std::int16_t>(line) * line_height(font_size);
        if (y >= kHeight)
            return true;

        clear_area(0, y, static_cast<std::uint8_t>(kWidth), line_height(font_size));
        show_string(0, y, text, font_size);
        return flush_area(0, y, static_cast<std::uint8_t>(kWidth), line_height(font_size));
    }

    /// Clear the target text bounds, render at the given position, then flush that region.
    /// Supports the same preformatted textual data types as `display_text`.
    bool display_at_text(
        std::int16_t x, std::int16_t y, std::string_view text, FontSize font_size = FontSize::k6x8) {
        if (!initialized_) {
            initialized_ = initialize();
            if (!initialized_)
                return false;
        }

        const auto width = text_width(text, font_size);
        const auto height = line_height(font_size);
        if (width == 0 || height == 0)
            return true;

        clear_area(x, y, width, height);
        show_string(x, y, text, font_size);
        return flush_area(x, y, width, height);
    }

    /// Format into a temporary buffer, clear the whole screen, then display the formatted text.
    /// Supports common `printf`-style data types through format specifiers, including signed and
    /// unsigned integers, floating-point values, characters, C strings, and pointers.
    bool display_printf(FontSize font_size, const char* format, ...) {
        std::va_list args;
        va_start(args, format);
        const auto result = vdisplay_printf(font_size, format, args);
        va_end(args);
        return result;
    }

    /// `va_list` variant of `display_printf`, with the same supported `printf`-style data types.
    bool vdisplay_printf(FontSize font_size, const char* format, std::va_list args) {
        std::array<char, kFormatBufferSize> buffer{};
        return display_text(format_to_string_view(buffer, format, args), font_size);
    }

    /// Format into a temporary buffer, clear one line, then display and flush that line.
    /// Supports the same `printf`-style data types as `display_printf`.
    bool display_line_printf(std::uint8_t line, FontSize font_size, const char* format, ...) {
        std::va_list args;
        va_start(args, format);
        const auto result = vdisplay_line_printf(line, font_size, format, args);
        va_end(args);
        return result;
    }

    /// `va_list` variant of `display_line_printf`, with the same supported `printf`-style data
    /// types.
    bool vdisplay_line_printf(
        std::uint8_t line, FontSize font_size, const char* format, std::va_list args) {
        std::array<char, kFormatBufferSize> buffer{};
        return display_line_text(line, format_to_string_view(buffer, format, args), font_size);
    }

    /// Format into a temporary buffer, clear the target bounds, then display and flush that area.
    /// Supports the same `printf`-style data types as `display_printf`.
    bool display_at_printf(
        std::int16_t x, std::int16_t y, FontSize font_size, const char* format, ...) {
        std::va_list args;
        va_start(args, format);
        const auto result = vdisplay_at_printf(x, y, font_size, format, args);
        va_end(args);
        return result;
    }

    /// `va_list` variant of `display_at_printf`, with the same supported `printf`-style data
    /// types.
    bool vdisplay_at_printf(
        std::int16_t x, std::int16_t y, FontSize font_size, const char* format, std::va_list args) {
        std::array<char, kFormatBufferSize> buffer{};
        return display_at_text(x, y, format_to_string_view(buffer, format, args), font_size);
    }

protected:
    void on_framebuffer_mutated() override { text_cache_valid_ = false; }

private:
    static constexpr std::size_t kFormatBufferSize = 256;

    [[nodiscard]] std::array<std::uint8_t, 25> build_init_sequence() const {
        return {
            0xAE,
            0xD5,
            config_.oscillator_frequency,
            0x20,
            static_cast<std::uint8_t>(config_.addressing_mode),
            0xA8,
            static_cast<std::uint8_t>(config_.height - 1),
            0xD3,
            config_.display_offset,
            static_cast<std::uint8_t>(0x40U | (config_.start_line & 0x3FU)),
            static_cast<std::uint8_t>(config_.segment_remap ? 0xA1U : 0xA0U),
            static_cast<std::uint8_t>(config_.com_scan_reverse ? 0xC8U : 0xC0U),
            0xDA,
            config_.com_pins_hardware_config,
            0x81,
            config_.contrast,
            0xD9,
            config_.precharge_period,
            0xDB,
            config_.vcomh_deselect_level,
            static_cast<std::uint8_t>(config_.entire_display_on ? 0xA5U : 0xA4U),
            static_cast<std::uint8_t>(config_.inverse_display ? 0xA7U : 0xA6U),
            0x8D,
            static_cast<std::uint8_t>(config_.enable_charge_pump ? 0x14U : 0x10U),
            0xAF,
        };
    }

    static std::string_view format_to_string_view(
        std::array<char, kFormatBufferSize>& buffer, const char* format, std::va_list args) {
        std::va_list args_copy;
        va_copy(args_copy, args);
        const auto written = std::vsnprintf(buffer.data(), buffer.size(), format, args_copy);
        va_end(args_copy);

        if (written <= 0)
            return {};

        const auto length = std::min<std::size_t>(
            static_cast<std::size_t>(written), buffer.size() - 1);
        return {buffer.data(), length};
    }

    static std::uint8_t text_width(std::string_view text, FontSize font_size) {
        std::uint16_t width = 0;

        for (std::size_t i = 0; i < text.size();) {
            const auto char_length = utf8_char_length(static_cast<std::uint8_t>(text[i]));
            if (char_length == 0) {
                ++i;
                continue;
            }
            if (i + char_length > text.size())
                break;

            i += char_length;
            width += char_length == 1 ? font_width(font_size)
                                      : (font_size == FontSize::k8x16 ? 16 : font_width(FontSize::k6x8));
        }

        return static_cast<std::uint8_t>(std::min<std::uint16_t>(width, kWidth));
    }

    static std::int16_t page_for_y(std::int32_t y) {
        auto page = static_cast<std::int16_t>(y / 8);
        if (y < 0 && y % 8 != 0)
            --page;
        return page;
    }

    void set_cursor(std::uint8_t page, std::uint8_t x) {
        const std::array<std::uint8_t, 3> cursor_commands{
            static_cast<std::uint8_t>(0xB0U | page),
            static_cast<std::uint8_t>(0x10U | ((x & 0xF0U) >> 4)),
            static_cast<std::uint8_t>(0x00U | (x & 0x0FU)),
        };
        write_commands(cursor_commands);
    }

    void write_commands(std::span<const std::uint8_t> commands) const {
        if (backend_.write_commands)
            backend_.write_commands(commands);
    }

    void write_data(std::span<const std::uint8_t> data) const {
        if (backend_.write_data)
            backend_.write_data(data);
    }

    Backend backend_;
    OledConfig config_;
    std::string displayed_text_;
    FontSize displayed_font_size_ = FontSize::k6x8;
    bool text_cache_valid_ = false;
    bool initialized_ = false;
};

} // namespace rmcs_core::hardware::device
