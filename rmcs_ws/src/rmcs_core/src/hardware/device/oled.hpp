#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <functional>
#include <rmcs_executor/component.hpp>
#include <span>
#include <string>
#include <string_view>
#include <utility>

#include "hardware/device/oled_data.hpp"

namespace rmcs_core::hardware::device {

class Oled {
public:
    static constexpr std::int16_t kWidth = 128;
    static constexpr std::int16_t kHeight = 64;
    static constexpr std::int16_t kPages = kHeight / 8;

    using DisplayBuffer = std::array<std::array<std::uint8_t, kWidth>, kPages>;
    using CommandWriter = std::function<void(std::uint8_t)>;
    using DataWriter = std::function<void(std::span<const std::uint8_t>)>;

    enum class FontSize : std::uint8_t {
        k6x8 = 6,
        k8x16 = 8,
    };

    struct Backend {
        CommandWriter write_command;
        DataWriter write_data;
    };

    explicit Oled(Backend backend = {})
        : backend_(std::move(backend)) {}

    explicit Oled(
        rmcs_executor::Component& command_component, const std::string& name_prefix = "/oled",
        Backend backend = {})
        : backend_(std::move(backend)) {
        command_component.register_input(name_prefix + "/text", text_input_, false);
    }

    void configure(Backend backend) { backend_ = std::move(backend); }
    void set_backend(Backend backend) { configure(std::move(backend)); }

    bool has_backend() const {
        return static_cast<bool>(backend_.write_command) && static_cast<bool>(backend_.write_data);
    }

    bool initialize() {
        clear();
        if (!has_backend())
            return false;

        for (auto command : kInitSequence)
            write_command(command);

        return flush();
    }

    bool flush() {
        if (!has_backend())
            return false;

        for (std::uint8_t page = 0; page < kPages; ++page) {
            set_cursor(page, 0);
            write_data({display_buffer_[page].data(), display_buffer_[page].size()});
        }
        return true;
    }

    bool flush_area(std::int16_t x, std::int16_t y, std::uint8_t width, std::uint8_t height) {
        if (!has_backend())
            return false;
        if (width == 0 || height == 0)
            return true;

        std::int16_t page_begin = y / 8;
        std::int16_t page_end = (y + height - 1) / 8 + 1;
        if (y < 0) {
            --page_begin;
            --page_end;
        }

        const std::int16_t start_x = std::clamp<std::int16_t>(x, 0, kWidth);
        const std::int16_t end_x = std::clamp<std::int16_t>(x + width, 0, kWidth);
        if (start_x >= end_x)
            return true;

        for (std::int16_t page = page_begin; page < page_end; ++page) {
            if (page < 0 || page >= kPages)
                continue;

            set_cursor(static_cast<std::uint8_t>(page), static_cast<std::uint8_t>(start_x));
            write_data({
                display_buffer_[page].data() + start_x,
                static_cast<std::size_t>(end_x - start_x),
            });
        }
        return true;
    }

    void clear() {
        for (auto& page : display_buffer_)
            page.fill(0x00);
    }

    bool display_text(std::string_view text, FontSize font_size = FontSize::k6x8) {
        if (!initialized_) {
            initialized_ = initialize();
            if (!initialized_)
                return false;
        }

        if (text == displayed_text_)
            return true;

        displayed_text_.assign(text.data(), text.size());
        clear();
        show_text(displayed_text_, font_size);
        return flush();
    }

    bool update_command(FontSize font_size = FontSize::k6x8) {
        if (!text_input_.ready())
            return true;
        return display_text(*text_input_, font_size);
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
        const std::int16_t line_height = font_size == FontSize::k8x16 ? 16 : 8;
        std::int16_t y = 0;

        while (y < kHeight) {
            const auto newline = text.find('\n');
            const auto line = text.substr(0, newline);
            show_string(0, y, line, font_size);

            if (newline == std::string_view::npos)
                break;

            text.remove_prefix(newline + 1);
            y += line_height;
        }
    }

    void show_num(
        std::int16_t x, std::int16_t y, std::uint32_t number, std::uint8_t length,
        FontSize font_size) {
        for (std::uint8_t i = 0; i < length; ++i) {
            show_char(
                x + static_cast<std::int16_t>(i * font_width(font_size)), y,
                static_cast<char>(number / pow(10, length - i - 1) % 10 + '0'), font_size);
        }
    }

    void show_signed_num(
        std::int16_t x, std::int16_t y, std::int32_t number, std::uint8_t length,
        FontSize font_size) {
        std::uint32_t positive_number = 0;
        if (number >= 0) {
            show_char(x, y, '+', font_size);
            positive_number = static_cast<std::uint32_t>(number);
        } else {
            show_char(x, y, '-', font_size);
            positive_number = static_cast<std::uint32_t>(-(static_cast<std::int64_t>(number)));
        }

        for (std::uint8_t i = 0; i < length; ++i) {
            show_char(
                x + static_cast<std::int16_t>((i + 1) * font_width(font_size)), y,
                static_cast<char>(positive_number / pow(10, length - i - 1) % 10 + '0'),
                font_size);
        }
    }

    void show_hex_num(
        std::int16_t x, std::int16_t y, std::uint32_t number, std::uint8_t length,
        FontSize font_size) {
        for (std::uint8_t i = 0; i < length; ++i) {
            const auto single_number =
                static_cast<std::uint8_t>(number / pow(16, length - i - 1) % 16);
            const char character = single_number < 10
                                     ? static_cast<char>(single_number + '0')
                                     : static_cast<char>(single_number - 10 + 'A');
            show_char(
                x + static_cast<std::int16_t>(i * font_width(font_size)), y, character,
                font_size);
        }
    }

    void show_bin_num(
        std::int16_t x, std::int16_t y, std::uint32_t number, std::uint8_t length,
        FontSize font_size) {
        for (std::uint8_t i = 0; i < length; ++i) {
            show_char(
                x + static_cast<std::int16_t>(i * font_width(font_size)), y,
                static_cast<char>(number / pow(2, length - i - 1) % 2 + '0'), font_size);
        }
    }

    void show_float_num(
        std::int16_t x, std::int16_t y, double number, std::uint8_t int_length,
        std::uint8_t frac_length, FontSize font_size) {
        const std::uint32_t pow_num = pow(10, frac_length);
        if (number >= 0.0) {
            show_char(x, y, '+', font_size);
        } else {
            show_char(x, y, '-', font_size);
            number = -number;
        }

        auto int_num = static_cast<std::uint32_t>(number);
        number -= int_num;
        const auto frac_num = static_cast<std::uint32_t>(std::round(number * pow_num));
        int_num += frac_num / pow_num;

        show_num(x + font_width(font_size), y, int_num, int_length, font_size);
        show_char(
            x + static_cast<std::int16_t>((int_length + 1) * font_width(font_size)), y, '.',
            font_size);
        show_num(
            x + static_cast<std::int16_t>((int_length + 2) * font_width(font_size)), y, frac_num,
            frac_length, font_size);
    }

    void printf(
        std::int16_t x, std::int16_t y, FontSize font_size, const char* format, ...) {
        std::va_list args;
        va_start(args, format);
        vprintf(x, y, font_size, format, args);
        va_end(args);
    }

    void vprintf(
        std::int16_t x, std::int16_t y, FontSize font_size, const char* format,
        std::va_list args) {
        std::array<char, 256> buffer{};
        std::vsnprintf(buffer.data(), buffer.size(), format, args);
        show_string(x, y, buffer.data(), font_size);
    }

    const DisplayBuffer& display_buffer() const { return display_buffer_; }

private:
    static constexpr std::array<std::uint8_t, 23> kInitSequence{
        0xAE, 0xD5, 0x80, 0xA8, 0x3F, 0xD3, 0x00, 0x40, 0xA1, 0xC8,
        0xDA, 0x12, 0x81, 0xCF, 0xD9, 0xF1, 0xDB, 0x30, 0xA4, 0xA6,
        0x8D, 0x14, 0xAF,
    };

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

    void set_cursor(std::uint8_t page, std::uint8_t x) {
        write_command(static_cast<std::uint8_t>(0xB0U | page));
        write_command(static_cast<std::uint8_t>(0x10U | ((x & 0xF0U) >> 4)));
        write_command(static_cast<std::uint8_t>(0x00U | (x & 0x0FU)));
    }

    void write_command(std::uint8_t command) const {
        if (backend_.write_command)
            backend_.write_command(command);
    }

    void write_data(std::span<const std::uint8_t> data) const {
        if (backend_.write_data)
            backend_.write_data(data);
    }

    static std::uint32_t pow(std::uint32_t x, std::uint32_t y) {
        std::uint32_t result = 1;
        while (y-- != 0)
            result *= x;
        return result;
    }

    Backend backend_;
    DisplayBuffer display_buffer_{};
    rmcs_executor::Component::InputInterface<std::string> text_input_;
    std::string displayed_text_;
    bool initialized_ = false;
};

} // namespace rmcs_core::hardware::device
