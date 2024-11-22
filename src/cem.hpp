#pragma once

#include <cstdint>
#include <limits>
#include <ranges>
#include <type_traits>

namespace cem {

template <typename T>
concept numeric_t = std::is_arithmetic_v<T>;

template <typename T>
concept floating_t = std::is_floating_point_v<T>;

consteval auto max(const numeric_t auto a, const numeric_t auto b) noexcept {
    return (a > b) ? a : b;
}

consteval auto max(const numeric_t auto first,
                   const numeric_t auto... rest) noexcept {
    if constexpr (!sizeof...(rest)) {
        return first;
    } else {
        return max(first, max(rest...));
    }
}

consteval auto abs(numeric_t auto value) noexcept {
    if constexpr (std::is_unsigned_v<decltype(value)>) {
        return value;
    }

    return value < 0 ? -value : value;
}

consteval auto sqrt(const floating_t auto x) noexcept {
    using type_t = decltype(x);

    if (x < 0) {
        return std::numeric_limits<type_t>::quiet_NaN();
    }

    if (x == 0) {
        return static_cast<type_t>(0);
    }

    auto guess = x / 2.0;

    const auto relative_epsilon = std::numeric_limits<type_t>::epsilon() * x;
    constexpr static const auto absolute_epsilon =
        std::numeric_limits<type_t>::epsilon();

    for ([[maybe_unused]] auto i : std::views::iota(0, 10)) {
        auto new_guess = (guess + x / guess) / 2.0;
        auto guess_abs_diff = abs(new_guess - guess);
        auto max_of_epsilon = max(absolute_epsilon, relative_epsilon);

        if (guess_abs_diff < max_of_epsilon) {
            return new_guess;
        }
        guess = new_guess;
    }

    return guess;
}

consteval auto Q_rsqrt(const floating_t auto x) noexcept {
    using type_t = decltype(x);
    constexpr static const type_t half = 0.5;
    constexpr static const type_t three_halfs = 0.5 * 3;

    constexpr static const auto magic = []() -> auto {
        if constexpr (std::same_as<type_t, const float>) {
            return std::uint32_t{0x5f3759df};
        } else if constexpr (std::same_as<type_t, const double>) {
            return std::uint64_t{0x5fe6eb50c7b537a9};
        } else {
            throw "Unsupported floating type";
        }
    }();

    constexpr auto compute_rsqrt = [](auto bit_value, auto input) -> type_t {
        bit_value = magic - (bit_value >> 1);
        type_t y = std::bit_cast<type_t>(bit_value);
        return y * (three_halfs - (input * half * y * y));
    };

    auto bit_value = std::bit_cast<decltype(magic)>(x);
    return compute_rsqrt(bit_value, x);
}

consteval auto are_approximately_equal(const numeric_t auto computed,
                                       const numeric_t auto expected,
                                       auto epsilon = 1e-2) noexcept {
    return abs(computed - expected) <= epsilon;
}

} // namespace cem
