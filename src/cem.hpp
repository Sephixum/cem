#pragma once

#include <cstdint>
#include <functional>
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

template <std::size_t max_iter = 10>
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
    static constexpr const auto absolute_epsilon =
        std::numeric_limits<type_t>::epsilon();

    for (std::size_t i{0}; i < max_iter; ++i) {
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

    static constexpr type_t half = 0.5;
    static constexpr type_t three_halfs = 0.5 * 3;

    static constexpr const auto magic = std::invoke([]() {
        if constexpr (std::same_as<type_t, const float>) {
            return std::uint32_t{0x5f3759df};
        } else if constexpr (std::same_as<type_t, const double>) {
            return std::uint64_t{0x5fe6eb50c7b537a9};
        } else {
            throw "Unsupported floating type";
        }
    });

    auto bit_value = std::bit_cast<decltype(magic)>(x);

    return std::invoke(
        [](auto bit_value, auto input) -> type_t {
            bit_value = magic - (bit_value >> 1);

            using mutable_type_t = std::remove_const_t<type_t>;
            mutable_type_t y = std::bit_cast<type_t>(bit_value);

            y = y * (three_halfs - (input * half * y * y));
            y = y * (three_halfs - (input * half * y * y));
            return y;
        },
        bit_value, x);
}

consteval auto are_approximately_equal(const numeric_t auto computed,
                                       const numeric_t auto expected,
                                       auto epsilon = 1e-2) noexcept {
    return abs(computed - expected) <= epsilon;
}

consteval auto quadratic_results(floating_t auto a, floating_t auto b,
                                 floating_t auto c) {
    using common_t = decltype(a + b + c);

    if (a == common_t{0}) {
        return std::tuple{false, common_t{0}, common_t{0}};
    }

    common_t discrminant = b * b - static_cast<common_t>(4) * a * c;
    if (discrminant < 0) {
        return std::tuple{false, common_t{0}, common_t{0}};
    }

    common_t sqrt_discrminant = sqrt(static_cast<double>(discrminant));
    common_t root1 = (-b + sqrt_discrminant) / (static_cast<common_t>(2) * a);
    common_t root2 = (-b - sqrt_discrminant) / (static_cast<common_t>(2) * a);

    return std::tuple{false, root1, root2};
}

} // namespace cem
