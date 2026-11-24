// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include <array>
#include <cstdint>

namespace dsp56300::detail {
// Encoded mixed-product operands in X0, Y0, X1, Y1 order.
inline constexpr std::array<std::array<std::uint8_t, 2>, 16> mixed_product_pairs{{{0, 0},
                                                                                  {1, 1},
                                                                                  {2, 0},
                                                                                  {3, 1},
                                                                                  {0, 3},
                                                                                  {1, 0},
                                                                                  {2, 1},
                                                                                  {3, 2},
                                                                                  {2, 2},
                                                                                  {3, 3},
                                                                                  {0, 2},
                                                                                  {1, 3},
                                                                                  {3, 0},
                                                                                  {0, 1},
                                                                                  {1, 2},
                                                                                  {2, 3}}};
}
