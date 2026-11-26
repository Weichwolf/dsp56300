// SPDX-License-Identifier: GPL-3.0-only
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "jit/fixed_pass.hpp"
#include "jit/image_plan.hpp"

#include <array>
#include <cstdlib>
#include <iostream>
#include <optional>
#include <span>

using dsp56300::CodeSpan;
using dsp56300::Word;
using dsp56300::detail::FixedPassCost;
using dsp56300::detail::ImagePlan;
using dsp56300::detail::plan_image;

int main() {
  constexpr Word head = 0x100;
  constexpr Word callee = 0x200;
  constexpr Word branch = 0x0c0000;
  constexpr Word call = 0x0d0000;
  constexpr Word conditional_call = 0x0f0000;
  constexpr Word rts = 0x00000c;
  constexpr Word rep = 0x0603a0;
  constexpr Word wait = 0x86;
  constexpr Word cas = 0x0b0080;
  constexpr Word loop_three = 0x060380;
  unsigned checked = 0;
  const auto check = [&](const ImagePlan &plan,
                         const std::span<const Word> path,
                         const std::optional<FixedPassCost> expected) {
    const auto got = plan.fixed_pass_cost(path);
    if (got != expected) {
      std::cerr << "case " << checked + 1 << " got ";
      if (got) { std::cerr << got->instructions << ' ' << got->cycles; }
      std::cerr << '\n';
      std::abort();
    }
    ++checked;
  };
  const auto plan = [&](const std::span<const Word> caller, const std::span<const Word> body) {
    const std::array ranges{CodeSpan{.address = head, .words = caller},
                            CodeSpan{.address = callee, .words = body}};
    return plan_image(ranges, std::array{head});
  };
  constexpr std::array regular{call | callee, branch | head};
  constexpr std::array regular_body{Word{0}, rts};
  const auto calls = plan(regular, regular_body);
  check(calls,
        std::array{head, callee, callee + 1, head + 1},
        FixedPassCost{.instructions = 4, .cycles = 10});
  check(calls, std::array{head, head + 1}, {});
  check(calls, std::array{head, callee, callee + 1, head + 1, head}, {});
  check(calls, std::array{head, callee, callee + 1, callee}, {});
  check(calls, std::array{head, callee + 2}, {});
  check(calls, std::array{head, callee, callee + 1}, {});
  check(calls, std::array<Word, 0>{}, {});
  check(calls, std::array{callee + 1}, {});

  const auto conditional = plan(std::array{conditional_call | callee, branch | head}, regular_body);
  check(conditional, std::array{head, head + 1}, FixedPassCost{.instructions = 2, .cycles = 7});
  check(conditional,
        std::array{head, callee, callee + 1, head + 1},
        FixedPassCost{.instructions = 4, .cycles = 11});
  const auto ambiguous =
      plan(std::array{conditional_call | (head + 1), branch | head}, regular_body);
  check(ambiguous, std::array{head, head + 1}, {});

  const auto bit_call = plan(std::array{cas, callee, branch | head}, regular_body);
  check(bit_call, std::array{head, head + 2}, FixedPassCost{.instructions = 2, .cycles = 7});
  check(bit_call,
        std::array{head, callee, callee + 1, head + 2},
        FixedPassCost{.instructions = 4, .cycles = 11});
  check(bit_call, std::array{head, head + 1, head + 2}, {});

  const auto repeat = plan(std::array{rep, Word{0}, branch | head}, regular_body);
  check(repeat, std::array{head, head + 1, head + 2}, {});
  const auto sleeping = plan(std::array{wait, branch | head}, regular_body);
  check(sleeping, std::array{head, head + 1}, {});
  const auto nested =
      plan(std::array{call | callee, branch | head}, std::array{call | (callee + 2), rts, rts});
  check(nested,
        std::array{head, callee, callee + 2, callee + 1, head + 1},
        FixedPassCost{.instructions = 5, .cycles = 15});
  check(nested, std::array{head, callee, callee + 2, head + 1}, {});
  const auto loop = plan(std::array{loop_three, head + 2, Word{0}, branch | head}, regular_body);
  check(loop,
        std::array{head, head + 2, head + 2, head + 2, head + 3},
        FixedPassCost{.instructions = 5, .cycles = 11});
  check(loop, std::array{head, head + 2, head + 2, head + 3}, {});
  check(loop, std::array{head, head + 2, head + 2, head + 2, head + 2, head + 3}, {});
  const auto zero =
      plan(std::array{Word{0x060080}, head + 2, Word{0}, branch | head}, regular_body);
  check(zero, std::array{head, head + 3}, FixedPassCost{.instructions = 2, .cycles = 8});
  check(zero, std::array{head, head + 2, head + 3}, {});
  const auto non_linear =
      plan(std::array{loop_three, head + 2, branch | head, branch | head}, regular_body);
  check(non_linear, std::array{head, head + 2, head + 2, head + 2, head + 3}, {});
  std::cout << checked << " decoded path controls passed\n";
}
