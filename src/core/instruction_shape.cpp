// SPDX-License-Identifier: GPL-3.0-only
#include "dsp56300/memory.hpp"
#include "instruction.hpp"
#include "instruction_decoder.hpp"

#include <optional>

namespace dsp56300 {
std::optional<InstructionShape> inspect_instruction(const Word opcode) noexcept {
  if (opcode > word_mask) { return std::nullopt; }
  const auto decoded = detail::decode(opcode);
  if (decoded.operation == detail::Operation::unsupported) { return std::nullopt; }
  return InstructionShape{.kind = decoded.operation,
                          .words = decoded.words,
                          .repeatable =
                              decoded.words == 1 && detail::repeatable(decoded.operation)};
}
} // namespace dsp56300
