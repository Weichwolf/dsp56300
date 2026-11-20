// SPDX-License-Identifier: GPL-3.0-only
#include "artifact.hpp"
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "image.hpp"
#include "instruction.hpp"
#include "jit/image_plan.hpp"
#include "semantic_ir.hpp"

#include <cstddef>
#include <cstdint>
#include <map>
#include <memory>
#include <set>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <tuple>
#include <utility>
#include <vector>

namespace dsp56300 {
namespace {
using detail::ArtifactReader;
using detail::ArtifactWriter;
using VectorKey = std::tuple<Word, Word, Word>;
constexpr std::string_view format = "dsp56300.semantic.2";

void sequence(ArtifactWriter &writer, const auto &values, const auto &write) {
  writer.integer(values.size());
  for (const auto &value : values) { write(value); }
}

void sequence(ArtifactReader &reader, const auto &read) {
  for (auto count = reader.count(sizeof(std::uint64_t)); count != 0; --count) { read(); }
}

Word word(ArtifactReader &reader) {
  const auto value = reader.integer<Word>();
  if (value > word_mask) { ArtifactReader::fail(); }
  return value;
}

bool flag(ArtifactReader &reader) {
  const auto value = reader.integer<unsigned>();
  if (value > 1) { ArtifactReader::fail(); }
  return value != 0;
}

template <class Value, class Read>
std::map<Word, Value> mapping(ArtifactReader &reader, const Read &read) {
  std::map<Word, Value> result;
  sequence(reader, [&] {
    const auto address = word(reader);
    auto value = read();
    if (!result.emplace(address, std::move(value)).second) { ArtifactReader::fail(); }
  });
  return result;
}

void instructions(ArtifactWriter &writer, const std::map<Word, detail::Instruction> &values) {
  sequence(writer, values, [&](const auto &entry) {
    const auto &[address, instruction] = entry;
    writer.integer(address);
    writer.integer(instruction.opcode);
    writer.integer(instruction.extension);
    writer.integer(static_cast<unsigned>(instruction.operation));
    writer.integer(instruction.words);
    writer.integer(instruction.cycles);
  });
}

std::map<Word, detail::Instruction> instructions(ArtifactReader &reader) {
  return mapping<detail::Instruction>(reader, [&] {
    auto instruction = detail::decode(word(reader));
    instruction.extension = word(reader);
    const auto operation = reader.integer<unsigned>();
    const auto words = reader.integer<unsigned>();
    const auto cycles = reader.integer<unsigned>();
    if (operation != static_cast<unsigned>(instruction.operation) || words != instruction.words ||
        cycles != instruction.cycles) {
      ArtifactReader::fail();
    }
    return instruction;
  });
}

void key(ArtifactWriter &writer, const VectorKey &value) {
  const auto &[address, first, second] = value;
  writer.integer(address);
  writer.integer(first);
  writer.integer(second);
}

VectorKey key(ArtifactReader &reader) {
  const auto address = word(reader);
  const auto first = word(reader);
  const auto second = word(reader);
  return {address, first, second};
}

void variant(ArtifactWriter &writer, const detail::ImageContracts::Variant &value) {
  writer.integer(value.id);
  std::size_t words = 0;
  for (const auto &run : value.program) { words += run.words.size(); }
  writer.integer(words);
  for (const auto &run : value.program) {
    for (std::size_t index = 0; index < run.words.size(); ++index) {
      writer.integer(run.first + index);
      writer.integer(run.words[index]);
    }
  }
  instructions(writer, value.instructions);
  sequence(writer, value.transitions, [&](const auto &entry) {
    writer.integer(entry.first);
    sequence(writer, entry.second, [&](const auto target) { writer.integer(target); });
  });
  sequence(writer, value.failed_fetches, [&](const auto address) { writer.integer(address); });
  writer.integer(static_cast<std::uint64_t>(value.fixed_memory));
  sequence(writer, value.memory, [&](const auto region) {
    writer.integer(static_cast<unsigned>(region.space));
    writer.integer(region.first);
    writer.integer(region.size);
    writer.integer(static_cast<std::uint64_t>(region.writable));
    writer.integer(static_cast<std::uint64_t>(region.tracked));
    writer.integer(static_cast<std::uint64_t>(region.aliases_program));
  });
  sequence(writer, value.prefixes, [&](const auto &entry) {
    writer.integer(entry.first);
    writer.integer(entry.second);
  });
}

detail::ImageContracts::Variant variant(ArtifactReader &reader) {
  detail::ImageContracts::Variant value;
  value.id = reader.integer();
  for (auto count = reader.count(2 * sizeof(std::uint64_t)); count != 0; --count) {
    const auto address = word(reader);
    const auto data = word(reader);
    if (!value.program.empty() &&
        address < value.program.back().first + value.program.back().words.size()) {
      ArtifactReader::fail();
    }
    value.append_word({address, data});
  }
  value.instructions = instructions(reader);
  value.transitions = mapping<std::vector<std::uint64_t>>(reader, [&] {
    std::vector<std::uint64_t> targets;
    sequence(reader, [&] { targets.push_back(reader.integer()); });
    return targets;
  });
  sequence(reader, [&] {
    if (!value.failed_fetches.insert(word(reader)).second) { ArtifactReader::fail(); }
  });
  value.fixed_memory = flag(reader);
  sequence(reader, [&] {
    const auto space = reader.integer<unsigned>();
    if (space > static_cast<unsigned>(Space::y)) { ArtifactReader::fail(); }
    const auto first = word(reader);
    const auto size = reader.integer<Word>();
    const bool writable = flag(reader);
    const bool tracked = flag(reader);
    const bool aliases = flag(reader);
    if (size == 0 || size > address_words - first) { ArtifactReader::fail(); }
    value.memory.push_back({.space = static_cast<Space>(space),
                            .first = first,
                            .size = size,
                            .writable = writable,
                            .tracked = tracked,
                            .aliases_program = aliases});
  });
  value.prefixes = mapping<std::uint64_t>(reader, [&] { return reader.integer(); });
  return value;
}
}

std::string_view Core::compiler_identity() noexcept {
  return detail::compiler_identity();
}

void SemanticImage::Impl::serialize(const std::string_view input_identity) {
  ArtifactWriter writer;
  writer.text(format);
  writer.text(Core::compiler_identity());
  writer.text(input_identity);
  writer.text(identity);
  writer.bytes(bitcode);
  writer.integer(static_cast<unsigned>(memory_mode));
  sequence(writer, exports, [&](const auto symbol) {
    writer.text(symbol.name);
    writer.text(symbol.abi);
  });
  sequence(writer, variants, [&](const auto &value) { variant(writer, value); });
  sequence(writer, interrupts, [&](const auto &entry) {
    writer.integer(entry.first);
    writer.integer(entry.second);
  });
  sequence(writer, vector_plans, [&](const auto &entry) {
    key(writer, entry.first);
    instructions(writer, entry.second.instructions);
    sequence(writer, entry.second.entries, [&](const auto address) { writer.integer(address); });
  });
  sequence(writer, variant_names, [&](const auto &name) { writer.text(name); });
  sequence(writer, vector_symbols, [&](const auto &entry) {
    key(writer, entry.first);
    writer.text(entry.second);
  });
  artifact = writer.finish();
}

std::shared_ptr<SemanticImage::Impl>
SemanticImage::Impl::deserialize(const std::span<const std::byte> artifact,
                                 const std::string_view input_identity) {
  ArtifactReader reader(artifact);
  if (reader.text() != format) {
    throw std::invalid_argument("DSP semantic artifact format mismatch");
  }
  if (reader.text() != Core::compiler_identity()) {
    throw std::invalid_argument("DSP semantic artifact compiler identity mismatch");
  }
  if (reader.text() != input_identity) {
    throw std::invalid_argument("DSP semantic artifact input identity mismatch");
  }
  auto result = std::make_shared<Impl>();
  result->identity = reader.text();
  const auto code = reader.bytes();
  if (result->identity.empty() || code.empty()) { ArtifactReader::fail(); }
  result->bitcode.assign(code.begin(), code.end());
  result->memory_mode =
      flag(reader) ? CompileOptions::MemoryMode::direct : CompileOptions::MemoryMode::exact;
  sequence(reader, [&] {
    result->names.push_back(reader.text());
    result->abis.push_back(reader.text());
  });
  std::set<std::uint64_t> ids;
  sequence(reader, [&] {
    auto value = variant(reader);
    if (!ids.insert(value.id).second) { ArtifactReader::fail(); }
    result->report.prepared_entries += value.instructions.size();
    result->variants.push_back(std::move(value));
  });
  sequence(reader, [&] {
    const auto vector = word(reader);
    const auto level = reader.integer<unsigned>();
    if (level > 3 || !result->interrupts.emplace(vector, level).second) { ArtifactReader::fail(); }
  });
  sequence(reader, [&] {
    const auto vector = key(reader);
    detail::ImagePlan plan;
    plan.instructions = instructions(reader);
    sequence(reader, [&] { plan.entries.push_back(word(reader)); });
    result->report.prepared_entries += plan.instructions.size();
    if (!result->vector_plans.emplace(vector, std::move(plan)).second) { ArtifactReader::fail(); }
  });
  sequence(reader, [&] { result->variant_names.push_back(reader.text()); });
  sequence(reader, [&] {
    const auto vector = key(reader);
    auto symbol = reader.text();
    if (!result->vector_symbols.emplace(vector, std::move(symbol)).second) {
      ArtifactReader::fail();
    }
  });
  reader.finish();
  if (result->variant_names.size() != result->variants.size() || result->variants.empty() ||
      result->names.empty() || result->vector_symbols.size() != result->vector_plans.size()) {
    ArtifactReader::fail();
  }
  for (std::size_t index = 0; index < result->names.size(); ++index) {
    result->exports.push_back({.name = result->names[index], .abi = result->abis[index]});
  }
  result->artifact.assign(artifact.begin(), artifact.end());
  return result;
}

std::span<const std::byte> SemanticImage::artifact() const noexcept {
  return impl_ ? std::span<const std::byte>(impl_->artifact) : std::span<const std::byte>{};
}
}
