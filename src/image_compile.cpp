// SPDX-License-Identifier: GPL-3.0-only
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "image.hpp"
#include "jit/image_plan.hpp"
#include "jit/llvm/image.hpp"
#include "jit/llvm/runtime.hpp"
#include "semantic_ir.hpp"

#include <algorithm>
#include <array>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <llvm/ADT/ArrayRef.h>
#include <llvm/ADT/StringExtras.h>
#include <llvm/ADT/StringRef.h>
#include <llvm/Support/SHA256.h>
#include <memory>
#include <set>
#include <span>
#include <stdexcept>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace dsp56300 {
namespace {
void hash_bytes(llvm::SHA256 &hash, const std::span<const std::byte> bytes) {
  hash.update(llvm::ArrayRef(reinterpret_cast<const std::uint8_t *>(bytes.data()), bytes.size()));
}

void hash_word(llvm::SHA256 &hash, const std::uint64_t value) {
  constexpr unsigned byte_bits = 8;
  constexpr std::uint64_t byte_mask = 0xff;
  std::array<std::uint8_t, sizeof(value)> bytes{};
  for (unsigned index = 0; index < bytes.size(); ++index) {
    bytes[index] = static_cast<std::uint8_t>((value >> (index * byte_bits)) & byte_mask);
  }
  hash.update(bytes);
}

std::string symbol(const CodeVariant &source) {
  llvm::SHA256 hash;
  const auto identity = detail::compiler_identity();
  hash.update(llvm::StringRef(identity.data(), identity.size()));
  hash_bytes(hash, detail::semantic_ir());
  hash_word(hash, source.id);
  hash_word(hash, source.code.size());
  for (const auto &span : source.code) {
    hash_word(hash, span.address);
    hash_word(hash, span.words.size());
    for (const auto word : span.words) { hash_word(hash, word); }
  }
  hash_word(hash, source.entries.size());
  for (const auto address : source.entries) { hash_word(hash, address); }
  return "dsp56300_image_" + llvm::toHex(hash.final(), true);
}

std::vector<RegionLayout> regions(const std::uint64_t id, const CompileOptions options) {
  const auto found = std::ranges::find(options.memory, id, &VariantLayout::id);
  if (found == options.memory.end()) {
    throw std::invalid_argument("DSP image has no layout for its code variant");
  }
  std::vector<RegionLayout> result(found->memory.begin(), found->memory.end());
  std::ranges::sort(result, [](const RegionLayout &left, const RegionLayout &right) {
    return std::tie(left.space, left.first) < std::tie(right.space, right.first);
  });
  std::array<std::size_t, 3> end{};
  for (const auto &region : result) {
    const auto space = static_cast<unsigned>(region.space);
    if (space >= end.size() || region.first > word_mask || region.size == 0 ||
        region.size > address_words - region.first || region.first < end[space]) {
      throw std::invalid_argument("DSP image has an invalid memory layout");
    }
    end[space] = region.first + region.size;
  }
  return result;
}

void validate(const CodeImage image, const CompileOptions options) {
  if (image.variants.empty()) { throw std::invalid_argument("DSP image requires code variants"); }
  if (!options.interrupts.empty()) {
    throw std::invalid_argument("DSP image vector entry forms are not prepared yet");
  }
  std::set<std::uint64_t> ids;
  for (const auto &source : image.variants) {
    if (!ids.insert(source.id).second) {
      throw std::invalid_argument("Duplicate DSP code variant");
    }
  }
  std::set<std::uint64_t> layouts;
  for (const auto &layout : options.memory) {
    if (!ids.contains(layout.id) || !layouts.insert(layout.id).second) {
      throw std::invalid_argument("Unknown or duplicate DSP variant layout");
    }
  }
  if (!layouts.empty() && layouts != ids) {
    throw std::invalid_argument("DSP image memory layouts do not cover every variant");
  }
}
}

NativeImage::NativeImage(std::shared_ptr<const Impl> image) noexcept : impl_(std::move(image)) {}

NativeImage::Impl::~Impl() = default;

PreparationReport NativeImage::report() const noexcept {
  return impl_ ? impl_->report
               : PreparationReport{.failure = {.kind = FaultKind::unprepared_execution}};
}

const NativeImage::Impl::Variant &NativeImage::Impl::variant(const std::uint64_t id) const {
  const auto found = std::ranges::find(variants, id, &Variant::id);
  if (found == variants.end()) { throw std::invalid_argument("Unknown DSP image variant"); }
  return *found;
}

NativeImage Core::compile(const CodeImage image, const CompileOptions options) {
  validate(image, options);
  auto result = std::make_shared<NativeImage::Impl>();
  result->compiler = std::make_unique<detail::LlvmCompiler>();
  result->variants.reserve(image.variants.size());
  for (const auto &source : image.variants) {
    const auto plan = detail::plan_image(source.code, source.entries);
    NativeImage::Impl::Variant variant;
    variant.id = source.id;
    variant.instructions = plan.instructions;
    variant.fixed_memory = !options.memory.empty();
    if (variant.fixed_memory) { variant.memory = regions(source.id, options); }
    for (const auto &span : source.code) {
      for (std::size_t index = 0; index < span.words.size(); ++index) {
        variant.words.emplace((span.address + static_cast<Word>(index)) & word_mask,
                              span.words[index]);
      }
    }
    detail::LlvmModule module(symbol(source));
    module.link_bitcode(detail::semantic_ir());
    detail::lift_image(module, source.code, source.entries);
    detail::specialize_image(module);
    variant.entry = std::bit_cast<detail::ImageEntry>(result->compiler->install(module));
    result->report.prepared_entries += variant.instructions.size();
    result->variants.push_back(std::move(variant));
  }
  result->report.native_bytes = result->compiler->native_bytes();
  return NativeImage(std::move(result));
}
}
