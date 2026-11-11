// SPDX-License-Identifier: GPL-3.0-only
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"
#include "image.hpp"
#include "jit/image_plan.hpp"
#include "jit/llvm/bindings.hpp"
#include "jit/llvm/image.hpp"
#include "jit/llvm/runtime.hpp"
#include "jit/prefixes.hpp"
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
#include <map>
#include <memory>
#include <optional>
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

void hash_variant(llvm::SHA256 &hash, const CodeVariant &source) {
  hash_word(hash, source.id);
  hash_word(hash, source.code.size());
  for (const auto &span : source.code) {
    hash_word(hash, span.address);
    hash_word(hash, span.words.size());
    for (const auto word : span.words) { hash_word(hash, word); }
  }
  hash_word(hash, source.entries.size());
  for (const auto address : source.entries) { hash_word(hash, address); }
}

std::string symbol(const CodeImage image, const CompileOptions options) {
  llvm::SHA256 hash;
  const auto identity = detail::compiler_identity();
  hash.update(llvm::StringRef(identity.data(), identity.size()));
  hash_bytes(hash, detail::semantic_ir());
  hash_word(hash, image.variants.size());
  for (const auto &source : image.variants) { hash_variant(hash, source); }
  hash_word(hash, options.memory.size());
  for (const auto &layout : options.memory) {
    hash_word(hash, layout.id);
    hash_word(hash, layout.memory.size());
    for (const auto &region : layout.memory) {
      hash_word(hash, static_cast<unsigned>(region.space));
      hash_word(hash, region.first);
      hash_word(hash, region.size);
      hash_word(hash, static_cast<std::uint64_t>(region.writable));
      hash_word(hash, static_cast<std::uint64_t>(region.tracked));
      hash_word(hash, static_cast<std::uint64_t>(region.aliases_program));
    }
  }
  hash_word(hash, options.interrupts.size());
  for (const auto request : options.interrupts) {
    hash_word(hash, request.vector);
    hash_word(hash, request.level);
  }
  hash_word(hash, options.transitions.size());
  for (const auto transfer : options.transitions) {
    hash_word(hash, transfer.from);
    hash_word(hash, transfer.to);
    hash_word(hash, transfer.entry);
  }
  hash_word(hash, options.callbacks.size());
  for (const auto &binding : options.callbacks) {
    hash_word(hash, static_cast<unsigned>(binding.role));
    hash_word(hash, binding.symbol.size());
    hash.update(llvm::StringRef(binding.symbol));
  }
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

std::optional<std::span<const RegionLayout>>
memory_view(const detail::ImageContracts::Variant &variant) {
  return variant.fixed_memory ? std::optional<std::span<const RegionLayout>>(variant.memory)
                              : std::nullopt;
}

void validate(const CodeImage image, const CompileOptions options) {
  if (image.variants.empty()) { throw std::invalid_argument("DSP image requires code variants"); }
  std::set<std::pair<Word, unsigned>> interrupts;
  for (const auto request : options.interrupts) {
    if (request.vector > word_mask || (request.vector & 1U) != 0 || request.level > 3 ||
        !interrupts.emplace(request.vector, request.level).second) {
      throw std::invalid_argument("DSP image has an invalid or duplicate interrupt contract");
    }
  }
  std::set<std::uint64_t> ids;
  for (const auto &source : image.variants) {
    if (!ids.insert(source.id).second) {
      throw std::invalid_argument("Duplicate DSP code variant");
    }
  }
  std::set<std::pair<std::uint64_t, Word>> transfers;
  for (const auto transfer : options.transitions) {
    if (!ids.contains(transfer.from) || !ids.contains(transfer.to) ||
        transfer.from == transfer.to || transfer.entry > word_mask ||
        !transfers.emplace(transfer.from, transfer.entry).second) {
      throw std::invalid_argument("Invalid or ambiguous DSP code transition");
    }
    const auto target = std::ranges::find(image.variants, transfer.to, &CodeVariant::id);
    if (std::ranges::find(target->entries, transfer.entry) == target->entries.end()) {
      throw std::invalid_argument("DSP code transition target is not a declared destination entry");
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

PreparationReport NativeImage::report() const noexcept {
  return impl_ ? impl_->report
               : PreparationReport{.failure = {.kind = FaultKind::unprepared_execution}};
}

const NativeImage::Impl::Variant &NativeImage::Impl::variant(const std::uint64_t id) const {
  const auto found = std::ranges::find(variants, id, &Variant::id);
  if (found == variants.end()) { throw std::invalid_argument("Unknown DSP image variant"); }
  return *found;
}

detail::ImageEntry NativeImage::Impl::vector_entry(const Word address,
                                                   const std::array<Word, 2> words) const {
  const auto found = vectors.find(std::tuple(address, words[0], words[1]));
  return found == vectors.end() ? nullptr : found->second;
}

std::map<std::tuple<Word, Word, Word>, detail::ImagePlan>
detail::ImageContracts::plan_vectors(const std::vector<detail::ImagePlan> &plans) const {
  std::map<std::tuple<Word, Word, Word>, detail::ImagePlan> result;
  std::set<Word> available;
  for (std::size_t index = 0; index < variants.size(); ++index) {
    const auto &source = variants[index];
    std::vector<Word> external;
    external.reserve(source.transitions.size());
    for (const auto &transfer : source.transitions) { external.push_back(transfer.first); }
    for (const auto &[address, level] : interrupts) {
      static_cast<void>(level);
      const auto first = source.words.find(address);
      const auto second = source.words.find(address + 1);
      if (first == source.words.end() && second == source.words.end()) { continue; }
      if (first == source.words.end() || second == source.words.end()) {
        throw std::invalid_argument("DSP vector has incomplete source at P:$" +
                                    llvm::utohexstr(address, true, 6));
      }
      // Validate the target closure in every foreground variant, even for shared vectors.
      auto plan =
          detail::plan_vector(address, {first->second, second->second}, plans[index], external);
      result.try_emplace(std::tuple(address, first->second, second->second), std::move(plan));
      available.insert(address);
    }
  }
  for (const auto &[address, level] : interrupts) {
    static_cast<void>(level);
    if (!available.contains(address)) {
      throw std::invalid_argument("DSP vector has no prepared source at P:$" +
                                  llvm::utohexstr(address, true, 6));
    }
  }
  return result;
}

SemanticImage Core::lower(const CodeImage image, const CompileOptions options) {
  validate(image, options);
  auto result = std::make_shared<SemanticImage::Impl>();
  for (const auto request : options.interrupts) {
    result->interrupts.emplace(request.vector, request.level);
  }
  result->variants.reserve(image.variants.size());
  std::vector<detail::ImagePlan> plans;
  plans.reserve(image.variants.size());
  for (const auto &source : image.variants) {
    detail::ImageContracts::Variant variant;
    variant.id = source.id;
    std::vector<Word> external;
    external.reserve(options.transitions.size());
    for (const auto transfer : options.transitions) {
      if (transfer.from != source.id) { continue; }
      variant.transitions.emplace(transfer.entry, transfer.to);
      external.push_back(transfer.entry);
    }
    const auto plan = detail::plan_image(source, external);
    for (const auto entry : external) {
      if (plan.find(entry) != nullptr) {
        throw std::invalid_argument(
            "DSP transition entry is already resident in its source variant");
      }
    }
    variant.instructions = plan.instructions;
    variant.fixed_memory = !options.memory.empty();
    if (variant.fixed_memory) { variant.memory = regions(source.id, options); }
    variant.prefixes = detail::foreground_prefixes(plan, variant.memory);
    for (const auto &span : source.code) {
      for (std::size_t index = 0; index < span.words.size(); ++index) {
        variant.words.emplace((span.address + static_cast<Word>(index)) & word_mask,
                              span.words[index]);
      }
    }
    plans.push_back(plan);
    result->variants.push_back(std::move(variant));
  }
  result->vector_plans = result->plan_vectors(plans);
  const auto &vector_plans = result->vector_plans;
  const auto image_name = symbol(image, options);
  std::vector<std::string> names;
  names.reserve(image.variants.size());
  std::map<std::tuple<Word, Word, Word>, std::string> vector_symbols;
  std::vector<detail::ImageTranslation> translations;
  translations.reserve(plans.size() + vector_plans.size());
  for (std::size_t index = 0; index < image.variants.size(); ++index) {
    const auto &variant = result->variants[index];
    names.push_back(index == 0 ? image_name
                               : image_name + ".variant." + llvm::utohexstr(variant.id, true));
    translations.push_back(
        {.name = names.back(), .plan = plans[index], .memory = memory_view(variant)});
    result->report.prepared_entries += variant.instructions.size();
  }
  for (const auto &[key, plan] : vector_plans) {
    const auto &[address, first, second] = key;
    const auto name = image_name + ".vector." + llvm::utohexstr(address, true) + "." +
                      llvm::utohexstr(first, true) + "." + llvm::utohexstr(second, true);
    const auto symbol = vector_symbols.emplace(key, name).first;
    translations.push_back(
        {.name = symbol->second, .plan = plan, .vector = true, .memory = std::nullopt});
    result->report.prepared_entries += plan.instructions.size();
  }
  auto artifact = detail::lower_images(image_name, translations, options.callbacks);
  result->bitcode = std::move(artifact.bitcode);
  result->names = std::move(artifact.entries);
  result->abis = std::move(artifact.abis);
  result->variant_names = std::move(names);
  result->vector_symbols = std::move(vector_symbols);
  result->exports.reserve(result->names.size());
  for (std::size_t index = 0; index < result->names.size(); ++index) {
    result->exports.push_back({.name = result->names[index], .abi = result->abis[index]});
  }
  llvm::SHA256 hash;
  hash.update(llvm::StringRef(image_name));
  hash_bytes(hash, result->bitcode);
  result->identity = "dsp56300_component_" + llvm::toHex(hash.final(), true);
  return SemanticImage(std::move(result));
}

SemanticImage::SemanticImage(std::shared_ptr<const Impl> image) noexcept
    : impl_(std::move(image)) {}

std::span<const std::byte> SemanticImage::bitcode() const noexcept {
  return impl_ ? std::span<const std::byte>(impl_->bitcode) : std::span<const std::byte>{};
}

std::span<const SemanticExport> SemanticImage::exports() const noexcept {
  return impl_ ? std::span<const SemanticExport>(impl_->exports)
               : std::span<const SemanticExport>{};
}

std::string_view SemanticImage::identity() const noexcept {
  return impl_ ? std::string_view(impl_->identity) : std::string_view{};
}

NativeImage Core::bind(const SemanticImage &image, const NativeSymbols &symbols) {
  if (!image.impl_ || !symbols.owner || image.identity() != symbols.identity ||
      symbols.symbols.size() != image.exports().size()) {
    throw std::invalid_argument("DSP native binding requires its exact component and code owner");
  }
  std::map<std::string_view, const void *> addresses;
  for (const auto &symbol : symbols.symbols) {
    if (symbol.name.empty() || symbol.name.find('\0') != std::string_view::npos ||
        symbol.address == nullptr || !addresses.emplace(symbol.name, symbol.address).second) {
      throw std::invalid_argument("DSP native binding has an invalid or duplicate export");
    }
  }
  for (const auto &entry : image.exports()) {
    if (!addresses.contains(entry.name)) {
      throw std::invalid_argument("DSP native binding lacks export " + std::string(entry.name));
    }
  }
  auto result = std::make_shared<NativeImage::Impl>();
  static_cast<detail::ImageContracts &>(*result) = *image.impl_;
  result->owner = symbols.owner;
  result->report.native_bytes = symbols.native_bytes;
  for (std::size_t index = 0; index < result->variants.size(); ++index) {
    result->variants[index].entry =
        std::bit_cast<detail::ImageEntry>(addresses.at(image.impl_->variant_names[index]));
  }
  for (const auto &[key, name] : image.impl_->vector_symbols) {
    result->vectors.emplace(key, std::bit_cast<detail::ImageEntry>(addresses.at(name)));
  }
  return NativeImage(std::move(result));
}

NativeImage Core::compile(const CodeImage image, const CompileOptions options) {
  const auto semantic = lower(image, options);
  detail::LlvmModule module(std::string(semantic.exports().front().name));
  module.link_bitcode(semantic.bitcode());
  detail::link_callbacks(module, options.bindings, options.callbacks);
  auto compiler = std::make_shared<detail::LlvmCompiler>();
  std::vector<NativeSymbol> addresses;
  addresses.reserve(semantic.exports().size());
  const auto *const first = compiler->install(module);
  for (const auto &entry : semantic.exports()) {
    addresses.push_back(
        {.name = entry.name,
         .address = addresses.empty() ? first : compiler->resolve(std::string(entry.name))});
  }
  return bind(semantic,
              {.owner = compiler,
               .symbols = addresses,
               .identity = semantic.identity(),
               .native_bytes = compiler->native_bytes()});
}
}
