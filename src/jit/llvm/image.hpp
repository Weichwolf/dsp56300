// SPDX-License-Identifier: GPL-3.0-only
#pragma once
#include "dsp56300/core.hpp"
#include "dsp56300/memory.hpp"

#include <cstddef>
#include <cstdint>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace dsp56300::detail {
class LlvmModule;
struct ImagePlan;

struct ImageTranslation {
  std::string_view name;
  const ImagePlan &plan;
  bool vector = false;
};

// Owned pre-O3 IR and exports; no execution owner or native compiler is retained.
struct ImageArtifact {
  std::string name;
  std::vector<std::string> entries, abis;
  std::vector<std::byte> bitcode;
};

[[nodiscard]] ImageArtifact lower_images(std::string_view name,
                                         std::span<const ImageTranslation> translations,
                                         std::span<const CallbackBinding> callbacks = {});
enum class ImageExit : std::uint8_t { owner, fault, budget, deadline, unprepared };
// Result: low 32 bits are retired instructions; high 32 bits contain ImageExit.
using ImageEntry = std::uint64_t (*)(void *, Word, std::uint32_t, std::uint64_t) noexcept;

// Link the exact semantic/binding bitcode first. Lift only explicitly declared code.
// Mutable instance state remains in the owner argument, never in exported objects.
void lift_image(LlvmModule &module, std::span<const CodeSpan> code, std::span<const Word> entries);
void lift_image(LlvmModule &module, const ImagePlan &plan, std::string_view entry = {});
void lift_vector(LlvmModule &module, const ImagePlan &plan, std::string_view entry = {});
// Remove ordinary ISA calls before the shared native O3/export/install pipeline.
void specialize_image(LlvmModule &module);
void specialize_image(LlvmModule &module, std::span<const std::string> entries);
}
