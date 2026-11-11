# SPDX-License-Identifier: GPL-3.0-only
"""Bundle the core's normal semantic bitcode and an exact compiler-source identity."""
import hashlib
import json
import pathlib
import subprocess
import sys


def main():
    bitcode, source_list, root, output, manifest = map(pathlib.Path, sys.argv[1:6])
    source_hash = hashlib.sha256()
    sources = {}
    for name in sorted(source_list.read_text().splitlines()):
        path = pathlib.Path(name)
        relative = path.relative_to(root).as_posix()
        data = path.read_bytes()
        source_hash.update(relative.encode() + b"\0" + data + b"\0")
        sources[relative] = {"bytes": len(data), "sha256": hashlib.sha256(data).hexdigest()}
    data = bitcode.read_bytes()
    identity = source_hash.hexdigest()
    bitcode_hash = hashlib.sha256(data).hexdigest()
    # The assembler includes the immutable blob without constructing millions of
    # C++ literal expressions. Its digest keeps compiler caches content-aware.
    include = '.incbin ' + json.dumps(str(bitcode.resolve())) + '\n'
    name = 'dsp56300_semantic_data'
    def assembly(apple):
        symbol = '_' + name if apple else name
        section = '__TEXT,__const' if apple else '.rodata'
        visibility = '.private_extern ' if apple else '.hidden '
        lines = [f'.pushsection {section}\n', '.p2align 4\n',
                 f'.globl {symbol}\n', visibility + symbol + '\n',
                 symbol + ':\n', include]
        if not apple:
            lines += [f'.type {symbol},@object\n', f'.size {symbol},{len(data)}\n']
        lines += ['.popsection\n']
        return 'asm(' + '\n'.join(json.dumps(line) for line in lines) + ');'
    text = '\n'.join([
        '// SPDX-License-Identifier: GPL-3.0-only',
        f'// Semantic bitcode SHA-256: {bitcode_hash}',
        '#include "semantic_ir.hpp"',
        '#include <cstddef>', '#include <span>', '#include <string_view>',
        'namespace dsp56300::detail {', 'namespace {',
        f'constexpr std::string_view identity = "{identity}";', '}',
        f'extern "C" const unsigned char {name}[{len(data)}];',
        '#if defined(__APPLE__)', assembly(True), '#else', assembly(False), '#endif',
        'std::span<const std::byte> semantic_ir() noexcept {',
        f'return std::as_bytes(std::span({name}));', '}',
        'std::string_view compiler_identity() noexcept { return identity; }', '}', ''])
    text = subprocess.check_output([sys.argv[6]], input=text, text=True)
    if not output.exists() or output.read_text() != text:
        output.write_text(text)
    manifest.write_text(json.dumps({
        "compiler_identity": identity, "sources": sources,
        "bitcode": {"bytes": len(data), "sha256": bitcode_hash},
    }, indent=2) + '\n')


if __name__ == "__main__":
    main()
