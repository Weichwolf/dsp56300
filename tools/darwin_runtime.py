# SPDX-License-Identifier: GPL-3.0-only
import argparse
import hashlib
import json
import platform
import shutil
import subprocess
import sys
import urllib.request
from pathlib import Path


def run(arguments, **kwargs):
    subprocess.run(
        [str(value) for value in arguments], check=True, stdout=sys.stderr, **kwargs
    )


def prepare(root, recipe, project):
    version = recipe["version"]
    for component, identity in recipe["archives"].items():
        name = f"{component}-{version}.src.tar.xz"
        archive = root / name
        candidate = archive if archive.exists() else archive.with_suffix(".part")
        if candidate != archive:
            with (
                urllib.request.urlopen(identity["source"], timeout=30) as response,
                candidate.open("wb") as output,
            ):
                shutil.copyfileobj(response, output)
        actual = hashlib.sha256(candidate.read_bytes()).hexdigest()
        if (
            actual != identity["sha256"]
            or candidate.stat().st_size != identity["bytes"]
        ):
            raise RuntimeError(f"Unexpected source identity: {candidate}")
        if candidate != archive:
            candidate.replace(archive)
        if not (root / f"{component}-{version}.src").exists():
            run(["tar", "-xf", archive, "-C", root])
    source = root / f"compiler-rt-{version}.src"
    patch = project / "cmake/compiler-rt-dyld.patch"
    applied = subprocess.run(
        ["patch", "--dry-run", "--batch", "-R", "-p1", "-i", str(patch)],
        cwd=source,
        capture_output=True,
        check=False,
    )
    if applied.returncode:
        run(["patch", "--batch", "--forward", "-p1", "-i", patch], cwd=source)
    return source


def build(root, source, llvm, version, jobs):
    binary = root / "build"
    options = {
        "CMAKE_BUILD_TYPE": "Release",
        "CMAKE_C_COMPILER": llvm / "bin/clang",
        "CMAKE_CXX_COMPILER": llvm / "bin/clang++",
        "CMAKE_C_COMPILER_TARGET": "arm64-apple-darwin",
        "CMAKE_CXX_COMPILER_TARGET": "arm64-apple-darwin",
        "CMAKE_OSX_ARCHITECTURES": "arm64",
        "CMAKE_OSX_DEPLOYMENT_TARGET": "15.0",
        "LLVM_CMAKE_DIR": llvm / "lib/cmake/llvm",
        "LLVM_COMMON_CMAKE_UTILS": root / f"cmake-{version}.src",
        "COMPILER_RT_DEFAULT_TARGET_ONLY": "ON",
        "COMPILER_RT_SANITIZERS_TO_BUILD": "asan;ubsan_standalone;tsan",
    }
    for component in (
        "BUILTINS",
        "XRAY",
        "LIBFUZZER",
        "PROFILE",
        "CTX_PROFILE",
        "MEMPROF",
        "ORC",
    ):
        options[f"COMPILER_RT_BUILD_{component}"] = "OFF"
    run(
        [
            "cmake",
            "-S",
            source,
            "-B",
            binary,
            "-G",
            "Ninja",
            *(f"-D{key}={value}" for key, value in options.items()),
        ]
    )
    targets = [f"clang_rt.{name}_osx_dynamic" for name in ("asan", "ubsan", "tsan")]
    run(["cmake", "--build", binary, "--parallel", jobs, "--target", *targets])
    return binary / "lib/darwin"


def resource_directory(root, llvm, patched):
    original = Path(
        subprocess.check_output(
            [str(llvm / "bin/clang"), "-print-resource-dir"], text=True
        ).strip()
    )
    resource = root / "resource"
    resource.mkdir(exist_ok=True)
    for name in ("include", "share"):
        target = resource / name
        if not target.exists():
            target.symlink_to(original / name, target_is_directory=True)
    libraries = resource / "lib/darwin"
    libraries.mkdir(parents=True, exist_ok=True)
    for item in (original / "lib/darwin").iterdir():
        target = libraries / item.name
        source = patched / item.name if (patched / item.name).exists() else item
        if target.is_symlink() and target.resolve() != source.resolve():
            target.unlink()
        if not target.exists():
            target.symlink_to(source)
    return resource


def main():
    parser = argparse.ArgumentParser(
        description="Build LLVM 19 sanitizer runtimes with the upstream dyld fix"
    )
    parser.add_argument("--llvm-root", type=Path)
    parser.add_argument("--root", type=Path)
    parser.add_argument("--jobs", type=int, default=4)
    args = parser.parse_args()
    if platform.system() != "Darwin" or platform.machine() != "arm64":
        parser.error("This bootstrap targets macOS ARM64")
    if args.jobs < 1:
        parser.error("--jobs must be positive")
    project = Path(__file__).resolve().parent.parent
    recipe = json.loads((project / "cmake/compiler-rt.json").read_text())
    llvm = args.llvm_root
    if llvm is None:
        brew = shutil.which("brew")
        if brew is None:
            parser.error("Specify --llvm-root for LLVM 19")
        llvm = Path(
            subprocess.check_output([brew, "--prefix", "llvm@19"], text=True).strip()
        )
    version = subprocess.check_output([str(llvm / "bin/clang"), "--version"], text=True)
    if "version 19." not in version:
        parser.error("The runtime must be built with LLVM 19")
    root = (args.root or project / "build/local/llvm19-runtime").resolve()
    root.mkdir(parents=True, exist_ok=True)
    source = prepare(root, recipe, project)
    patched = build(root, source, llvm.resolve(), recipe["version"], args.jobs)
    print(resource_directory(root, llvm.resolve(), patched))


if __name__ == "__main__":
    main()
