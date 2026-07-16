#!/usr/bin/env python3
"""Verify the canonical-layout ROS2 merged install against ABI-v2 producers."""

from __future__ import annotations

import argparse
import hashlib
from pathlib import Path
import re
import subprocess
import sys

PACKAGE = "robobaton_4p_ros2_demo"
NODE = "robobaton_sensors_node"
MANIFEST = "abi_manifest.sha256"
LIBRARIES = {
    "icm42688": {
        "names": ("libicm42688.so.2.0.0", "libicm42688.so.2", "libicm42688.so"),
        "soname": "libicm42688.so.2",
        "version": "ICM42688_X5_2.0",
    },
    "sc132": {
        "names": ("libsc132.so.2.0.0", "libsc132.so.2", "libsc132.so"),
        "soname": "libsc132.so.2",
        "version": "LIBSC132_2.0",
    },
}


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def readelf(path: Path, *args: str) -> str:
    return subprocess.run(
        ["readelf", *args, str(path)], text=True, stdout=subprocess.PIPE,
        stderr=subprocess.PIPE, check=True
    ).stdout


def versions(path: Path) -> set[str]:
    return set(re.findall(r"Name:\s*([^\s]+)", readelf(path, "--version-info", "--wide")))


def soname(path: Path) -> str | None:
    match = re.search(r"Library soname: \[([^\]]+)\]", readelf(path, "-d", "--wide"))
    return match.group(1) if match else None


def needed(path: Path) -> list[str]:
    return re.findall(r"Shared library: \[([^\]]+)\]", readelf(path, "-d", "--wide"))


def runpath(path: Path) -> str:
    match = re.search(r"Library runpath: \[([^\]]*)\]", readelf(path, "-d", "--wide"))
    return match.group(1) if match else ""


def verify_relocatable_bash_setup(install_dir: Path) -> None:
    setup_bash = install_dir / "setup.bash"
    if not setup_bash.is_file():
        raise AssertionError(f"missing relocatable setup entry: {setup_bash}")
    hardcoded_prefixes = re.findall(
        r'''(?m)^COLCON_CURRENT_PREFIX=["'](/[^"']+)["']$''',
        setup_bash.read_text(encoding="utf-8", errors="strict"),
    )
    if hardcoded_prefixes:
        raise AssertionError(f"setup.bash embeds absolute underlays: {hardcoded_prefixes}")
    source_result = subprocess.run(
        [
            "/bin/bash", "--noprofile", "--norc", "-c",
            'set -e\nsource "$1/setup.bash"\n'
            'printf "__AMENT_PREFIX_PATH__=%s\\n" "${AMENT_PREFIX_PATH:-}"\n'
            'printf "__COLCON_PREFIX_PATH__=%s\\n" "${COLCON_PREFIX_PATH:-}"',
            "verify-install", str(install_dir),
        ],
        text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, cwd="/",
        env={"HOME": "/tmp", "LANG": "C", "PATH": "/usr/bin:/bin"},
    )
    if source_result.returncode != 0 or source_result.stderr:
        raise AssertionError(
            f"setup.bash clean-shell failure rc={source_result.returncode}: "
            f"{source_result.stderr.strip()}"
        )
    expected = str(install_dir)
    for name in ("AMENT_PREFIX_PATH", "COLCON_PREFIX_PATH"):
        marker = f"__{name}__="
        lines = [line[len(marker):] for line in source_result.stdout.splitlines()
                 if line.startswith(marker)]
        if len(lines) != 1 or [value for value in lines[0].split(":") if value] != [expected]:
            raise AssertionError(f"setup.bash leaked prefixes into {name}: {lines}")


def artifact_names() -> list[str]:
    names = [NODE]
    for spec in LIBRARIES.values():
        names.extend(spec["names"])
    return sorted(names)


def write_manifest(runtime_dir: Path) -> None:
    lines = [f"{sha256(runtime_dir / name)}  {name}" for name in artifact_names()]
    (runtime_dir / MANIFEST).write_text("\n".join(lines) + "\n", encoding="utf-8")


def verify_manifest(runtime_dir: Path) -> None:
    manifest = runtime_dir / MANIFEST
    if not manifest.is_file() or manifest.is_symlink():
        raise AssertionError(f"missing regular manifest: {manifest}")
    expected_names = set(artifact_names())
    seen: set[str] = set()
    for line in manifest.read_text(encoding="utf-8").splitlines():
        digest, separator, name = line.partition("  ")
        if not separator or name not in expected_names or name in seen:
            raise AssertionError(f"invalid manifest line: {line!r}")
        if sha256(runtime_dir / name) != digest:
            raise AssertionError(f"manifest hash mismatch: {name}")
        seen.add(name)
    if seen != expected_names:
        raise AssertionError(f"manifest inventory mismatch: {sorted(seen)} != {sorted(expected_names)}")


def verify(install_dir: Path) -> None:
    runtime_dir = install_dir / "lib" / PACKAGE
    node = runtime_dir / NODE
    if not node.is_file() or node.is_symlink() or not (node.stat().st_mode & 0o111):
        raise AssertionError(f"missing executable node: {node}")

    project_needed = {name for name in needed(node)
                      if name.startswith(("libicm42688", "libsc132"))}
    expected_project_needed = {spec["soname"] for spec in LIBRARIES.values()}
    if project_needed != expected_project_needed:
        raise AssertionError(
            f"project DT_NEEDED mismatch: {sorted(project_needed)} != {sorted(expected_project_needed)}"
        )
    node_versions = versions(node)
    required_versions = {spec["version"] for spec in LIBRARIES.values()}
    if not required_versions.issubset(node_versions):
        raise AssertionError(f"node version needs missing: {sorted(required_versions - node_versions)}")
    old_versions = {value for value in node_versions if value.endswith("_1.0") and
                    value.startswith(("ICM42688_", "LIBSC132_"))}
    if old_versions:
        raise AssertionError(f"node retains old ABI needs: {sorted(old_versions)}")
    node_runpath = runpath(node)
    if "$ORIGIN" not in node_runpath or str(install_dir) in node_runpath:
        raise AssertionError(f"node RUNPATH is not package-relative: {node_runpath!r}")

    for library, spec in LIBRARIES.items():
        paths = [runtime_dir / name for name in spec["names"]]
        for path in paths:
            if not path.is_file():
                raise AssertionError(f"missing {library} package entry: {path}")
        hashes = {sha256(path) for path in paths}
        if len(hashes) != 1:
            raise AssertionError(f"{library} version chain bytes differ: {sorted(hashes)}")
        real = paths[0]
        if soname(real) != spec["soname"]:
            raise AssertionError(f"{library} SONAME mismatch: {soname(real)!r}")
        if spec["version"] not in versions(real):
            raise AssertionError(f"{library} version node missing: {spec['version']}")

    verify_relocatable_bash_setup(install_dir)
    verify_manifest(runtime_dir)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("install_dir", type=Path)
    parser.add_argument("--write-manifest", action="store_true")
    args = parser.parse_args()
    install_dir = args.install_dir.resolve()
    runtime_dir = install_dir / "lib" / PACKAGE
    if args.write_manifest:
        write_manifest(runtime_dir)
    verify(install_dir)
    print(f"ROS2 ABI-v2 install verified: {install_dir}")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (AssertionError, OSError, subprocess.CalledProcessError) as error:
        print(f"ROS2 ABI install verification failed: {error}", file=sys.stderr)
        raise SystemExit(1)
