#!/usr/bin/env python3
"""Fail closed when the canonical source or installed node retains a forbidden protocol surface."""

from __future__ import annotations

import argparse
from pathlib import Path
import subprocess
import sys

PACKAGE = "robobaton_4p_ros2_demo"
NODE = "robobaton_sensors_node"
# 2026-07-16：拆分构造禁用 token，确保扫描器自身不会污染被扫描源码面。
TOKEN = b"r" + b"t" + b"s" + b"p"
EXCLUDED_DIR_NAMES = {".git", "build", "install", "log"}


def count_token(data: bytes) -> int:
    return data.lower().count(TOKEN)


def source_findings(root: Path) -> list[str]:
    findings: list[str] = []
    pycache_dirs: set[Path] = set()
    for path in sorted(root.rglob("*")):
        relative = path.relative_to(root)
        # 2026-07-16：只排除版本库元数据与构建产物；canonical 源码和生成缓存均须纳入门禁。
        if any(part in EXCLUDED_DIR_NAMES or part.endswith(".ros2_build")
               for part in relative.parts):
            continue
        if "__pycache__" in relative.parts:
            # 记录缓存目录本身，防止缓存内容恰好不含禁用 token 时漏报生成残留。
            pycache_dirs.add(next(
                root.joinpath(*relative.parts[:index + 1])
                for index, part in enumerate(relative.parts) if part == "__pycache__"
            ))
        path_occurrences = count_token(relative.as_posix().encode("utf-8", "surrogateescape"))
        if path_occurrences:
            findings.append(
                f"SOURCE_HIT path={relative} path_occurrences={path_occurrences} content_occurrences=0"
            )
        if not path.is_file() or path.is_symlink():
            continue
        content_occurrences = count_token(path.read_bytes())
        if content_occurrences:
            findings.append(
                f"SOURCE_HIT path={relative} path_occurrences=0 "
                f"content_occurrences={content_occurrences}"
            )
    for path in sorted(pycache_dirs):
        findings.append(f"GENERATED_CACHE_PRESENT path={path.relative_to(root)}")
    return findings


def readelf_lines(node: Path, *arguments: str) -> list[str]:
    result = subprocess.run(
        ["readelf", *arguments, str(node)],
        check=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    return [line.decode("utf-8", "replace") for line in result.stdout.splitlines()
            if TOKEN in line.lower()]


def install_findings(install_dir: Path) -> list[str]:
    findings: list[str] = []
    runtime_dir = install_dir / "lib" / PACKAGE
    node = runtime_dir / NODE
    if not node.is_file():
        return [f"MISSING_NODE path={node}"]
    raw_occurrences = count_token(node.read_bytes())
    symbol_lines = readelf_lines(node, "-Ws", "--wide")
    needed_lines = readelf_lines(node, "-d", "--wide")
    version_lines = readelf_lines(node, "--version-info", "--wide")
    inventory_hits = [entry.name for entry in sorted(runtime_dir.iterdir())
                      if count_token(entry.name.encode("utf-8", "surrogateescape"))]
    print(f"NODE_RAW_OCCURRENCES={raw_occurrences}")
    print(f"NODE_MATCHING_SYMBOL_LINES={len(symbol_lines)}")
    print(f"NODE_MATCHING_NEEDED_LINES={len(needed_lines)}")
    print(f"NODE_MATCHING_VERSION_LINES={len(version_lines)}")
    print(f"RUNTIME_MATCHING_PATHS={len(inventory_hits)}")
    for category, lines in (
        ("SYMBOL", symbol_lines), ("NEEDED", needed_lines), ("VERSION", version_lines)
    ):
        for line in lines:
            print(f"{category}_HIT {line}")
    for name in inventory_hits:
        print(f"RUNTIME_PATH_HIT {name}")
    if raw_occurrences:
        findings.append(f"NODE_RAW_TOKEN_COUNT={raw_occurrences}")
    if symbol_lines:
        findings.append(f"NODE_SYMBOL_HIT_COUNT={len(symbol_lines)}")
    if needed_lines:
        findings.append(f"NODE_NEEDED_HIT_COUNT={len(needed_lines)}")
    if version_lines:
        findings.append(f"NODE_VERSION_HIT_COUNT={len(version_lines)}")
    if inventory_hits:
        findings.append(f"RUNTIME_PATH_HIT_COUNT={len(inventory_hits)}")
    return findings


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source-root", type=Path)
    parser.add_argument("--install-dir", type=Path)
    args = parser.parse_args()
    if args.source_root is None and args.install_dir is None:
        parser.error("at least one scan root is required")

    findings: list[str] = []
    if args.source_root is not None:
        source_root = args.source_root.resolve()
        findings.extend(source_findings(source_root))
    if args.install_dir is not None:
        findings.extend(install_findings(args.install_dir.resolve()))

    for finding in findings:
        print(finding)
    result = "PASS" if not findings else "FAIL"
    print(f"PROTOCOL_FREE_SCAN={result} findings={len(findings)}")
    return 0 if not findings else 1


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, subprocess.CalledProcessError) as error:
        print(f"protocol-free scan error: {error}", file=sys.stderr)
        raise SystemExit(2)
