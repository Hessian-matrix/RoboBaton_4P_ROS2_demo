#!/usr/bin/env python3
"""Verify the canonical-layout ROS2 merged install against ABI-v2 producers."""

from __future__ import annotations

import argparse
import hashlib
import os
from pathlib import Path
import re
import subprocess
import sys

PACKAGE = "robobaton_4p_ros2_demo"
RELEASE_VERSION = (Path(__file__).resolve().parents[1] / "VERSION").read_text(encoding="utf-8").strip()
NODE = "robobaton_sensors_node"
IMU_RATE_MONITOR = "robobaton_imu_rate_monitor"
EXECUTABLES = (NODE, IMU_RATE_MONITOR)
MANIFEST = "abi_manifest.sha256"
PLUGIN_LIBRARY = "librobobaton_nv12_compressed_image_transport.so"
PLUGIN_XML = f"share/{PACKAGE}/robobaton_nv12_compressed_plugins.xml"
ENV_SCRIPT = "robobaton_ros2_env.bash"
PLUGIN_RESOURCE = f"share/ament_index/resource_index/image_transport__pluginlib__plugin/{PACKAGE}"
# 内部DDS probe属于主仓validation工具，正式install必须拒绝混入。
FORBIDDEN_INTERNAL_ARTIFACTS = {"topic_rate_probe"}
ROS_TRANSITIVE_RUNTIME_LIBS = (
    "libconsole_bridge.so", "libconsole_bridge.so.1.0",
    "libspdlog.so", "libspdlog.so.1", "libspdlog.so.1.9.2",
    "libfmt.so", "libfmt.so.8", "libfmt.so.8.1.1",
    "libtinyxml2.so", "libtinyxml2.so.9", "libtinyxml2.so.9.0.0",
    "libssl.so", "libssl.so.3", "libcrypto.so", "libcrypto.so.3",
    "libblas.so.3", "liblapack.so.3", "libgfortran.so.5",
)
ROS_RUNTIME_LINKS = {
    "libconsole_bridge.so": "libconsole_bridge.so.1.0",
    "libspdlog.so": "libspdlog.so.1", "libspdlog.so.1": "libspdlog.so.1.9.2",
    "libfmt.so": "libfmt.so.8", "libfmt.so.8": "libfmt.so.8.1.1",
    "libtinyxml2.so": "libtinyxml2.so.9", "libtinyxml2.so.9": "libtinyxml2.so.9.0.0",
    "libssl.so": "libssl.so.3", "libcrypto.so": "libcrypto.so.3",
}
LIBRARIES = {
    "icm42688": {
        "names": ("libicm42688.so.2.1.2", "libicm42688.so.2", "libicm42688.so"),
        "soname": "libicm42688.so.2",
        "versions": ("ICM42688_X5_2.0", "ICM42688_X5_2.1"),
    },
    "sc132": {
        "names": ("libsc132.so.2.0.2", "libsc132.so.2", "libsc132.so"),
        "soname": "libsc132.so.2",
        "versions": ("LIBSC132_2.0",),
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


def dynamic_symbols(path: Path) -> set[str]:
    return set(re.findall(
        r"\b(?:GLOBAL|WEAK)\b\s+\bDEFAULT\b\s+\S+\s+([A-Za-z_][A-Za-z0-9_]*)",
        readelf(path, "--dyn-syms", "--wide"),
    ))


def runpath(path: Path) -> str:
    match = re.search(r"Library runpath: \[([^\]]*)\]", readelf(path, "-d", "--wide"))
    return match.group(1) if match else ""


def normalize_generated_setup(install_dir: Path) -> None:
    """Remove only the four known Humble/colcon POSIX build-prefix fallbacks."""
    package_dir = f"share/{PACKAGE}"
    colcon_prefix = '"${COLCON_CURRENT_PREFIX:-}"'
    specifications = (
        ("setup.sh", "colcon_core/shell/template/prefix_chain.sh.em",
         "_colcon_prefix_chain_sh_COLCON_CURRENT_PREFIX=", colcon_prefix),
        ("local_setup.sh", "colcon_core/shell/template/prefix.sh.em",
         "_colcon_prefix_sh_COLCON_CURRENT_PREFIX=", colcon_prefix),
        (f"{package_dir}/package.sh", "colcon_core/shell/template/package.sh.em",
         "_colcon_package_sh_COLCON_CURRENT_PREFIX=", colcon_prefix),
        (f"{package_dir}/local_setup.sh", "ament_package/template/package_level/local_setup.sh.in",
         ': ${AMENT_CURRENT_PREFIX:=', '"${COLCON_CURRENT_PREFIX:-}"}'),
    )
    changes: list[tuple[Path, str]] = []
    for name, generator, assignment, replacement in specifications:
        path = install_dir / name
        if not path.is_file() or path.is_symlink():
            raise AssertionError(f"missing regular generated setup: {name}")
        text = path.read_text(encoding="utf-8")
        lines = text.splitlines(keepends=True)
        if not lines or lines[0] != f"# generated from {generator}\n":
            raise AssertionError(f"unexpected setup generator: {name}")
        candidates = [index for index, line in enumerate(lines) if line.startswith(assignment)]
        if len(candidates) != 1:
            raise AssertionError(f"unexpected generated prefix assignment: {name}")
        index = candidates[0]
        portable = f"{assignment}{replacement}\n"
        if lines[index] == portable:
            continue
        original = lines[index][len(assignment):].strip()
        if assignment.startswith(":"):
            if not original.endswith("}"):
                raise AssertionError(f"unexpected ament prefix fallback: {name}")
            original = original[:-1]
        if original not in (str(install_dir), f'"{install_dir}"', f"'{install_dir}'"):
            raise AssertionError(f"unexpected generated build prefix: {name}: {original!r}")
        lines[index] = portable
        if assignment.startswith(":"):
            lines.insert(index + 1,
                         'if [ -z "$AMENT_CURRENT_PREFIX" ]; then\n'
                         '  echo "Set AMENT_CURRENT_PREFIX or COLCON_CURRENT_PREFIX before sourcing local_setup.sh." 1>&2\n'
                         '  return 1\nfi\n')
        changes.append((path, "".join(lines)))
    # Validate every generator before touching any file; .dsv resources are unchanged.
    for path, text in changes:
        path.write_text(text, encoding="utf-8")


def verify_portable_paths(install_dir: Path, build_prefixes: tuple[str, ...] = ()) -> None:
    marker = install_dir / "share/ament_index/resource_index/parent_prefix_path" / PACKAGE
    if marker.exists() or marker.is_symlink():
        if marker.is_symlink() or not marker.is_file():
            raise AssertionError("parent prefix marker must be a regular file")
        prefixes = marker.read_text(encoding="utf-8").strip().split(";")
        if any(prefix not in ("", "{prefix}", "/opt/ros/humble") for prefix in prefixes):
            raise AssertionError("parent prefix marker retains a build-only ROS underlay")
    private_workspace = re.compile(rb"/root/(?:x5|4cam_candidates)/")
    generated_prefix = re.compile(
        rb'(?m)^_colcon_(?:prefix_chain|prefix|package)_sh_COLCON_CURRENT_PREFIX=["\x27]?/'
        rb'|^: \$\{AMENT_CURRENT_PREFIX:=["\x27]?/')
    forbidden = tuple(value.encode("utf-8") for value in build_prefixes if value)
    overlap = max([64, *(len(value) for value in forbidden)]) - 1
    for path in sorted(install_dir.rglob("*")):
        if path.is_symlink():
            target = os.fsencode(os.readlink(path))
            if private_workspace.search(target) or any(value in target for value in forbidden):
                raise AssertionError(f"private/build path in symlink: {path.relative_to(install_dir)}")
        elif path.is_file():
            with path.open("rb") as stream:
                previous = b""
                for block in iter(lambda: stream.read(1024 * 1024), b""):
                    data = previous + block
                    if (private_workspace.search(data) or generated_prefix.search(data)
                            or any(value in data for value in forbidden)):
                        raise AssertionError(f"private/build path in install file: {path.relative_to(install_dir)}")
                    previous = data[-overlap:]


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


def verify_runtime_env_script(install_dir: Path) -> None:
    env_script = install_dir / ENV_SCRIPT
    if not env_script.is_file() or env_script.is_symlink() or not (env_script.stat().st_mode & 0o111):
        raise AssertionError(f"missing executable runtime env script: {env_script}")
    source_result = subprocess.run(
        [
            "/bin/bash", "--noprofile", "--norc", "-c",
            'set -e\nsource "$1/robobaton_ros2_env.bash"\n'
            'printf "__PREFIX__=%s\\n" "${ROBOBATON_ROS2_INSTALL_PREFIX:-}"\n'
            'printf "__BUFFER__=%s\\n" "${RCUTILS_LOGGING_BUFFERED_STREAM:-}"\n'
            'printf "__FASTDDS__=%s\\n" "${FASTDDS_DEFAULT_PROFILES_FILE:-}"\n'
            'declare -F robobaton_ros2_list_topics >/dev/null\n',
            "verify-env", str(install_dir),
        ],
        text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, cwd="/",
        env={"HOME": "/tmp", "LANG": "C", "PATH": "/usr/bin:/bin", "ROBOBATON_ROS_UNDERLAY": ""},
    )
    if source_result.returncode != 0 or source_result.stderr:
        raise AssertionError(
            f"runtime env script source failure rc={source_result.returncode}: "
            f"{source_result.stderr.strip()}"
        )
    expected_profile = install_dir / "share" / PACKAGE / "config" / "fastdds" / "robobaton_shm.xml"
    expected_markers = {
        "__PREFIX__": str(install_dir),
        "__BUFFER__": "0",
        "__FASTDDS__": str(expected_profile),
    }
    observed = dict(line.split("=", 1) for line in source_result.stdout.splitlines() if line.startswith("__"))
    if observed != expected_markers:
        raise AssertionError(f"runtime env script exported unexpected values: {observed}")
    execute_result = subprocess.run(
        [str(env_script), "bash", "-c", 'printf "__EXEC_PREFIX__=%s\\n" "${ROBOBATON_ROS2_INSTALL_PREFIX:-}"'],
        text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, cwd="/",
        env={"HOME": "/tmp", "LANG": "C", "PATH": "/usr/bin:/bin", "ROBOBATON_ROS_UNDERLAY": ""},
    )
    if execute_result.returncode != 0 or execute_result.stderr:
        raise AssertionError(
            f"runtime env script execute failure rc={execute_result.returncode}: "
            f"{execute_result.stderr.strip()}"
        )
    if execute_result.stdout.strip() != f"__EXEC_PREFIX__={install_dir}":
        raise AssertionError(f"runtime env script command wrapper failed: {execute_result.stdout!r}")




def required_symbol_versions(spec: dict[str, object]) -> set[str]:
    return set(spec["versions"])

NODE_LIBRARY_KEYS = ("icm42688", "sc132")
# Node只需声明其实际调用符号的版本；producer仍必须提供完整ABI 2.1版本节点集合。
NODE_REQUIRED_VERSIONS = {"ICM42688_X5_2.0", "LIBSC132_2.0"}
PLUGIN_PLATFORM_SONAMES = {"libmultimedia.so.1", "libhbmem.so.1", "libalog.so.1"}


def artifact_names() -> list[str]:
    names = [*EXECUTABLES, PLUGIN_LIBRARY]
    for spec in LIBRARIES.values():
        names.extend(spec["names"])
    return sorted(names)


def verify_library_links(runtime_dir: Path) -> None:
    lexical = runtime_dir.absolute()
    if any(parent.is_symlink() for parent in (lexical, *lexical.parents)) or not runtime_dir.is_dir():
        raise AssertionError(f"runtime library directory must be real: {runtime_dir}")
    expected_links = dict(ROS_RUNTIME_LINKS)
    for spec in LIBRARIES.values():
        real, soname_name, linker = spec["names"]
        expected_links.update({soname_name: real, linker: soname_name})
    for path in runtime_dir.rglob("*"):
        if path.is_symlink():
            relative = path.relative_to(runtime_dir).as_posix()
            if relative not in expected_links or os.readlink(path) != expected_links[relative]:
                raise AssertionError(f"unexpected runtime symlink: {relative}")
    for spec in LIBRARIES.values():
        real, soname_name, linker = spec["names"]
        real_path = runtime_dir / real
        if not real_path.is_file() or real_path.is_symlink():
            raise AssertionError(f"real ELF must be a regular file: {real}")
        for name, target in ((soname_name, real), (linker, soname_name)):
            path = runtime_dir / name
            if not path.is_symlink() or os.readlink(path) != target:
                raise AssertionError(f"library symlink must be {name} -> {target}")


def write_manifest(runtime_dir: Path) -> None:
    verify_library_links(runtime_dir)
    lines = [f"{sha256(runtime_dir / name)}  {name}" for name in artifact_names()]
    (runtime_dir / MANIFEST).write_text("\n".join(lines) + "\n", encoding="utf-8")


def verify_plugin_registration(install_dir: Path) -> None:
    plugin_xml = install_dir / PLUGIN_XML
    plugin_resource = install_dir / PLUGIN_RESOURCE
    if not plugin_xml.is_file() or plugin_xml.is_symlink():
        raise AssertionError(f"missing image_transport plugin XML: {plugin_xml}")
    if not plugin_resource.is_file() or plugin_resource.is_symlink():
        raise AssertionError(f"missing image_transport plugin resource: {plugin_resource}")
    xml_text = plugin_xml.read_text(encoding="utf-8")
    if 'name="robobaton_4p_ros2_demo/compressed_pub"' not in xml_text:
        raise AssertionError("NV12 compressed plugin does not claim compressed transport")
    resource_entries = {line.strip() for line in plugin_resource.read_text(
        encoding="utf-8").splitlines() if line.strip()}
    if PLUGIN_XML not in resource_entries:
        raise AssertionError(f"image_transport plugin resource missing {PLUGIN_XML}")


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


def verify_executable(runtime_dir: Path, install_dir: Path, name: str) -> Path:
    executable = runtime_dir / name
    if not executable.is_file() or executable.is_symlink() or not (executable.stat().st_mode & 0o111):
        raise AssertionError(f"missing executable: {executable}")
    executable_runpath = runpath(executable)
    if "$ORIGIN" not in executable_runpath or str(install_dir) in executable_runpath:
        raise AssertionError(f"{name} RUNPATH is not package-relative: {executable_runpath!r}")
    return executable


def verify(install_dir: Path, build_prefixes: tuple[str, ...] = ()) -> None:
    runtime_dir = install_dir / "lib" / PACKAGE
    verify_portable_paths(install_dir, build_prefixes)
    verify_library_links(runtime_dir)
    version_file = install_dir / "share" / PACKAGE / "VERSION"
    if not version_file.is_file() or version_file.read_text(encoding="utf-8").strip() != RELEASE_VERSION:
        raise AssertionError(f"missing or invalid ROS2 release VERSION: {version_file}")
    node = verify_executable(runtime_dir, install_dir, NODE)
    verify_executable(runtime_dir, install_dir, IMU_RATE_MONITOR)
    for name in FORBIDDEN_INTERNAL_ARTIFACTS:
        if (runtime_dir / name).exists():
            raise AssertionError(f"internal validation artifact leaked into release install: {name}")

    plugin = runtime_dir / PLUGIN_LIBRARY
    if not plugin.is_file() or plugin.is_symlink():
        raise AssertionError(f"missing image_transport plugin library: {plugin}")
    if "robobaton_nv12_compressed_image_transport_get_version" not in dynamic_symbols(plugin):
        raise AssertionError("compressed image plugin does not export its release version getter")

    for name in ROS_TRANSITIVE_RUNTIME_LIBS:
        path = runtime_dir / name
        if not path.is_file():
            raise AssertionError(f"missing ROS2 transitive runtime library: {path}")

    project_needed = {name for name in needed(node)
                      if name.startswith(("libicm42688", "libsc132", "libmultimedia"))}
    expected_project_needed = {LIBRARIES[key]["soname"] for key in NODE_LIBRARY_KEYS}
    if project_needed != expected_project_needed:
        raise AssertionError(
            f"project DT_NEEDED mismatch: {sorted(project_needed)} != {sorted(expected_project_needed)}"
        )
    node_versions = versions(node)
    if not NODE_REQUIRED_VERSIONS.issubset(node_versions):
        raise AssertionError(
            f"node version needs missing: {sorted(NODE_REQUIRED_VERSIONS - node_versions)}"
        )
    old_versions = {value for value in node_versions if value.endswith("_1.0") and
                    value.startswith(("ICM42688_", "LIBSC132_"))}
    if old_versions:
        raise AssertionError(f"node retains old ABI needs: {sorted(old_versions)}")
    # 可执行文件的$ORIGIN RUNPATH已由verify_executable统一校验。

    plugin_needed = {name for name in needed(plugin)
                     if name.startswith(("libmultimedia", "libhbmem", "libalog"))}
    if plugin_needed != PLUGIN_PLATFORM_SONAMES:
        raise AssertionError(
            f"plugin DT_NEEDED mismatch: {sorted(plugin_needed)} != {sorted(PLUGIN_PLATFORM_SONAMES)}"
        )
    plugin_runpath = runpath(plugin)
    if "$ORIGIN" not in plugin_runpath or str(install_dir) in plugin_runpath:
        raise AssertionError(f"plugin RUNPATH is not package-relative: {plugin_runpath!r}")

    for library, spec in LIBRARIES.items():
        real = runtime_dir / spec["names"][0]
        if soname(real) != spec["soname"]:
            raise AssertionError(f"{library} SONAME mismatch: {soname(real)!r}")
        missing_versions = required_symbol_versions(spec) - versions(real)
        if missing_versions:
            raise AssertionError(f"{library} version nodes missing: {sorted(missing_versions)}")

    verify_plugin_registration(install_dir)
    verify_runtime_env_script(install_dir)
    verify_relocatable_bash_setup(install_dir)
    verify_manifest(runtime_dir)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("install_dir", type=Path)
    parser.add_argument("--write-manifest", action="store_true")
    parser.add_argument("--normalize-generated-setup", action="store_true",
                        help="Remove known generated POSIX build-prefix fallbacks before verification")
    parser.add_argument("--build-prefix", action="append", default=[],
                        help="Additional build-only prefix forbidden anywhere in the install")
    args = parser.parse_args()
    install_dir = args.install_dir.absolute()
    runtime_dir = install_dir / "lib" / PACKAGE
    if args.normalize_generated_setup:
        normalize_generated_setup(install_dir)
    if args.write_manifest:
        verify_portable_paths(install_dir, tuple(args.build_prefix))
        write_manifest(runtime_dir)
    verify(install_dir, tuple(args.build_prefix))
    print(f"ROS2 ABI-v2 install verified: {install_dir}")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (AssertionError, OSError, subprocess.CalledProcessError) as error:
        print(f"ROS2 ABI install verification failed: {error}", file=sys.stderr)
        raise SystemExit(1)
