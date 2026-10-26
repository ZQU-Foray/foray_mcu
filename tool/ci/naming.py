#!/usr/bin/env python3
"""按根目录 .clang-tidy 检查或修正本仓 C++ 代码的命名（ROS 2 风格）。

翻译单元取自给定的编译数据库（可以给多个，如固件与宿主测试各一个），
只处理 C++ 源文件，文件范围与 format.py 相同（排除生成区、厂商代码与 vendor 头）。
头文件由 .clang-tidy 的 HeaderFilterRegex 纳入；C 源文件与 C ABI 头不改名（P3a 重写）。
没有被任何翻译单元包含的头文件（孤立头）借用路径最近的 C++ 翻译单元的编译参数单独检查，
否则其中的名字永远不会被检查到。
被 #if 关掉的代码对 clang-tidy 不可见：编译数据库要覆盖各个配置（如固件 Debug 打开诊断代码）。

用法：
    naming.py -p build/legacy-Debug -p build/host-tests --check
    naming.py -p ... --fix [--report 改名表.tsv]
    naming.py -p ... --list               # 列出将要检查的翻译单元

--fix 汇总各翻译单元导出的修正，去重并检查冲突后一次写回；
同一个头文件会被多个翻译单元重复报告，不能直接用 clang-tidy --fix 并行改写。

交叉编译器（如 arm-none-eabi-g++）的目标三元组与系统头目录向编译器本身查询，
再传给 clang-tidy；编译器内部头（lib/gcc/<三元组>/<版本>/include）换成 clang 内置头。
clang-tidy 取自 --clang-tidy、环境变量 CLANG_TIDY 或 PATH，版本须为 18。依赖 PyYAML。
"""

from __future__ import annotations

import argparse
import collections
import concurrent.futures
import itertools
import json
import os
import re
import shlex
import subprocess
import sys
import tempfile
from dataclasses import dataclass
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from format import EXCLUDE, git

CHECK = "readability-identifier-naming"
GCC_INTERNAL = re.compile(
    r"[/\\]lib[/\\]gcc[/\\][^/\\]+[/\\][^/\\]+[/\\]include(-fixed)?$"
)
SEARCH_BEGIN = "#include <...> search starts here:"
SEARCH_END = "End of search list."
MESSAGE = re.compile(r"invalid case style for (?P<kind>.+) '(?P<name>[^']+)'$")
INCLUDE_TRACE = re.compile(r"^\.+ (.+)$")  # -H 输出：每层包含一个点


@dataclass(frozen=True)
class Unit:
    database: Path
    directory: Path
    source: Path  # 编译参数所属的源文件
    arguments: tuple[str, ...]
    file: Path  # 实际检查的文件：普通单元即 source，孤立头为该头文件

    @property
    def compiler(self) -> str:
        return self.arguments[0]

    @property
    def flags(self) -> tuple[str, ...]:
        return tuple(
            arg for arg in self.arguments[1:] if re.match(r"-m|-std=|--target", arg)
        )

    @property
    def header(self) -> bool:
        return self.file != self.source


@dataclass(frozen=True)
class Edit:
    path: str
    offset: int
    length: int
    text: str


def split_command(command: str) -> list[str]:
    if os.name != "nt":
        return shlex.split(command)
    # Windows 下 CMake 写出的命令只用双引号包住含空格的参数，反斜杠是路径分隔符。
    return [
        quoted if quoted else bare
        for quoted, bare in re.findall(r'"([^"]*)"|(\S+)', command)
    ]


def load_units(databases: list[Path], scope: set[str], root: Path) -> list[Unit]:
    units: dict[tuple[str, str, tuple[str, ...]], Unit] = {}
    for database in databases:
        entries = json.loads((database / "compile_commands.json").read_text("utf-8"))
        for entry in entries:
            path = Path(entry["directory"], entry["file"]).resolve()
            try:
                relative = path.relative_to(root).as_posix()
            except ValueError:
                continue
            if relative not in scope or not relative.endswith(".cpp"):
                continue
            arguments = entry.get("arguments") or split_command(entry["command"])
            directory = Path(entry["directory"])
            unit = Unit(database, directory, path, tuple(arguments), path)
            # 同一数据库里同一文件的多条命令由 clang-tidy -p 逐条处理，这里只留一个单元。
            units.setdefault((str(database), relative, unit.flags), unit)
    return sorted(units.values(), key=lambda unit: (str(unit.file), str(unit.database)))


def header_unit(header: Path, units: list[Unit]) -> Unit:
    """孤立头：借用路径公共前缀最长的翻译单元的编译参数。"""

    def shared(unit: Unit) -> int:
        count = 0
        for left, right in zip(header.parts, unit.source.parts):
            if left != right:
                break
            count += 1
        return count

    donor = max(units, key=shared)  # 并列时取排序靠前者
    return Unit(donor.database, donor.directory, donor.source, donor.arguments, header)


def donor_arguments(unit: Unit) -> list[str]:
    """去掉编译器、源文件、输出与依赖文件参数，按 C++ 头文件解析。"""
    result: list[str] = []
    skip = False
    for arg in unit.arguments[1:]:
        if skip:
            skip = False
            continue
        if arg in ("-o", "-MT", "-MF", "-MQ"):
            skip = True
            continue
        if arg in ("-c", "-MD", "-MMD"):
            continue
        if Path(unit.directory, arg).resolve() == unit.source:
            continue
        result.append(arg)
    return [*result, "-xc++-header"]


def query_compiler(compiler: str, flags: tuple[str, ...]) -> list[str]:
    """返回传给 clang-tidy 的 --extra-arg：目标三元组与系统头目录。"""
    triple = subprocess.run(
        [compiler, *flags, "-dumpmachine"],
        check=True,
        stdout=subprocess.PIPE,
        text=True,
    ).stdout.strip()
    verbose = subprocess.run(
        [compiler, *flags, "-xc++", "-E", "-v", "-"],
        check=True,
        input="",
        stdout=subprocess.DEVNULL,
        stderr=subprocess.PIPE,
        text=True,
    ).stderr.splitlines()
    begin = verbose.index(SEARCH_BEGIN)
    end = verbose.index(SEARCH_END)
    directories = [
        os.path.normpath(line.strip())
        for line in verbose[begin + 1 : end]
        if not GCC_INTERNAL.search(os.path.normpath(line.strip()))
    ]
    # -w：只关心命名检查，编译告警（含 -Werror）一律关闭，避免 clang 与 gcc 告警差异变成错误。
    return [f"--target={triple}", "-w", *(f"-isystem{d}" for d in directories)]


def run_unit(
    clang_tidy: str, unit: Unit, extra: list[str], export: Path
) -> tuple[Unit, int, str, set[Path]]:
    """运行 clang-tidy；普通单元附带 -H，返回实际包含的头文件，用于找出孤立头。"""
    command = [
        clang_tidy,
        "--quiet",
        f"--export-fixes={export}",
        *(f"--extra-arg={arg}" for arg in extra),
    ]
    if unit.header:
        command += [str(unit.file), "--", *donor_arguments(unit)]
    else:
        command += [f"-p={unit.database}", "--extra-arg=-H", str(unit.file)]
    result = subprocess.run(
        command,
        check=False,
        cwd=unit.directory,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        encoding="utf-8",
        errors="replace",
    )
    lines = result.stdout.splitlines()
    includes = {
        Path(unit.directory, match[1]).resolve()
        for match in map(INCLUDE_TRACE.match, lines)
        if match
    }
    output = "\n".join(line for line in lines if not INCLUDE_TRACE.match(line))
    return unit, result.returncode, output, includes


def collect(export: Path, root: Path, scope: set[str]) -> list[dict]:
    import yaml

    if not export.exists():
        return []
    # BaseLoader 把所有标量都读成字符串：新名字可能是 ON、NO 之类，
    # 按 YAML 1.1 会被解析成布尔值。
    data = yaml.load(export.read_text("utf-8"), Loader=yaml.BaseLoader) or {}
    diagnostics = []
    for diagnostic in data.get("Diagnostics", []):
        if diagnostic.get("DiagnosticName") != CHECK:
            continue
        message = diagnostic["DiagnosticMessage"]
        match = MESSAGE.search(message["Message"])
        edits = []
        for replacement in message.get("Replacements", []):
            path = Path(replacement["FilePath"]).resolve()
            edits.append(
                Edit(
                    (
                        path.relative_to(root).as_posix()
                        if path.is_relative_to(root)
                        else str(path)
                    ),
                    int(replacement["Offset"]),
                    int(replacement["Length"]),
                    replacement["ReplacementText"],
                )
            )
        location = Path(message["FilePath"]).resolve()
        try:
            location = location.relative_to(root).as_posix()
        except ValueError:
            continue
        # 仓库若位于名为 src/test 等的目录下，HeaderFilterRegex 会误纳入厂商头；以范围为准。
        if location not in scope:
            continue
        diagnostics.append(
            {
                "file": location,
                "offset": int(message["FileOffset"]),
                "kind": match["kind"] if match else "?",
                "name": match["name"] if match else message["Message"],
                "new": edits[0].text if edits else "",
                "edits": edits,
            }
        )
    return diagnostics


def line_column(root: Path, path: str, offset: int) -> tuple[int, int]:
    data = (root / path).read_bytes()[:offset]
    line = data.count(b"\n") + 1
    return line, offset - (data.rfind(b"\n") + 1) + 1


def apply_edits(root: Path, edits: set[Edit], scope: set[str]) -> int:
    """先校验并算出全部文件的新内容，全部成功后再写盘，避免中途失败留下半成品。"""
    by_file: dict[str, list[Edit]] = collections.defaultdict(list)
    for edit in edits:
        by_file[edit.path].append(edit)
    results: dict[str, bytes] = {}
    for path, file_edits in sorted(by_file.items()):
        if path not in scope:
            sys.exit(f"修正落在范围外的文件：{path}")
        file_edits.sort(key=lambda edit: (edit.offset, edit.length))
        for previous, current in itertools.pairwise(file_edits):
            if previous.offset + previous.length > current.offset:
                sys.exit(f"修正冲突：{path} 偏移 {previous.offset} 与 {current.offset}")
        data = bytearray((root / path).read_bytes())
        for edit in reversed(file_edits):
            data[edit.offset : edit.offset + edit.length] = edit.text.encode("utf-8")
        results[path] = bytes(data)
    for path, data in results.items():
        (root / path).write_bytes(data)
    return len(results)


def load_header_filter(root: Path) -> re.Pattern[str]:
    import yaml

    config = yaml.load(
        (root / ".clang-tidy").read_text("utf-8"), Loader=yaml.BaseLoader
    )
    return re.compile(config["HeaderFilterRegex"])


def check_version(clang_tidy: str) -> None:
    output = subprocess.run(
        [clang_tidy, "--version"], check=True, stdout=subprocess.PIPE, text=True
    ).stdout
    print(output.strip().splitlines()[-2].strip())
    if "version 18." not in output:
        print("需要 clang-tidy 18（与 CI 一致）", file=sys.stderr)
        sys.exit(2)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--check", action="store_true")
    mode.add_argument("--fix", action="store_true")
    mode.add_argument("--list", action="store_true")
    parser.add_argument(
        "-p",
        dest="databases",
        action="append",
        required=True,
        type=Path,
        help="含 compile_commands.json 的构建目录，可重复",
    )
    parser.add_argument("--report", type=Path, help="把改名表写入该文件（TSV）")
    parser.add_argument("-j", "--jobs", type=int, default=os.cpu_count() or 1)
    parser.add_argument(
        "--clang-tidy", default=os.environ.get("CLANG_TIDY", "clang-tidy")
    )
    args = parser.parse_args()

    root = Path(git("rev-parse", "--show-toplevel")[0]).resolve()
    databases = [path.resolve() for path in args.databases]
    os.chdir(root)
    scope = {
        path
        for path in git("ls-files", "--", "*.c", "*.h", "*.cpp", "*.hpp")
        if not EXCLUDE.search(path)
    }
    units = load_units(databases, scope, root)
    if args.list:
        for unit in units:
            print(f"{unit.file.relative_to(root).as_posix()}\t{unit.database}")
        return 0
    if not units:
        print("编译数据库中没有范围内的 C++ 翻译单元。", file=sys.stderr)
        return 2

    check_version(args.clang_tidy)
    extras = {
        key: query_compiler(*key) for key in {(u.compiler, u.flags) for u in units}
    }
    diagnostics: list[dict] = []
    failed: list[str] = []
    included: set[Path] = set()

    def run_all(batch: list[Unit], temporary: str, label: str) -> None:
        with concurrent.futures.ThreadPoolExecutor(args.jobs) as pool:
            futures = [
                pool.submit(
                    run_unit,
                    args.clang_tidy,
                    unit,
                    extras[(unit.compiler, unit.flags)],
                    Path(temporary, f"{label}{index}.yaml"),
                )
                for index, unit in enumerate(batch)
            ]
            for index, future in enumerate(futures):
                unit, code, output, includes = future.result()
                name = unit.file.relative_to(root).as_posix()
                # 只有编译错误会让 clang-tidy 返回非零（WarningsAsErrors 为空）。
                if code != 0:
                    failed.append(name)
                    print(output, file=sys.stderr)
                included.update(includes)
                diagnostics.extend(
                    collect(Path(temporary, f"{label}{index}.yaml"), root, scope)
                )
                print(f"[{label}{index + 1}/{len(batch)}] {name}", file=sys.stderr)

    with tempfile.TemporaryDirectory() as temporary:
        run_all(units, temporary, "")
        header_filter = load_header_filter(root)
        orphans = sorted(
            path
            for path in scope
            if path.endswith((".h", ".hpp"))
            and header_filter.search("/" + path)
            and (root / path).resolve() not in included
        )
        if orphans:
            print(f"孤立头文件 {len(orphans)} 个，单独检查：", file=sys.stderr)
            print("\n".join(orphans), file=sys.stderr)
            run_all(
                [header_unit((root / path).resolve(), units) for path in orphans],
                temporary,
                "h",
            )

    if failed:
        print(f"{len(failed)} 个翻译单元解析失败，结果不完整：", file=sys.stderr)
        print("\n".join(failed), file=sys.stderr)
        return 2

    # 同一声明会被每个包含它的翻译单元各报告一次，各自只带本单元看到的用法；
    # 修正取并集。只要有一个单元无法自动修正（如用法在宏展开里），整个标识符都不改。
    groups: dict[tuple[str, int, str], list[dict]] = collections.defaultdict(list)
    for diagnostic in diagnostics:
        key = (diagnostic["file"], diagnostic["offset"], diagnostic["name"])
        groups[key].append(diagnostic)
    rows = []
    edits: set[Edit] = set()
    for (path, offset, name), group in sorted(groups.items()):
        line, column = line_column(root, path, offset)
        fixable = all(diagnostic["edits"] for diagnostic in group)
        new = group[0]["new"] if fixable else "-"
        rows.append((path, line, column, group[0]["kind"], name, new))
        if fixable:
            edits.update(edit for diagnostic in group for edit in diagnostic["edits"])
    for path, line, column, kind, name, new in rows:
        print(f"{path}:{line}:{column}: {kind} '{name}' -> {new}")
    if args.report:
        with args.report.open("w", encoding="utf-8", newline="\n") as report:
            report.write("文件\t行\t列\t类别\t原名\t新名\n")
            for row in rows:
                report.write("\t".join(map(str, row)) + "\n")

    unfixable = [row for row in rows if row[5] == "-"]
    print(f"命名不合规 {len(rows)} 处（其中无自动修正 {len(unfixable)} 处）")
    if args.check:
        return 1 if rows else 0

    changed = apply_edits(root, edits, scope)
    print(f"已写回 {len(edits)} 处修改，涉及 {changed} 个文件")
    return 0


if __name__ == "__main__":
    sys.exit(main())
