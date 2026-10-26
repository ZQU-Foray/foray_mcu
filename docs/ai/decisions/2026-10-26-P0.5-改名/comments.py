#!/usr/bin/env python3
"""把 C/C++ 注释中提到的旧标识符同步为新名字（P0.5 改名的手工修正之一）。

只处理满足以下条件的名字，避免误改普通英文单词：
- 驼峰形式：camelCase，或至少两段的 PascalCase（如 GetReadCount、PostKernel）；
- 改名表中映射唯一（同一个旧名在不同位置改成了不同新名的不处理）；
- 不是仍在使用的类型或命名空间名（如 ChassisTask）。

用法（仓库根执行）：comments.py <改名表.tsv>
"""

import collections
import csv
import re
import subprocess
import sys
from pathlib import Path

DIRECTORIES = (
    "legacy/Application",
    "legacy/Libraries",
    "legacy/Platform/Interface",
    "test/host",
)
CAMEL = re.compile(r"^(?:[a-z]+[A-Z]|[A-Z][a-z0-9]+[A-Z])")
COMMENT = re.compile(r"//[^\n]*|/\*.*?\*/", re.DOTALL)
DECLARATION = re.compile(
    r"\b(?:class|struct|enum(?:\s+class)?|union|namespace|using)\s+([A-Za-z_]\w*)"
)


def main() -> int:
    (report,) = sys.argv[1:]
    with open(report, encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle, delimiter="\t"))
    mapping: dict[str, set[str]] = collections.defaultdict(set)
    for row in rows:
        if row["新名"] != "-":
            mapping[row["原名"]].add(row["新名"])

    listed = subprocess.run(
        ["git", "ls-files", "--", *DIRECTORIES],
        capture_output=True,
        text=True,
        check=True,
    )
    files = [
        Path(path)
        for path in listed.stdout.split()
        if path.endswith((".h", ".hpp", ".cpp"))
    ]
    texts = {path: path.read_text("utf-8") for path in files}
    types = {name for text in texts.values() for name in DECLARATION.findall(text)}
    usable = {
        old: next(iter(new))
        for old, new in mapping.items()
        if len(new) == 1 and old not in types and CAMEL.match(old)
    }
    alternatives = "|".join(sorted(map(re.escape, usable), key=len, reverse=True))
    word = re.compile(rf"(?<![A-Za-z0-9_])({alternatives})(?![A-Za-z0-9_])")

    changed = collections.Counter()
    for path, text in texts.items():

        def replace(match: re.Match[str], path: Path = path) -> str:
            changed[path] += len(word.findall(match.group(0)))
            return word.sub(lambda found: usable[found.group(1)], match.group(0))

        updated = COMMENT.sub(replace, text)
        if updated != text:
            path.write_text(updated, "utf-8", newline="\n")
    print(
        f"注释中更新 {sum(changed.values())} 处，涉及 {len([p for p in changed if changed[p]])} 个文件"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
