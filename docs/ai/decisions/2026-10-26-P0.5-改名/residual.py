"""残留扫描：列出代码中（去掉注释与字符串后）仍出现的旧名字。

clang-tidy 看不到未实例化模板里依赖类型的访问，编译器也只在实例化时检查，
这类遗漏只能靠扫描发现。旧名字恰好与仍在使用的合规名字同名时（如类型名 State、
别处的局部变量 tick）也会列出，需要人工排除；类型与命名空间名已自动排除。

用法（仓库根执行）：python residual.py <改名表.tsv>
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
TOKEN = re.compile(
    r"//[^\n]*|/\*.*?\*/"  # 注释
    r'|"(?:\\.|[^"\\\n])*"|\'(?:\\.|[^\'\\\n])*\''  # 字符串与字符字面量
    r"|[A-Za-z_][A-Za-z0-9_]*",
    re.DOTALL,
)
DECLARATION = re.compile(
    r"\b(?:class|struct|enum(?:\s+class)?|union|namespace|using)\s+([A-Za-z_]\w*)"
)


def main() -> int:
    (report,) = sys.argv[1:]
    with open(report, encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle, delimiter="\t"))
    old = {row["原名"] for row in rows}
    new = {row["新名"] for row in rows}

    listed = subprocess.run(
        ["git", "ls-files", "--", *DIRECTORIES],
        capture_output=True,
        text=True,
        check=True,
    )
    texts = {
        path: Path(path).read_text("utf-8")
        for path in listed.stdout.split()
        if path.endswith((".h", ".hpp", ".cpp"))
    }
    types = {name for text in texts.values() for name in DECLARATION.findall(text)}

    hits = collections.defaultdict(list)
    for path, text in texts.items():
        lines = text.splitlines()
        for match in TOKEN.finditer(text):
            token = match.group(0)
            if token[0] in "/\"'" or token not in old or token in new or token in types:
                continue
            line = text.count("\n", 0, match.start()) + 1
            hits[token].append(f"{path}:{line}: {lines[line - 1].strip()[:150]}")
    for token, locations in sorted(hits.items()):
        for location in locations:
            print(f"{token:24s} {location}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
