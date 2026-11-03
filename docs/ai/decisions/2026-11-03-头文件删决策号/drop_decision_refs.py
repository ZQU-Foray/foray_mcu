"""删掉头文件注释里的决策号（负责人 2026-11-03）。用法：drop_decision_refs.py 文件...

决策号连同只对它有意义的后缀（D7、第 9 点、的 M0、方案 1、～03、、-03）一起删；
删完括号空了就连括号一起删，括号里还有别的内容就只删决策号和它前后的分隔符。
"""

import re
import sys

ONE = r"Decision-\d{8}-\d{2}(?:～\d{2})?(?:、-\d{2})*"
SUFFIX = r"(?:\s*(?:D\d+|第\s*[\d、–]+\s*点|的\s*M\d+|方案\s*\d+))?"
TOKEN = rf"{ONE}(?:、{ONE})*{SUFFIX}"

RULES = [
    (re.compile(rf"（{TOKEN}）"), ""),  # 括号里只有决策号
    (re.compile(rf"（{TOKEN}："), "（"),  # 括号以"决策号："开头
    (re.compile(rf"（{TOKEN}，"), "（"),  # 括号里决策号在前
    (re.compile(rf"，{TOKEN}(?=[）。，])"), ""),  # 决策号在后（括号可以跨行）
]

left = 0
for path in sys.argv[1:]:
    with open(path, encoding="utf-8") as file:
        text = file.read()
    new = text
    for pattern, repl in RULES:
        new = pattern.sub(repl, new)
    if new != text:
        with open(path, "w", encoding="utf-8", newline="\n") as file:
            file.write(new)
    for number, line in enumerate(new.splitlines(), 1):
        if "Decision-20" in line:
            left += 1
            print(f"未处理 {path}:{number}: {line.strip()}")
print(f"未处理 {left} 处")
