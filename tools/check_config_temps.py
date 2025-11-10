#!/usr/bin/env python3
"""
Simple heuristic scanner to find suspicious usages of SelectConfig/StringConfig
that may be passed temporary String expressions (use-after-free risk in builder lambdas).

Search patterns:
 - SelectConfig\(.*"literal".*\)  (literal string passed directly)
 - SelectConfig\(.*String\([^\)]*\).*\)  (temporary String constructed inline)
 - StringConfig\(.*String\([^\)]*\).*\)
 - new String\([^\)]*\) occurrences are considered safe; we flag SelectConfig/StringConfig
   calls that don't use a pointer dereference (i.e., patterns without '*' before args).

This is a heuristic — review matches manually.
"""

import re
import sys
from pathlib import Path

root = Path(__file__).resolve().parents[1]
print(f"Scanning repository at: {root}\n")

select_re_literal = re.compile(r"SelectConfig\([^\)]*\"[^\"]+\"[^\)]*\)")
select_re_tempstring = re.compile(r"SelectConfig\([^\)]*String\([^\)]*\)[^\)]*\)")
stringcfg_temp = re.compile(r"StringConfig\([^\)]*String\([^\)]*\)[^\)]*\)")

suspicious = []

for f in root.rglob('src/**/*.cpp'):
    try:
        text = f.read_text(encoding='utf-8')
    except Exception as e:
        continue
    for m in select_re_literal.finditer(text):
        # skip cases where arg is dereferenced pointer like *l3_v1_default
        span = m.group(0)
        if '*"' in span or '*String' in span:
            continue
        suspicious.append((f, m.start(), span.strip()))
    for m in select_re_tempstring.finditer(text):
        suspicious.append((f, m.start(), m.group(0).strip()))
    for m in stringcfg_temp.finditer(text):
        suspicious.append((f, m.start(), m.group(0).strip()))

if not suspicious:
    print('No obvious suspicious SelectConfig/StringConfig usages found (heuristic).')
    sys.exit(0)

print('Potential issues found (manual review recommended):\n')
for f, pos, snippet in suspicious:
    # compute line number
    text = f.read_text(encoding='utf-8')
    line_no = text[:pos].count('\n') + 1
    print(f"{f}:{line_no}: {snippet}")

print('\nScan complete. Review the reported lines for temporaries passed to config constructors.')
