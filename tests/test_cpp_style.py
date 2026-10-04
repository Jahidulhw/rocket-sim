"""C++ hazard check that GoogleTest + g++ -Werror would otherwise only catch in CI.

GoogleTest's ASSERT_*/EXPECT_* macros expand to an if/else statement. Used as
the unbraced body of an if, else, for or while, the else becomes ambiguous
and g++ -Werror=dangling-else rejects the file (it broke CI once). MSVC and
clang-tidy accept it, so this scan runs locally with pytest.
"""

from __future__ import annotations

import re
from pathlib import Path

from sim.config import REPO_ROOT

MACRO = re.compile(r"\b(ASSERT|EXPECT)_[A-Z_]+\s*\(")
HEADER = re.compile(r"^\s*(\}\s*)?(else\s+if|if|for|while)\s*\(")
ELSE = re.compile(r"^\s*(\}\s*)?else\b(?!\s+if)")


def _after_condition(code: str, start: int) -> str | None:
    """Text after the parenthesised condition starting at/after `start`."""
    i = code.index("(", start)
    depth = 0
    for j in range(i, len(code)):
        depth += code[j] == "("
        depth -= code[j] == ")"
        if depth == 0:
            return code[j + 1:]
    return None   # condition continues on the next line


def unbraced_gtest_bodies(lines: list[str]) -> list[int]:
    """1-based line numbers of if/else/for/while whose unbraced body is a gtest macro."""
    hits = []
    for n, line in enumerate(lines):
        code = line.split("//")[0]
        m = HEADER.match(code)
        if m:
            body = _after_condition(code, m.start(2))
        elif ELSE.match(code):
            body = code[ELSE.match(code).end():]
        else:
            continue
        if body is None:
            continue
        rest = body.strip()
        if rest.startswith("{"):
            continue
        if rest == "":
            nxt = lines[n + 1].strip() if n + 1 < len(lines) else ""
            if MACRO.match(nxt):
                hits.append(n + 1)
        elif MACRO.match(rest):
            hits.append(n + 1)
    return hits


def test_scanner_detects_the_pattern():
    bad = ["  if (x) ASSERT_TRUE(y) << z;", "  for (int i = 0; i < 3; ++i)", "    EXPECT_EQ(a, b);",
           "  else EXPECT_GT(a, b);", "  if (x) {", "    ASSERT_TRUE(y);", "  }"]
    assert unbraced_gtest_bodies(bad) == [1, 2, 4]


def test_no_gtest_macro_is_an_unbraced_body():
    offenders = []
    for path in sorted((REPO_ROOT / "fc").rglob("*")):
        if path.suffix not in (".cpp", ".hpp", ".h", ".cc") or "build" in path.relative_to(REPO_ROOT).parts:
            continue
        lines = path.read_text(encoding="utf-8").splitlines()
        offenders += [f"{path.relative_to(REPO_ROOT).as_posix()}:{n}" for n in unbraced_gtest_bodies(lines)]
    assert offenders == [], "brace these (g++ -Werror=dangling-else): " + ", ".join(offenders)
