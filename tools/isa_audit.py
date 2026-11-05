# SPDX-License-Identifier: GPL-3.0-only
"""Report the explicit ISA backlog; completeness is a separate, failing gate."""

import argparse
from collections import Counter
import csv
from pathlib import Path
import sys


def audit(root: Path, require_complete: bool) -> int:
    path = root / "test/isa-coverage.tsv"
    with path.open(encoding="utf-8", newline="") as source:
        rows = list(csv.DictReader(source, delimiter="\t"))
    expected_columns = {"id", "manual_page", "status", "implementation", "tests", "remaining"}
    if not rows or set(rows[0]) != expected_columns:
        raise ValueError("Invalid ISA catalogue columns")
    identifiers = [row["id"] for row in rows]
    # 117 distinct entries in the Rev.5 instruction summary, plus LEA alias.
    if len(rows) != 118 or len(set(identifiers)) != len(identifiers):
        raise ValueError("Expected 118 distinct instruction/family/move-form entries")
    states = {"missing", "untested", "partial", "complete"}
    for row in rows:
        if row["status"] not in states or not row["manual_page"] or not row["remaining"]:
            raise ValueError(f"Invalid ISA entry: {row['id']}")
        for column in ("implementation", "tests"):
            for value in row[column].split(";"):
                if value != "-" and not (root / value).is_file():
                    raise ValueError(f"{row['id']}: absent {column} evidence: {value}")
        if row["status"] in {"partial", "complete"} and row["tests"] == "-":
            raise ValueError(f"{row['id']}: no test evidence")
        if row["status"] != "missing" and row["implementation"] == "-":
            raise ValueError(f"{row['id']}: no implementation evidence")
        if row["status"] == "complete" and row["remaining"] != "none":
            raise ValueError(f"{row['id']}: cannot be complete with remaining work")
    counts = Counter(row["status"] for row in rows)
    print(f"ISA catalogue: {len(rows)} entries; " + ", ".join(f"{state}={counts[state]}" for state in sorted(states)))
    for state in ("missing", "untested"):
        print(f"{state}: " + ", ".join(row["id"] for row in rows if row["status"] == state))
    incomplete = counts["complete"] != len(rows)
    print("INCOMPLETE: a green implementation test suite is not full ISA qualification." if incomplete
          else "Catalogue declares completeness; verify every linked format/mode/sequence witness separately.")
    return int(require_complete and incomplete)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--require-complete", action="store_true", help="Fail until every entry is fully qualified")
    args = parser.parse_args()
    try:
        return audit(Path(__file__).resolve().parent.parent, args.require_complete)
    except (OSError, ValueError) as error:
        print(f"ISA audit error: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
