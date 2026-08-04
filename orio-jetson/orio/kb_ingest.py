"""CLI to ingest documents into a knowledge-base profile.

    uv run python -m orio.kb_ingest --profile grocery-store aisles.md
    uv run python -m orio.kb_ingest --profile grocery-store docs/ --replace

Splits each file on blank lines — one paragraph becomes one chunk, so write
source documents as short, self-contained paragraphs; a chunk is returned to
the LLM verbatim and read aloud. .md and .txt files only for now.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from . import knowledge


def _paragraphs(text: str) -> list[str]:
    return [p.strip() for p in text.split("\n\n") if p.strip()]


def _files(paths: list[str]) -> list[Path]:
    out = []
    for raw in paths:
        path = Path(raw)
        if path.is_dir():
            out.extend(sorted(path.rglob("*.md")) + sorted(path.rglob("*.txt")))
        elif path.is_file():
            out.append(path)
        else:
            print(f"skipping {path} (not found)", file=sys.stderr)
    return out


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--profile", required=True, help="Knowledge-base profile name, e.g. 'grocery-store'"
    )
    parser.add_argument(
        "--replace", action="store_true", help="Clear the profile's existing chunks first"
    )
    parser.add_argument("paths", nargs="+", help=".md/.txt files or directories of them")
    args = parser.parse_args()

    if args.replace:
        knowledge.clear(args.profile)

    files = _files(args.paths)
    if not files:
        print("No .md/.txt files found in the given paths.", file=sys.stderr)
        raise SystemExit(1)

    total = 0
    for path in files:
        chunks = _paragraphs(path.read_text(encoding="utf-8"))
        added = knowledge.ingest(args.profile, chunks, source=path.name)
        print(f"{path}: {added} chunk(s)")
        total += added
    print(f"Total: {total} chunk(s) ingested into profile '{args.profile}'")


if __name__ == "__main__":
    main()
