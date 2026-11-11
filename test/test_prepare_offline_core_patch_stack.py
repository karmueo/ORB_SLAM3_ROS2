#!/usr/bin/env python3
"""验证核心 patch stack 状态识别在 fresh/partial/complete/unknown 下安全。"""

from __future__ import annotations

import difflib
import subprocess
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
HELPER = ROOT / "scripts" / "apply_core_patch_stack.sh"


def _run(command: list[str], cwd: Path) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        command,
        cwd=cwd,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
    )


def _write_unified_diff(path: Path, before: str, after: str) -> None:
    lines = list(
        difflib.unified_diff(
            before.splitlines(keepends=True),
            after.splitlines(keepends=True),
            fromfile="a/include/value.h",
            tofile="b/include/value.h",
            n=3,
        )
    )
    assert lines
    path.write_text(
        "diff --git a/include/value.h b/include/value.h\n" + "".join(lines),
        encoding="utf-8",
    )


def _make_repo(tmp_path: Path) -> tuple[Path, Path, Path, Path]:
    repo = tmp_path / "repo"
    (repo / "include").mkdir(parents=True)
    target = repo / "include" / "value.h"
    target.write_text("value = 0\ncontext_a\ncontext_b\n", encoding="utf-8")
    assert _run(["git", "init", "-q"], repo).returncode == 0
    assert _run(["git", "config", "user.email", "test@example.com"], repo).returncode == 0
    assert _run(["git", "config", "user.name", "test"], repo).returncode == 0
    assert _run(["git", "add", "include/value.h"], repo).returncode == 0
    assert _run(["git", "commit", "-q", "-m", "base"], repo).returncode == 0

    first = "value = 1\ncontext_a\ncontext_b\n"
    second = "value = 2\ncontext_a\ncontext_b\nextra\n"
    patch_one = tmp_path / "0001.patch"
    patch_two = tmp_path / "0002.patch"
    _write_unified_diff(patch_one, target.read_text(), first)
    _write_unified_diff(patch_two, first, second)
    return repo, target, patch_one, patch_two


def _run_helper(repo: Path, *patches: Path) -> subprocess.CompletedProcess[str]:
    return _run(
        ["bash", str(HELPER), str(repo), *(str(patch) for patch in patches)],
        repo,
    )


def test_fresh_partial_complete_and_rerun(tmp_path: Path) -> None:
    repo, target, patch_one, patch_two = _make_repo(tmp_path)
    expected = "value = 2\ncontext_a\ncontext_b\nextra\n"

    fresh = _run_helper(repo, patch_one, patch_two)
    assert fresh.returncode == 0, fresh.stderr
    assert target.read_text() == expected

    rerun = _run_helper(repo, patch_one, patch_two)
    assert rerun.returncode == 0, rerun.stderr
    assert "already complete" in rerun.stdout
    assert target.read_text() == expected

    # 只应用 0001 是合法的可推进状态；第二次运行只补齐 0002。
    target.write_text("value = 1\ncontext_a\ncontext_b\n", encoding="utf-8")
    partial = _run_helper(repo, patch_one, patch_two)
    assert partial.returncode == 0, partial.stderr
    assert "0002.patch" in partial.stdout
    assert target.read_text() == expected


def test_unknown_change_is_rejected_and_preserved(tmp_path: Path) -> None:
    repo, target, patch_one, patch_two = _make_repo(tmp_path)
    target.write_text("value = 999\ncontext_a\ncontext_b\n", encoding="utf-8")

    rejected = _run_helper(repo, patch_one, patch_two)
    assert rejected.returncode == 3
    assert "未知" in rejected.stderr
    assert target.read_text() == "value = 999\ncontext_a\ncontext_b\n"


def test_unknown_extra_tracked_change_is_rejected(tmp_path: Path) -> None:
    repo, target, patch_one, patch_two = _make_repo(tmp_path)
    other = repo / "include" / "other.h"
    other.write_text("base\n", encoding="utf-8")
    assert _run(["git", "add", "include/other.h"], repo).returncode == 0
    assert _run(["git", "commit", "-q", "-m", "other"], repo).returncode == 0
    assert _run_helper(repo, patch_one, patch_two).returncode == 0
    other.write_text("local change\n", encoding="utf-8")

    rejected = _run_helper(repo, patch_one, patch_two)
    assert rejected.returncode == 3
    assert "未知" in rejected.stderr
    assert other.read_text() == "local change\n"
    assert target.read_text() == "value = 2\ncontext_a\ncontext_b\nextra\n"
