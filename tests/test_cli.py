"""
:File:      tests/test_cli.py

:Details:   Test batch parallelism, output preservation, and map failure reports.

:Date:      2026-11-08
:Author:    Mick K
"""

from __future__ import annotations

import json
import os
import struct
import subprocess
import sys
from concurrent.futures import Future
from pathlib import Path
from unittest.mock import MagicMock

import pytest
from typer.testing import CliRunner

from qw_waypoint_gen import cli
from qw_waypoint_gen.cli import app


@pytest.mark.parametrize("fmt", ["ktx", "frogbot"])
def test_parallel_batch_matches_sequential(
    tmp_path: Path, collision_bsp: bytes, fmt: str,
) -> None:
    maps = tmp_path / "maps"
    maps.mkdir()
    for name in ("alpha", "beta", "kept"):
        (maps / f"{name}.bsp").write_bytes(collision_bsp)
    broken = bytearray(collision_bsp)
    clip_offset = struct.unpack_from("<i", broken, 4 + 9 * 8)[0]
    struct.pack_into("<i", broken, clip_offset, 999)
    (maps / "broken.bsp").write_bytes(broken)
    def filename(name: str) -> str:
        return f"{name}.bot" if fmt == "ktx" else f"map_{name}.qc"

    reports = []
    for workers in (1, 2, 0):
        output = tmp_path / f"output-{workers}"
        output.mkdir()
        (output / filename("kept")).write_text("hand-made", encoding="utf-8")
        result = subprocess.run(
            [
                sys.executable, "-m", "qw_waypoint_gen.cli", "batch", str(maps),
                "--out-dir", str(output), "--workers", str(workers), "--format", fmt,
            ],
            capture_output=True, text=True, timeout=60, check=False,
        )
        assert result.returncode == 0, result.stderr
        report = json.loads(result.stdout)
        assert list(report) == ["alpha", "beta", "broken", "kept"]
        assert report["alpha"] == report["beta"] == "ok"
        assert report["kept"] == "skipped (exists)"
        assert "references plane 999" in report["broken"]
        assert str(maps / "broken.bsp") in report["broken"]
        assert "broken: failed:" in result.stderr
        assert not (output / filename("broken")).exists()
        assert (output / filename("kept")).read_text(encoding="utf-8") == "hand-made"
        for name in ("alpha", "beta"):
            generated = (output / filename(name)).read_bytes()
            assert generated == (tmp_path / "output-1" / filename(name)).read_bytes()
        reports.append(report)
    assert reports[0] == reports[1] == reports[2]


def test_batch_empty_directory(tmp_path: Path) -> None:
    result = CliRunner().invoke(app, ["batch", str(tmp_path), "--out-dir", str(tmp_path / "out")])
    assert result.exit_code == 0
    assert json.loads(result.stdout) == {}


def test_batch_rejects_negative_workers(tmp_path: Path) -> None:
    result = CliRunner().invoke(
        app, ["batch", str(tmp_path), "--out-dir", str(tmp_path / "out"), "--workers", "-1"],
    )
    assert result.exit_code == 2


def test_batch_overwrite(tmp_path: Path, collision_bsp: bytes) -> None:
    (tmp_path / "map.bsp").write_bytes(collision_bsp)
    target = tmp_path / "out" / "map.bot"
    target.parent.mkdir()
    target.write_text("hand-made", encoding="utf-8")
    result = CliRunner().invoke(
        app, ["batch", str(tmp_path), "--out-dir", str(target.parent), "--overwrite", "--workers", "1"],
    )
    assert result.exit_code == 0, result.output
    assert json.loads(result.stdout) == {"map": "ok"}
    assert target.read_text(encoding="utf-8") != "hand-made"


@pytest.mark.parametrize(
    ("cpu_count", "workers", "expected"),
    [(4, 0, 3), (1, 0, 1), (None, 0, 1), (128, 0, 61), (4, 5, 5)],
)
def test_batch_worker_count(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, cpu_count: int | None, workers: int, expected: int,
) -> None:
    for index in range(65):
        (tmp_path / f"map{index}.bsp").touch()
    monkeypatch.setattr(os, "cpu_count", lambda: cpu_count)
    monkeypatch.setattr(sys, "platform", "win32")
    monkeypatch.setattr(cli, "_batch_generate_one", lambda *args: {"problems": [], "markers_total": 2})
    executor = MagicMock()

    def completed_future(*args: object) -> Future[dict[str, object]]:
        future: Future[dict[str, object]] = Future()
        future.set_result({"problems": [], "markers_total": 2})
        return future

    executor.return_value.__enter__.return_value.submit.side_effect = completed_future
    monkeypatch.setattr(cli, "ProcessPoolExecutor", executor)
    result = CliRunner().invoke(
        app, ["batch", str(tmp_path), "--out-dir", str(tmp_path / "out"), "--workers", str(workers)],
    )
    assert result.exit_code == 0, result.output
    assert len(json.loads(result.stdout)) == 65
    if expected == 1:
        executor.assert_not_called()
    else:
        executor.assert_called_once_with(max_workers=expected)
