"""
:File:      tests/test_integration.py

:Details:   End-to-end generation tests against real Quake BSP maps. Skipped
            automatically when the maps are not present on this machine.

:Date:      2026-11-07
:Author:    Mick K
"""

from __future__ import annotations

from pathlib import Path

import pytest

from qw_waypoint_gen.exporters.frogbot_qc import render_frogbot_qc
from qw_waypoint_gen.exporters.ktx_bot import render_ktx_bot
from qw_waypoint_gen.generator import generate_waypoints
from qw_waypoint_gen.validation.checks import validate

_SERVER_DIR = Path(__file__).resolve().parents[2] / "mvdsv-ktx-docker-main"
_MAPS_DIR_CANDIDATES = (
    Path("E:/[GAMES]/Quakeworld/id1/maps"),
    _SERVER_DIR / "data/media/maps",
)
_MAPS_DIR = next((d for d in _MAPS_DIR_CANDIDATES if d.is_dir()), _MAPS_DIR_CANDIDATES[0])
# Hand-made route files shipped with KTX 1.47.
_KTX_BOTS_DIR = _SERVER_DIR / "server/files/ktx/bots/maps"

# Reference entity marker counts derived from the hand-made waypoint files in
# DrLex0/quake-frogbots (custom_marker_start - 1).
_REFERENCE_ENTITY_COUNTS = {
    "ztndm3": 51,
    "aerowalk": 49,
}


def _map_available(name: str) -> bool:
    """Return True if the given BSP exists locally."""
    return (_MAPS_DIR / f"{name}.bsp").exists()


@pytest.mark.parametrize("map_name", ["ztndm3", "aerowalk"])
def test_generation_matches_reference_entity_count(map_name: str) -> None:
    if not _map_available(map_name):
        pytest.skip(f"{map_name}.bsp not available")
    result = generate_waypoints(_MAPS_DIR / f"{map_name}.bsp")

    entity_count = sum(1 for m in result.markers if not m.is_custom)
    assert _REFERENCE_ENTITY_COUNTS[map_name] == entity_count

    assert not validate(result.markers)

    routable = sum(1 for m in result.markers if m.paths)
    assert routable >= len(result.markers) * 0.9

    text = render_frogbot_qc(result.map_name, result.markers)
    assert f"custom_marker_start={entity_count + 1};" in text
    assert "LSQ();" in text


def test_teleporters_get_one_way_paths() -> None:
    if not _map_available("ztndm3"):
        pytest.skip("ztndm3.bsp not available")
    result = generate_waypoints(_MAPS_DIR / "ztndm3.bsp")

    by_index = {m.index: m for m in result.markers}
    triggers = [m for m in result.markers if "trigger_teleport" == m.classname]
    assert triggers
    for trigger in triggers:
        assert 1 == len(trigger.paths)
        destination = by_index[trigger.paths[0]]
        assert "info_teleport_destination" == destination.classname


def _ktx_reference_entity_count(bot_text: str) -> int:
    """Derive KTX's runtime entity marker count from a hand-made .bot file."""
    creates = 0
    highest = 0
    for line in bot_text.splitlines():
        tokens = line.split()
        if not tokens:
            continue
        if "CreateMarker" == tokens[0]:
            creates += 1
        elif tokens[0] in ("SetZone", "SetGoal", "SetMarkerFlag"):
            highest = max(highest, int(tokens[1]))
        elif "SetMarkerPath" == tokens[0]:
            highest = max(highest, int(tokens[1]), int(tokens[3]))
    return highest - creates


@pytest.mark.parametrize("map_name", ["aerowalk", "ztndm3", "schloss", "cmt4"])
def test_ktx_bot_matches_reference_numbering(map_name: str) -> None:
    reference = _KTX_BOTS_DIR / f"{map_name}.bot"
    if not _map_available(map_name) or not reference.exists():
        pytest.skip(f"{map_name}.bsp or reference .bot not available")
    result = generate_waypoints(_MAPS_DIR / f"{map_name}.bsp")

    entity_count = sum(1 for m in result.markers if not m.is_custom)
    assert _ktx_reference_entity_count(reference.read_text(errors="replace")) == entity_count
    assert len(result.markers) <= 300
    assert not validate(result.markers)

    text = render_ktx_bot(result.map_name, result.markers)
    creates = sum(1 for ln in text.splitlines() if ln.startswith("CreateMarker "))
    assert len(result.markers) - entity_count == creates
