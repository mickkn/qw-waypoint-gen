# qw-waypoint-gen

Automatic **bot waypoint generator for KTX 1.47 QuakeWorld servers**, built from compiled
Quake BSP29 maps.

Point it at a `.bsp` and it produces a ready-to-use KTX route file, `<map>.bot`, in the
command format KTX reads at map load (`CreateMarker`, `SetZone`, `SetGoal`,
`SetMarkerPath`, `SetMarkerPathFlags`, `SetMarkerFlag`). Drop it into
`<gamedir>/ktx/bots/maps/` (or `maps/`) and `/botcommand add` works on that map.
It can also still emit Frogbot v2 `map_<name>.qc` files
([DrLex0/quake-frogbots](https://github.com/DrLex0/quake-frogbots)) with `--format frogbot`.

- **KTX-exact marker numbering**: KTX creates entity markers itself at map load (items,
  spawns, teleports, doors, buttons, triggers) and appends one marker per `CreateMarker`
  line. The generator reproduces that numbering, so all indices in the file line up.
  Verified against all 22 hand-made `.bot` files shipped with KTX that have a matching BSP.
- **KTX limits enforced**: 300 markers, 24 zones, 32 markers per zone, 8 paths per
  marker, and every path endpoint zoned (KTX route calculation indexes the target's
  zone). Large maps get wider custom-marker spacing instead of losing whole regions
  to the marker cap.
- **Real collision**: parses BSP clipnodes and ports the engine's hull tracer
  (`SV_HullPointContents` / `SV_RecursiveHullCheck`) so all queries use the true
  player collision hull, not render geometry.
- **QuakeWorld movement simulation**: step-walking with 18-unit stairs, wall sliding
  (PM_FlyMove-style), ballistic jump arcs (270 up / gravity 800 / run 320), fall-height
  limits, and water volume detection.
- **Custom markers** sampled from floor faces, hull-validated, spaced, and pruned.
- **Paths**: up to 8 outgoing paths per marker with directional diversity,
  `j` (`JUMP_LEDGE`) flags for committed jumps, and one-way teleporter wiring
  (`trigger_teleport` marker → `info_teleport_destination` marker).
- **Zones**: connected regions grown over the path graph within engine limits.
- **Goals**: priority numbering for quad/pent/armor/megahealth/big weapons; KTX's
  built-in default goals (G19-G24) for ammo and weak weapons are left alone.

## Install

```bash
python -m venv .venv
.venv/Scripts/activate       # Windows
pip install -e .[dev]
```

## Usage

Single map (writes `<map>.bot` in the current directory unless `--out` is given):

```bash
qw-wpgen generate /path/to/maps/ztndm3.bsp --out /path/to/ktx/bots/maps/ztndm3.bot
```

Every map in a directory, straight into the server's KTX folder. Existing `.bot` files,
such as the hand-made ones KTX ships, are kept unless you pass `--overwrite`:

```bash
qw-wpgen batch /path/to/qw/maps --out-dir /path/to/qw/ktx/bots/maps
```

Batch generation runs independent maps in worker processes to use multiple CPU cores.
Use `--workers 4` to limit concurrency, or `--workers 1` for sequential generation.
The default, `--workers 0`, uses CPU count minus one (at least one worker, up to
61 on Windows), limited to the number of maps needing generation. This leaves
some CPU capacity for other work. Each worker loads its own map,
so reduce the worker count if memory usage is too high.
Progress is printed as maps finish; the final JSON report is sorted by map name.
Invalid collision-hull references or cycles are reported as map failures, and
the remaining maps continue.

Options:

| Option            | Default | Meaning                                       |
|-------------------|---------|-----------------------------------------------|
| `--format`        | `ktx`   | `ktx` (`<map>.bot`) or `frogbot` (`map_<map>.qc`) |
| `--grid`          | 128     | Floor sampling grid spacing (map units)       |
| `--link-distance` | 400     | Maximum straight-line path link distance      |
| `--max-fall`      | ~264    | Maximum fall height allowed on paths          |
| `--workers`       | 0       | Batch only: process count; 0 = automatic, 1 = sequential |

`generate` prints a JSON summary (marker/path/zone/goal counts, unreachable items,
constraint violations) and exits nonzero if any hard engine limit is violated.

## Using the output on KTX

KTX looks for `maps/<map>.bot` first, then `bots/maps/<map>.bot`, inside the game
directory. When `k_entityfile` is set, it looks for that name instead. Bots only load
routes in deathmatch (not CTF or race). Once the file is in place, change map and
`/botcommand add`.

To refine a generated route, use KTX's in-game bot editor (`set k_fb_options 2`, editor
mode) and save over the generated file.

## Using the output with Frogbot (`--format frogbot`)

1. Copy `map_<name>.qc` into `quake-frogbots/src/maps/`.
2. From `src/maps/`, run `python generate_maplist.py -lg` to regenerate
   `map_load_gen.qc`.
3. Rebuild with `build-qwbot.sh` / `build-quakebot.sh`.

Or inject into an `.ent` file with `waypoint/waypoint_map_inject.py`.

## What to expect

Generated waypoints give bots solid basic navigation: full-map coverage, items, spawns,
and teleporters connected. They will not match hand-tuned files for advanced play —
rocket jumps, precision jumps, wall strafes, lift timing, shootable/touchable doors,
and view offsets are deliberately left for manual tuning in the bot editor. Items the simulator cannot connect (e.g. items on narrow beams
needing `NARROW_PATH` markers) are listed under `unreachable_items` in the summary so
you know exactly what to hand-fix.

## Design

```text
parsers/     BSP29 reader (entities, planes, faces, clipnodes, models, textures)
geom/        vector math
sim/         hull tracer (engine port) + QW movement simulation
nav/         marker placement, path graph, zones, goals
exporters/   KTX <map>.bot emitter and Frogbot map_<name>.qc emitter
validation/  engine limit checks and generation summary
```

A clone of the frogbots repo used for format research sits in `.reference/` (not
required at runtime).

## Tests

```bash
pytest
```

Integration tests run against real maps when they are present (`E:/[GAMES]/Quakeworld/id1/maps`
or the sibling `mvdsv-ktx-docker-main/data/media/maps`). They cross-check entity-marker
numbering against the hand-made `.bot` files shipped with KTX and the Frogbot reference files.
