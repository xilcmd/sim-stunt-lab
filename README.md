# beamng-demo

A small BeamNGpy demo. It loads the `west_coast_usa` map, spawns an ETK 800,
turns on the built-in AI driver, and prints speed, RPM, and gear once per second.

## Setup

```bash
cd ~/src/python/beamNG
uv sync
```

The `beamngpy` version must match your BeamNG.tech / BeamNG.drive version.
Change the pin in `pyproject.toml` if they differ.

## Run

### Option A: let the script start the game (game on the same OS as Python)

```bash
uv run beamng-demo --home "/path/to/BeamNG.tech"
```

### Option B: game already running (for example, game on Windows, Python in WSL)

1. On Windows, start the game with the API listening on all interfaces
   (use `BeamNG.drive.x64.exe` for the normal game):

   ```bat
   Bin64\BeamNG.tech.x64.exe -nosteam -console -tcom -tport 25252 -tcom-listen-ip 0.0.0.0
   ```

2. In WSL, connect to the Windows host:

   ```bash
   export BNG_HOST=$(ip route show default | awk '{print $3}')
   uv run beamng-demo --no-launch
   ```

   Windows Firewall must allow inbound TCP on port 25252.
   With WSL mirrored networking, `localhost` works and you can skip the export.

## Stock car race

`beamng-race` builds a rounded-rectangle track on the flat `smallgrid` map and
races four modern cars picked for close lap times (flying lap at AI aggression
0.8): an SBR4 Track (car1 red, 44.8 s), two ETK K-Series GT-IV touring cars
(car2 white as the "BMW", car4 silver as the "Mercedes", 45.1 s), and a Sunburst
Trackday (car3 yellow, 45.8 s). ETK is BeamNG's BMW stand-in; the game has no
Mercedes look-alike.
A checkered start/finish line crosses the track at the front of the grid.
The cars race with BeamNG's own racing AI (`ai.driveUsingPath` with
`avoidCars`): each picks its own line across the full track width and drives
around the others to pass. The camera cycles
through TV-tower, chase, and blimp shots, and the terminal prints the standings.

```bash
uv run beamng-race --no-launch --laps 5
```

Flags: `--laps`, `--direction` (`cw` right turns, default; `ccw` left turns),
`--top-speed` (m/s), `--aggression` (BeamNG AI, 0.3 calm to 1.0 at the limit;
default 0.8, each car gets ±0.08),
`--shot-seconds`, `--seed`, plus the connection flags below.

### Highlight reel

Each race is recorded with the game's replay system (saved under the game's
user folder in `replays/`). The script marks the start, passes (an order change
that holds 2 s), cars within 3 m of each other, crashes, and the win. After the
race it plays those moments back with captions and camera shots.

The game cannot export video. Record the screen while the reel plays: the game
shows a 5 s countdown; press **Win+Alt+R** (Windows Game Bar) to start and
again to stop. Use `--reel-countdown` to change the countdown and
`--no-highlights` to skip recording and the reel.

The camera runs inside the game as `raceCam.lua`. The script copies it into the
game's user folder (`lua/ge/extensions/raceCam.lua`) on each run.

## Options

| Flag | Env var | Default |
| --- | --- | --- |
| `--host` | `BNG_HOST` | `localhost` |
| `--port` | `BNG_PORT` | `25252` |
| `--home` | `BNG_HOME` | none |
| `--no-launch` | | off |
| `--seconds` | | `20` |
