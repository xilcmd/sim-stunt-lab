# sim-stunt-lab

Scripted race and stunt scenarios for the BeamNG driving simulator, written in
Python with [BeamNGpy](https://github.com/BeamNG/BeamNGpy): an 8-car race with a
replay highlight reel (`beamng-race`), a downhill toy-track course
(`beamng-soapbox`), and a ski-jump show with random cars (`beamng-bigjump`). A
one-car starter demo (`beamng-demo`) loads the `west_coast_usa` map, spawns an
ETK 800, turns on the built-in AI driver, and prints speed, RPM, and gear once
per second.

A learning project: non-commercial, MIT licensed (see `LICENSE`).

## Credits and requirements

Built on [BeamNG.tech](https://beamng.tech) / BeamNG.drive and BeamNGpy by
BeamNG GmbH. You need your own copy of the simulator. This project is not made,
endorsed, or supported by BeamNG; BeamNG names are used only to say what the
code works with. All maps, vehicles and models come from the simulator and are
not part of this repository. The courses draw ideas from videos by Car Pal and
DestructionNation; no material from those videos is included.

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

   **BeamNG.drive 0.40.3 does not include the API** (`lua/ge/extensions/tech/` is
   missing), so `-tcom` crashes it at startup. Use 0.39.4 instead: in the Steam
   console (`steam://open/console`) run
   `download_depot 284160 284161 5975039348266591145`, then start the game from
   `C:\Program Files (x86)\Steam\steamapps\content\app_284160\depot_284161`.

2. In WSL, run a script. It finds the game by itself: `BNG_HOST` if set, else
   `localhost`, else the Windows host (the WSL default gateway).

   ```bash
   uv run beamng-demo --no-launch
   ```

   Windows Firewall must allow inbound TCP on port 25252.

## Stock car race

`beamng-race` builds a rounded-rectangle track on the flat `smallgrid` map and
races eight modern cars (see `CARS` in `race.py` for models, colours, and the
time-trial lap time of each). ETK is BeamNG's BMW stand-in; the white and silver
ETK GT-IV touring cars play the "BMW" and "Mercedes" (the game has no Mercedes
look-alike).

The grid is a reverse grid by time-trial lap time: slowest car on pole, fastest
at the back, two cars per row. A checkered start/finish line crosses the track
at the front of the grid.

The cars race with BeamNG's own racing AI (`ai.driveUsingPath`) with car
avoidance on. Each car races in its own corridor, 5 m wide, with the 8 corridor
centres spread from 7 m inside to 7 m outside the centreline. On one shared
full-width path every AI picks the same fastest line, so the field runs nose to
tail: with avoidance on nobody passes, and with it off (`--no-avoid-cars`) they
push each other off. A car that makes no progress for 15 s is out. The race ends 45 s after the winner
finishes; cars still running are placed by distance covered.

The camera cycles through TV-tower, chase, and blimp shots, and the terminal
prints the standings.

```bash
uv run beamng-race --no-launch
```

Flags: `--laps` (default 3), `--direction` (`cw` right turns, default; `ccw`
left turns), `--top-speed` (m/s), `--aggression` (BeamNG AI, 0.3 calm to 1.0 at
the limit; default 0.75, each car gets ±0.03), `--no-avoid-cars`,
`--shot-seconds`, `--seed`, plus the connection flags below.

Do not press keys or click in the game window during a race: Esc opens the
menu, and choosing a level there ends the race. Win+Alt+R is safe.

## Soapbox race

`beamng-soapbox` is a downhill race inspired by Car Pal's "the Soapbox race I
always wanted to do as a kid". Six cars start from a gate with lane dividers on
a solid starting block 80 m up, drive down a wide orange chevron ramp that
passes through a five-story building (in at a back window, out at a front one),
dip and climb over a steep symmetric hill, run down a pitched house roof, funnel
onto a narrow blue chevron ramp with gentle S-bends, and finish on the ground.
The blue ramp is a U-shaped channel (edges 1.2 m above the middle), built as
custom meshes because mesh roads are always flat across. The track is
about 1,600 m to the line. A runout with side walls and a crash wall stops the
cars after the finish. BeamNG cannot shrink cars, so the track is built big
instead.

```bash
uv run beamng-soapbox --no-launch
```

The textures are drawn in Python and written, with a materials file, into the
game's user folder under `levels/smallgrid/art/soapbox/`; the game loads them
with the map. The camera, standings, and highlight reel work as in the stock car
race. Flags: `--aggression` (default 0.8), `--shot-seconds`, `--seed`,
`--no-highlights`, `--reel-countdown`.

## Big jump

`beamng-bigjump` sends random cars from the game's car list, one at a time, at
full throttle down a ski-jump style ramp, inspired by DestructionNation's "Big
Ramp Jumps with Random Cars" video. A 150 m start tower leads to a 30° in-run
with hazard-striped edges and low walls, a level takeoff lip, and a long landing
hill with distance lines and signs every 25 m. Each jump is measured along the
landing hill from the knoll, like ski jumping, and a scoreboard follows the last
car. Grandstands line the outrun, a pool with water waits where wrecked cars
usually run out of speed, and light towers and a judges' tower stand beside the
hill.

By default the course stands on a hillside in the Italy map, looking out over
the sea: the spot was chosen by scoring placements against the map's height
grid, the trees along the course are removed, and posts and towers reach down
to the real ground. `--map smallgrid` builds it on the empty grid map instead,
with a grass slab and cone mountains.

Each car ends with one mark: in the pool (the win: a wreck that still limps
into the water), overshot the pool, short of the pool, or off the side. The
scoreboard shows jump distance (metres), the mark, and top speed (mph).

```bash
uv run beamng-bigjump --no-launch --jumps 10
```

Flags: `--jumps` (default 10), `--seed` (repeat the same car picks), `--map`
(`italy`, default, or `smallgrid`). The cars are driven by the game AI told to
hold an unreachable speed, because spawned cars ignore direct throttle input;
the AI is switched off at the lip.

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
