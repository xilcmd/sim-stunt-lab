"""Stock car race: four AI cars on a rounded-rectangle track, with a TV-style camera."""

import argparse
import math
import os
import random
import time
from importlib import resources

from beamngpy import BeamNGpy, MeshRoad, Scenario, Vehicle, angle_to_quat

from beamng_demo import default_host
from beamng_demo.highlights import Spotter, play_reel

# Track: a rectangle with rounded corners on the flat smallgrid map, centred on
# the origin. Driven clockwise (right turns only) unless --direction ccw.
CLOCKWISE = True
TRACK_LENGTH = 600.0   # x extent of the centreline, metres
TRACK_DEPTH = 260.0    # y extent of the centreline, metres
CORNER_RADIUS = 90.0   # centreline corner radius, metres
TRACK_WIDTH = 24.0
# Starting grid: two columns (lateral offsets from the centreline, + is outward),
# rows GRID_ROW_GAP metres apart behind the line.
GRID_COLUMNS = (-4.0, 4.0)
GRID_ROW_GAP = 10.0
GROUND_Z = 0.0
PATH_SPACING = 8.0     # metres between racing-path nodes
# Each car races in its own corridor: a line LINE_SPREAD metres either side of the
# centreline at most, LINE_HALF_WIDTH metres wide each way. On one full-width path
# every AI picks the same fastest line, so the field ran nose to tail in a single
# lane: with avoidCars on nobody passed, with it off they pushed each other off.
LINE_SPREAD = 7.0
LINE_HALF_WIDTH = 2.5
# Each car keeps to its grid lane for LEAD_IN metres, then moves across to its
# corridor over WIDEN metres. Longer (60 + 80 m) queued 8 cars in single file
# and cost the back rows 8-11 s on lap 1.
LEAD_IN = 15.0
WIDEN = 40.0
LEAD_IN_WIDTH = 2.0    # path half-width while in the grid lane

# terrain_grass2 is a terrain material and shows "no texture" on a mesh, so the
# slab gets its own material built from the same grass image.
GRASS_MATERIAL = "race_grass"
GRASS_TEXTURE = "/levels/GridMap/art/shapes/misc/Grass-02-D.dds"
GRASS_TINT = "0.55 0.8 0.45 1"
SLAB_DEPTH = 1.0
WALL_MATERIAL = "concrete_plain"
WALL_GAP = 1.5         # metres from the track edge to the outer wall
WALL_HEIGHT = 1.2
WALL_THICKNESS = 0.8
WALL_CORNER_STEP = 2.0  # metres per wall block in the turns

AGGRESSION = 0.75     # BeamNG AI: 0.3 is shopping trip, 1.0 is the limit of grip
AGGRESSION_SPREAD = 0.03  # each car gets AGGRESSION +/- this, so paces differ
STUCK_SECONDS = 15.0   # a car that makes no progress this long is out of the race
FINISH_GRACE = 45.0    # seconds the others get to finish after the winner

CARS = (
    # vid, model, part config, colour (RGB 0..1), caption label, lap time (s).
    # Modern cars with close lap times on this track (flying lap, AI aggression
    # 0.8, one time trial on 2026-10-17). The grid is a reverse grid by lap time:
    # slowest on pole, fastest at the back, so the quick cars have to pass.
    # BeamNG has no real brands: ETK is its BMW stand-in, and the K-Series GT-IV
    # touring car also stands in for a Mercedes (no Mercedes look-alike exists).
    ("car1", "sbr", "vehicles/sbr/track.pc", (0.85, 0.10, 0.10), "Red SBR4", 44.8),
    ("car2", "etkc", "vehicles/etkc/race_DCT.pc", (0.92, 0.92, 0.95), "White GT-IV", 45.1),
    ("car3", "sunburst2", "vehicles/sunburst2/trackday_M.pc", (0.95, 0.75, 0.10),
     "Yellow Sunburst", 45.8),
    ("car4", "etkc", "vehicles/etkc/race_DCT.pc", (0.62, 0.64, 0.68), "Silver GT-IV", 45.1),
    ("car5", "etk800", "vehicles/etk800/rennspecht_8r_A.pc", (0.10, 0.30, 0.85),
     "Blue Rennspecht", 45.9),
    ("car6", "etkc", "vehicles/etkc/kc6x_trackday_A.pc", (0.10, 0.60, 0.25), "Green Kc6x", 47.3),
    ("car7", "sbr", "vehicles/sbr/TT_AWD_S2_DCT.pc", (0.95, 0.45, 0.05), "Orange SBR4 S2", 47.3),
    ("car8", "sunburst2", "vehicles/sunburst2/trackday_M.pc", (0.50, 0.15, 0.65),
     "Purple Sunburst", 45.8),
)
LABELS = {c[0]: c[4] for c in CARS}  # for on-screen captions

# Start/finish line: checkered squares across the track at s = 0.
LINE_SQUARE = 1.0      # metres
LINE_ROWS = 2


# ---------------------------------------------------------------- geometry

def track_point(s: float, offset: float = 0.0) -> tuple[float, float, float]:
    """Point at arc length s along the lane at `offset`, plus heading in radians.

    Starts in the middle of the bottom straight, heading +x. Clockwise is the
    counterclockwise layout mirrored across the x axis.
    """
    x, y, heading = _ccw_point(s, offset)
    return (x, -y, -heading) if CLOCKWISE else (x, y, heading)


def _segments(offset: float):
    """(kind, centre/start, angle, length) for each piece of the counterclockwise lane."""
    a = TRACK_LENGTH / 2 - CORNER_RADIUS   # half straight length (x)
    b = TRACK_DEPTH / 2 - CORNER_RADIUS    # half straight length (y)
    r = CORNER_RADIUS + offset
    return (
        ("straight", (0.0, -b - r), 0.0, a),                 # bottom, first half
        ("arc", (a, -b), -math.pi / 2, math.pi / 2 * r),      # bottom-right
        ("straight", (a + r, -b), math.pi / 2, 2 * b),        # right
        ("arc", (a, b), 0.0, math.pi / 2 * r),                # top-right
        ("straight", (a, b + r), math.pi, 2 * a),             # top
        ("arc", (-a, b), math.pi / 2, math.pi / 2 * r),       # top-left
        ("straight", (-a - r, b), -math.pi / 2, 2 * b),       # left
        ("arc", (-a, -b), math.pi, math.pi / 2 * r),          # bottom-left
        ("straight", (-a, -b - r), 0.0, a),                   # bottom, second half
    )


def _ccw_point(s: float, offset: float) -> tuple[float, float, float]:
    r = CORNER_RADIUS + offset
    segs = _segments(offset)
    s %= lane_length(offset)
    for kind, (cx, cy), ang, length in segs:
        if s <= length:
            if kind == "straight":
                return cx + s * math.cos(ang), cy + s * math.sin(ang), ang
            theta = ang + s / r
            return cx + r * math.cos(theta), cy + r * math.sin(theta), theta + math.pi / 2
        s -= length
    return segs[0][1][0], segs[0][1][1], 0.0


def lane_length(offset: float = 0.0) -> float:
    a = TRACK_LENGTH / 2 - CORNER_RADIUS
    b = TRACK_DEPTH / 2 - CORNER_RADIUS
    return 4 * a + 4 * b + 2 * math.pi * (CORNER_RADIUS + offset)


def offset_point(s: float, offset: float) -> tuple[float, float]:
    """Point `offset` metres outward from the centreline at centreline arc length s."""
    x, y, h = track_point(s)
    side = -1.0 if CLOCKWISE else 1.0  # outward is right of travel for ccw, left for cw
    return x + side * offset * math.sin(h), y - side * offset * math.cos(h)


def drive_race(car: Vehicle, start_s: float, lane: float, line: float, laps: int,
               aggression: float, top_speed: float, avoid_cars: bool) -> None:
    """Hand the car to BeamNG's own racing AI inside its own corridor.

    The corridor is centred `line` metres off the centreline. The AI picks its
    own speed and exact line inside it. With avoidCars on it sees the other cars
    and backs off when one is in its way; separate corridors mean that seldom
    happens, so faster cars can still pass.
    The AI first heads for the path's first node, so each car's path starts in
    its own grid lane and moves to its corridor after LEAD_IN + WIDEN metres;
    a shared start point makes all the cars converge and crash at the green flag.
    """
    end = (laps + 1) * lane_length()  # one spare lap so nobody runs out at the flag
    nodes = []
    d = 6.0
    while d < end:
        t = min(max((d - LEAD_IN) / WIDEN, 0.0), 1.0)
        t = t * t * (3 - 2 * t)  # smoothstep
        x, y = offset_point(start_s + d, lane + (line - lane) * t)
        r = LEAD_IN_WIDTH + (LINE_HALF_WIDTH - LEAD_IN_WIDTH) * t
        nodes.append(f"{{x={x:.2f}, y={y:.2f}, z={GROUND_Z + 0.02:.2f}, r={r:.1f}}}")
        d += PATH_SPACING
    car.queue_lua_command(
        f"ai.driveUsingPath{{script = {{{', '.join(nodes)}}}, "
        f"avoidCars = '{'on' if avoid_cars else 'off'}', aggression = {aggression:.3f}, "
        f"routeSpeed = {top_speed:.1f}, routeSpeedMode = 'limit'}}")


_CENTRELINE: list[tuple[float, float]] = []


def progress(pos) -> float:
    """Fraction of a lap (0..1) at the centreline point nearest to `pos`."""
    if not _CENTRELINE:  # sampled every 2 m; rebuilt per run since direction can change
        length = lane_length()
        _CENTRELINE.extend(track_point(i * 2.0)[:2] for i in range(int(length / 2)))
    best = min(range(len(_CENTRELINE)),
               key=lambda i: (_CENTRELINE[i][0] - pos[0]) ** 2 + (_CENTRELINE[i][1] - pos[1]) ** 2)
    return best / len(_CENTRELINE)


# ---------------------------------------------------------------- camera

def install_camera(bng: BeamNGpy, vids: list[str]) -> None:
    """Copy raceCam.lua into the game's user folder and load it.

    Each API call waits about one game frame, so a camera driven from Python
    stutters. The extension moves the camera every frame instead.
    """
    src = (resources.files(__package__) / "raceCam.lua").read_text()
    assert "]==]" not in src
    bng.control.queue_lua_command(
        f"writeFile('/lua/ge/extensions/raceCam.lua', [==[{src}]==])\n"
        "extensions.unload('raceCam')\n"
        "extensions.load('raceCam')")
    a = TRACK_LENGTH / 2
    b = TRACK_DEPTH / 2
    # TV towers outside each corner, plus mid-straight on the long sides.
    towers = [(sx * (a + 25), sy * (b + 25), 14.0) for sx in (-1, 1) for sy in (-1, 1)]
    towers += [(0.0, -b - 35, 10.0), (0.0, b + 35, 10.0)]
    lua_towers = ", ".join(f"{{{x}, {y}, {z}}}" for x, y, z in towers)
    lua_vids = ", ".join(f"'{v}'" for v in vids)
    bng.control.queue_lua_command(f"raceCam.setup({{{lua_vids}}}, {{{lua_towers}}})")


class Director:
    """Picks the next shot and target every few seconds; raceCam does the moving."""

    SHOTS = ("tv", "chase", "blimp")

    def __init__(self, bng: BeamNGpy, shot_seconds: float):
        self.bng = bng
        self.shot_seconds = shot_seconds
        self.shot_i = -1
        self.shot_started = -1e9
        self.target = None

    def tick(self, now: float, order: list[str]) -> None:
        if now - self.shot_started < self.shot_seconds:
            return
        self.shot_i = (self.shot_i + 1) % len(self.SHOTS)
        self.shot_started = now
        # Mostly follow the leader, sometimes someone in the pack.
        self.target = order[0] if random.random() < 0.5 else random.choice(order)
        self.bng.control.queue_lua_command(
            f"raceCam.setShot('{self.SHOTS[self.shot_i]}', '{self.target}')")

    def describe(self) -> str:
        return f"{self.SHOTS[self.shot_i]} on {self.target}"


def park(car: Vehicle) -> None:
    """AI off, parking brake on, engine off, so a wrecked car stops revving.

    Ignition-off calls do not hold on a wrecked car (ignition stays at 2 and the
    engine keeps revving), so switch off the engine device itself.
    """
    car.ai.set_mode("disabled")
    car.queue_lua_command(
        'input.event("throttle", 0, 1) input.event("parkingbrake", 1, 1) '
        'local e = powertrain.getDevice("mainEngine") if e then e:disable() end')


def car_positions(bng: BeamNGpy) -> tuple[dict[str, tuple[float, float, float]], float]:
    """All car positions, plus the replay recording time, in one round trip."""
    raw = bng.control.queue_lua_command("return raceCam.states()", response=True)
    out, replay_t = {}, 0.0
    for item in str(raw).split(";"):
        parts = item.split()
        if len(parts) == 4:
            out[parts[0]] = tuple(float(c) for c in parts[1:])
        elif len(parts) == 2 and parts[0] == "T":
            replay_t = float(parts[1])
    return out, replay_t


# ---------------------------------------------------------------- race

def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--host", default=default_host(),
                   help="Host where BeamNG listens (default: env BNG_HOST, else found automatically)")
    p.add_argument("--port", type=int, default=int(os.environ.get("BNG_PORT", "25252")),
                   help="BeamNG port (env BNG_PORT)")
    p.add_argument("--home", default=os.environ.get("BNG_HOME"),
                   help="BeamNG install folder; needed only to launch the game (env BNG_HOME)")
    p.add_argument("--no-launch", action="store_true",
                   help="Connect to a game that is already running")
    p.add_argument("--laps", type=int, default=3)
    p.add_argument("--no-avoid-cars", dest="avoid_cars", action="store_false",
                   help="AI ignores other cars: more contact and crashes")
    p.add_argument("--direction", choices=("cw", "ccw"), default="cw",
                   help="cw = right turns, ccw = left turns like most stock car ovals")
    p.add_argument("--top-speed", type=float, default=55.0,
                   help="Speed limit for the AI in m/s (55 is about 200 km/h)")
    p.add_argument("--aggression", type=float, default=AGGRESSION,
                   help="AI aggression, 0.3 (calm) to 1.0 (limit of grip); higher crashes more")
    p.add_argument("--shot-seconds", type=float, default=8.0, help="Seconds per camera shot")
    p.add_argument("--seed", type=int, default=None, help="Random seed for car pace and shots")
    p.add_argument("--no-highlights", dest="highlights", action="store_false",
                   help="Do not record a replay or play the highlight reel")
    p.add_argument("--reel-countdown", type=float, default=5.0,
                   help="Seconds to start screen recording before the reel plays")
    return p.parse_args()


def place_scenery(bng: BeamNGpy) -> None:
    """Grass slab over the grid floor, plus a concrete wall around the outside.

    BeamNGpy's ProceduralCube fails on BeamNG.drive: techCore grabs the
    proceduralPrimitives extension before it is loaded. So load it here and
    build the boxes with the same steps techCore uses.
    """
    boxes = []  # name, size, material, pos, quat
    # Slab top sits 1 cm above the floor and 1 cm below the track surface.
    boxes.append(("grass", (TRACK_LENGTH + 400, TRACK_DEPTH + 400, SLAB_DEPTH), GRASS_MATERIAL,
                  (0.0, 0.0, GROUND_Z + 0.01 - SLAB_DEPTH / 2), (0, 0, 0, 1)))
    # Outer wall: one block per straight, short chords through the turns so the
    # wall follows the curve without a sawtooth.
    offset = TRACK_WIDTH / 2 + WALL_GAP
    pieces = []  # (centre s, block length)
    s0 = 0.0
    for kind, _, _, length in _segments(offset):
        if kind == "straight":
            pieces.append((s0 + length / 2, length))
        else:
            n = math.ceil(length / WALL_CORNER_STEP)
            step = length / n
            chord = 2 * (CORNER_RADIUS + offset) * math.sin(step / (2 * (CORNER_RADIUS + offset)))
            pieces += [(s0 + (k + 0.5) * step, chord) for k in range(n)]
        s0 += length
    for i, (s, block) in enumerate(pieces):
        x, y, heading = track_point(s, offset)
        boxes.append((f"wall_{i:03d}", (block + 0.05, WALL_THICKNESS, WALL_HEIGHT),
                      WALL_MATERIAL, (x, y, GROUND_Z + WALL_HEIGHT / 2),
                      angle_to_quat((0, 0, math.degrees(heading)))))

    solid = len(boxes)
    # Checkered start/finish line, 1 cm thick on top of the 2 cm track surface.
    # Visual only: it is created after the collision rebuild below, because even a
    # 1 cm solid step stops the cars dead.
    cols = int(TRACK_WIDTH / LINE_SQUARE)
    for row in range(LINE_ROWS):
        s = (row - (LINE_ROWS - 1) / 2) * LINE_SQUARE
        for col in range(cols):
            lateral = (col - (cols - 1) / 2) * LINE_SQUARE
            x, y, heading = track_point(s, lateral)
            mat = "race_white" if (row + col) % 2 == 0 else "race_black"
            boxes.append((f"line_{row}_{col:02d}", (LINE_SQUARE, LINE_SQUARE, 0.01), mat,
                          (x, y, GROUND_Z + 0.025), angle_to_quat((0, 0, math.degrees(heading)))))

    def nums(v):
        return ", ".join(f"{c:.4f}" for c in v)

    lua = [
        "extensions.load('util_trackBuilder_proceduralPrimitives')",
        "local P = util_trackBuilder_proceduralPrimitives",
        "local function material(name, color, map)",
        "  if scenetree.findObject(name) then return end",
        "  local m = createObject('Material')",
        "  if map then m:setField('diffuseMap', 0, map) end",
        "  m:setField('diffuseColor', 0, color)",
        "  m:registerObject(name)",
        "end",
        f"material('{GRASS_MATERIAL}', '{GRASS_TINT}', '{GRASS_TEXTURE}')",
        "material('race_white', '0.95 0.95 0.95 1')",
        "material('race_black', '0.03 0.03 0.03 1')",
        "local function put(name, size, mat, pos, q)",
        "  local old = scenetree.findObject(name) if old then old:delete() end",
        "  local proc = createObject('ProceduralMesh')",
        "  proc:registerObject(name)",
        "  proc.canSave = false",
        "  scenetree.MissionGroup:add(proc.obj)",
        # createCube lays its first size component along local y, so swap x and y.
        "  proc:createMesh({{P.createCube(vec3(size[2], size[1], size[3]), mat)}})",
        "  proc:setPosition(vec3(pos[1], pos[2], pos[3]))",
        "  local r = quat(q[1], q[2], q[3], q[4]):toTorqueQuat()",
        "  proc:setField('rotation', 0, r.x .. ' ' .. r.y .. ' ' .. r.z .. ' ' .. r.w)",
        "  proc.scale = vec3(1, 1, 1)",
        "end",
    ]
    for i, (name, size, mat, pos, q) in enumerate(boxes):
        if i == solid:
            lua.append("be:reloadCollision()")
        lua.append(f"put('{name}', {{{nums(size)}}}, '{mat}', {{{nums(pos)}}}, {{{nums(q)}}})")
    bng.control.queue_lua_command("\n".join(lua))


def grid_s(i: int) -> float:
    """Arc length of grid slot i (0 = pole): two per row, behind the line."""
    return -4.0 - GRID_ROW_GAP * (i // 2)


def grid_lane(i: int) -> float:
    """Lateral offset of grid slot i; pole is on the inside."""
    return GRID_COLUMNS[i % len(GRID_COLUMNS)]


def racing_lines(n: int) -> list[float]:
    """Corridor centre for each grid slot: evenly spread across the track.

    Cars in the inside grid column get the inside half of the lines and the
    outside column the outside half, shuffled within each half, so nobody has
    to cut across the field at the start.
    """
    step = 2 * LINE_SPREAD / max(n - 1, 1)
    spread = [-LINE_SPREAD + k * step for k in range(n)]
    inside, outside = spread[: (n + 1) // 2], spread[(n + 1) // 2:]
    random.shuffle(inside)
    random.shuffle(outside)
    return [(inside if i % 2 == 0 else outside)[i // 2] for i in range(n)]


def grid_order() -> list[tuple]:
    """CARS as a reverse grid: slowest time-trial lap on pole, fastest last."""
    return sorted(CARS, key=lambda c: c[5], reverse=True)


def build_scenario(bng: BeamNGpy) -> tuple[Scenario, list[tuple[Vehicle, float]]]:
    scenario = Scenario("smallgrid", "stock_car_race")

    # smallgrid has a plain ground plane that decal roads cannot project onto, so the
    # track is a thin mesh road whose top sits just above the floor.
    road = MeshRoad("track_editor_C_center", side_material="track_editor_C_border",
                    rid="oval", default_width=TRACK_WIDTH, default_depth=0.3)
    length = lane_length()
    nodes = [(*track_point(i * 8.0)[:2], GROUND_Z + 0.02) for i in range(int(length / 8.0))]
    road.add_nodes(*nodes, nodes[0])  # repeat the first node to close the loop
    scenario.add_mesh_road(road)

    cars = []
    for i, (vid, model, config, color, _, _) in enumerate(grid_order()):
        offset = grid_lane(i)
        x, y, heading = track_point(grid_s(i), offset)
        car = Vehicle(vid, model=model, part_config=config, color=color, license=vid.upper())
        rot = angle_to_quat((0, 0, math.degrees(heading) + 90))  # vehicles face -y by default
        scenario.add_vehicle(car, pos=(x, y, GROUND_Z + 0.5), rot_quat=rot)
        cars.append((car, offset))

    scenario.make(bng)
    return scenario, cars


def main() -> None:
    global CLOCKWISE
    args = parse_args()
    CLOCKWISE = args.direction == "cw"
    random.seed(args.seed)

    bng = BeamNGpy(args.host, args.port, home=args.home)
    bng.open(launch=not args.no_launch)

    try:
        scenario, cars = build_scenario(bng)
        bng.scenario.load(scenario)
        bng.scenario.start()
        bng.control.pause()
        place_scenery(bng)

        aggression = {}
        lines = racing_lines(len(cars))
        for i, (car, lane) in enumerate(cars):
            aggression[car.vid] = min(1.0, max(0.3, args.aggression + random.uniform(
                -AGGRESSION_SPREAD, AGGRESSION_SPREAD)))
            drive_race(car, grid_s(i), lane, lines[i], args.laps, aggression[car.vid],
                       args.top_speed, args.avoid_cars)
        print("Aggression:", ", ".join(f"{v} {a:.2f}" for v, a in aggression.items()))

        install_camera(bng, [car.vid for car, _ in cars])
        director = Director(bng, args.shot_seconds)
        # Laps run so far, unwrapped from the 0..1 lap fraction. Back-row cars start
        # just behind the line, so they start slightly negative.
        total, last = {}, {}
        positions, _ = car_positions(bng)
        for car, _ in cars:
            frac = progress(positions[car.vid])
            last[car.vid] = frac
            total[car.vid] = frac - 1 if frac > 0.5 else frac
        finished: list[str] = []
        finish_time: dict[str, float] = {}
        out: list[str] = []  # stuck after a crash
        best = dict(total)   # furthest progress so far
        best_time = {vid: 0.0 for vid in total}
        best_rt = {vid: 0.0 for vid in total}  # same, in replay seconds
        last_print = 0.0
        spotter = Spotter(LABELS)
        replay_file = None
        if args.highlights:
            replay_file = bng.control.queue_lua_command(
                "local r = core_replay.startRecording() return r.success and r.filename or ''",
                response=True) or None

        print("Green flag!")
        bng.control.resume()
        t0 = time.monotonic()
        while len(finished) + len(out) < len(cars):
            now = time.monotonic() - t0
            positions, rt = car_positions(bng)
            for car, _ in cars:
                if car.vid in finished or car.vid in out or car.vid not in positions:
                    continue
                frac = progress(positions[car.vid])
                delta = frac - last[car.vid]
                if delta < -0.5:
                    delta += 1
                elif delta > 0.5:
                    delta -= 1
                total[car.vid] += delta
                last[car.vid] = frac
                if total[car.vid] >= args.laps:
                    finished.append(car.vid)
                    finish_time[car.vid] = now
                    car.ai.set_mode("stopping")
                    spotter.finish(rt, car.vid, len(finished))
                    print(f"  {now:6.1f}s  {car.vid} takes the flag in P{len(finished)}")
                elif total[car.vid] > best[car.vid] + 0.01:  # 1% of a lap counts as progress
                    best[car.vid], best_time[car.vid], best_rt[car.vid] = total[car.vid], now, rt
                elif now - best_time[car.vid] > STUCK_SECONDS:
                    out.append(car.vid)
                    park(car)
                    spotter.crash(best_rt[car.vid], car.vid)  # when it stopped making progress
                    print(f"  {now:6.1f}s  {car.vid} is stuck and out of the race")

            running = sorted((v for v in total if v not in finished and v not in out),
                             key=total.get, reverse=True)
            order = finished + running + out
            if replay_file:
                spotter.tick(rt, running, positions, total)
            director.tick(now, finished + running or order)

            if now - last_print > 3.0:
                last_print = now
                board = "  ".join(
                    f"P{i + 1} {v} " + ("OUT" if v in out else
                                        f"L{min(max(int(total[v]) + 1, 1), args.laps)}")
                    for i, v in enumerate(order))
                print(f"{now:6.1f}s  {board}   [cam: {director.describe()}]")

            if finished and now - finish_time[finished[0]] > FINISH_GRACE:
                print(f"{FINISH_GRACE:.0f} s after the winner: race over.")
                break
            if now > 60 + args.laps * 60:  # safety stop if a car is stuck
                print("Time limit reached.")
                break
            time.sleep(0.05)

        # Cars still running are placed by distance covered, behind the finishers.
        running = sorted((v for v in total if v not in finished and v not in out),
                         key=total.get, reverse=True)
        places = [f"P{i + 1} {LABELS[v]}" for i, v in enumerate(finished)]
        places += [f"P{len(finished) + i + 1} {LABELS[v]} ({max(int(total[v]), 0)} laps)"
                   for i, v in enumerate(running)]
        result = "  ".join(places)
        if out:
            result += "   Out: " + ", ".join(LABELS[v] for v in out)
        print("Result:", result)
        bng.ui.display_message(f"Checkered flag!  {result}")
        time.sleep(5)  # let the camera watch the cars slow down
        if replay_file:
            bng.control.queue_lua_command("core_replay.stopRecording()")
        for car, _ in cars:
            park(car)
        if replay_file:
            print(f"Replay saved: {replay_file} (in the game's user folder)")
            play_reel(bng, replay_file, spotter.moments, args.reel_countdown)
        bng.control.queue_lua_command("raceCam.stop()")
    finally:
        bng.disconnect()  # leave the game running so you can look around
