"""Big jump: random cars, one at a time, launch off a giant ski-jump ramp.

Inspired by DestructionNation's "Big Ramp Jumps with Random Cars" video: a
tall in-run with hazard-striped edges, a takeoff lip, and a long steep landing
hill with distance markers. Each car drives full throttle down the in-run; the
jump is measured along the landing hill from the knoll, like ski jumping.
"""

import argparse
import math
import os
import random
import time
from collections import deque

from beamngpy import BeamNGpy, MeshRoad, Scenario, Vehicle, angle_to_quat
from beamngpy.logging import BNGError

from beamng_demo import default_host, world
from beamng_demo.world import car_positions, park

LEVEL = "smallgrid"
ART = f"/levels/{LEVEL}/art/bigjump"  # game path; the files go in the user folder
GROUND_Z = 0.0

# Profiles are built from exact straight lines and circular arcs, in the x-z
# plane along y = 0, toward +x. ("line", length, angle) and
# ("arc", radius, from angle, to angle); angles in degrees, positive is uphill.
# With a +10 degree lip and a 125 m start, test cars flew about 180 m, past the
# end of the steep landing slope, and landed on the flat. A level lip keeps the
# flight closer to the slope, and the higher start makes the landing hill
# longer without changing the in-run drop.
START_X, START_Z = -15.0, 150.0
INRUN = [
    ("line", 27, 0),            # start platform (long enough for a truck)
    ("arc", 40, 0, -30),
    ("line", 110, -30),         # the in-run
    ("arc", 100, -30, 0),       # transition into the lip
    ("line", 6, 0),             # takeoff lip, level
]
KNOLL_DROP = 4.0                # landing hill starts this far below the lip
LANDING = [
    ("line", 15, -5),           # knoll
    ("arc", 120, -5, -33),
    ("line", None, -33),        # length set so the outrun lands on the ground
    ("arc", 180, -33, 0),
    ("line", 260, 0),           # outrun
]
INRUN_WIDTH = 8.0
STRIPE_WIDTH = 1.0              # hazard stripes along both in-run edges
LANDING_WIDTH = 40.0
ROAD_DEPTH = 1.5
NODE_STEP = 3.0                 # metres between mesh road nodes
WALL_HEIGHT = 1.0
WALL_STEP = 6.0
MARK_FIRST, MARK_EVERY, MARK_LAST = 50, 25, 300  # distance lines and signs, metres

SPAWN_X = 0.0                   # on the start platform
FULL_THROTTLE_SPEED = 100.0     # m/s the AI is told to hold: more than any car reaches
AI_FIRST_NODE = 12.0            # metres ahead of the car; at 6 m the car passed it at
                                # once and the AI backed up and stopped
OVERVIEW_CAMERA = ((90.0, 260.0, 130.0), (0.05, -0.92, -0.38))  # side view of the course
GROUND_CAM_AFTER = 60.0         # metres past the lip where the ground camera takes over
GROUND_CAM_AHEAD = 30.0         # it stands this far onto the flat outrun...
GROUND_CAM_SIDE = 12.0          # ...this far out from the track edge...
GROUND_CAM_HEIGHT = 1.7         # ...at eye level
ORBIT_AGAIN_AT = 200.0          # metres down the landing hill: back to the orbit camera
MAX_BELIEVABLE_SPEED = 150.0    # m/s; faster readings are position glitches
SPEED_WINDOW = 0.5              # seconds to average speed over
AIRBORNE = 3.0                  # metres above the surface that counts as flying
TOUCHDOWN = 1.6                 # ... and back below this counts as landing
RUN_TIMEOUT = 35.0
STOPPED_SPEED = 0.7
STOPPED_FOR = 2.5
NOT_CARS = {"unicycle"}         # models that are not drivable cars

TEXTURE_SIZE = 256

# Decorations
STAND_LENGTH, STAND_ROWS, STAND_DEPTH, STAND_RISE, STAND_GAP = 140.0, 6, 2.6, 1.3, 6.0
POOL_LENGTH, POOL_WALL, POOL_HEIGHT, POOL_WATER = 34.0, 0.8, 1.6, 1.3
JUDGES_AT = 100.0              # metres down the landing hill
LIGHT_EVERY, LIGHT_HEIGHT = 50.0, 16.0
MOUNTAINS, MOUNTAIN_SEED = 12, 7
MOUNTAIN_DISTANCE = (600.0, 900.0)
MOUNTAIN_RADIUS = (160.0, 320.0)
MOUNTAIN_HEIGHT = (140.0, 320.0)


# ---------------------------------------------------------------- geometry

class Profile:
    """A path of lines and arcs, sampled every `step` metres along it."""

    def __init__(self, x0: float, z0: float, segments: list[tuple], step: float = 0.5):
        self.x, self.z, self.s, self.angle = [x0], [z0], [0.0], [segments[0][-1]]
        for seg in segments:
            if seg[0] == "line":
                _, length, a = seg
                a0 = a1 = a
            else:
                _, radius, a0, a1 = seg
                length = radius * math.radians(abs(a1 - a0))
            n = max(1, math.ceil(length / step))
            ds = length / n
            for k in range(n):
                a = math.radians(a0 + (a1 - a0) * (k + 0.5) / n)
                self.x.append(self.x[-1] + math.cos(a) * ds)
                self.z.append(self.z[-1] + math.sin(a) * ds)
                self.s.append(self.s[-1] + ds)
                self.angle.append(a0 + (a1 - a0) * (k + 1) / n)

    def _index(self, x: float) -> int:
        lo, hi = 0, len(self.x) - 1
        while lo < hi - 1:
            mid = (lo + hi) // 2
            if self.x[mid] <= x:
                lo = mid
            else:
                hi = mid
        return lo

    def at_x(self, x: float) -> tuple[float, float]:
        """(surface z, distance along) at x, clamped to the ends."""
        if x <= self.x[0]:
            return self.z[0], 0.0
        if x >= self.x[-1]:
            return self.z[-1], self.s[-1]
        i = self._index(x)
        t = (x - self.x[i]) / (self.x[i + 1] - self.x[i])
        return (self.z[i] + (self.z[i + 1] - self.z[i]) * t,
                self.s[i] + (self.s[i + 1] - self.s[i]) * t)

    def at_s(self, s: float) -> tuple[float, float, float]:
        """(x, z, angle in degrees) at distance s along."""
        i = min(max(round(s / (self.s[1] - self.s[0])), 0), len(self.s) - 1)
        return self.x[i], self.z[i], self.angle[i]

    def end(self) -> tuple[float, float]:
        return self.x[-1], self.z[-1]


def build_course() -> tuple[Profile, Profile]:
    inrun = Profile(START_X, START_Z, INRUN)
    lip_x, lip_z = inrun.end()
    start_z = lip_z - KNOLL_DROP
    # Size the steep straight so the outrun ends up on the ground.
    probe = Profile(lip_x, start_z, [s if s[1] is not None else ("line", 0, s[2]) for s in LANDING])
    drop_left = probe.end()[1] - (GROUND_Z + 0.02)
    steep = next(s[2] for s in LANDING if s[1] is None)
    length = drop_left / -math.sin(math.radians(steep))
    if length < 0:
        raise ValueError("START_Z is too low for this landing hill")
    landing = Profile(lip_x, start_z, [s if s[1] is not None else ("line", length, s[2])
                                       for s in LANDING])
    return inrun, landing


# ---------------------------------------------------------------- textures

def _hazard(x, y):
    return (245, 200, 20) if ((x + y) // 32) % 2 == 0 else (25, 25, 25)


def _concrete(x, y):
    n = ((x * 73856093) ^ (y * 19349663)) & 0xFF  # cheap repeatable speckle
    g = 178 + (n % 18) - 9
    return (g, g, g + 2)


SEGMENTS = {  # 7-segment digits: a b c d e f g
    "0": "abcdef", "1": "bc", "2": "abdeg", "3": "abcdg", "4": "bcfg",
    "5": "acdfg", "6": "acdefg", "7": "abc", "8": "abcdefg", "9": "abcdfg",
}


def _sign(number: int):
    """White board with black 7-segment digits."""
    text = str(number)
    digit_w, digit_h, gap, thick = 56, 110, 18, 12
    total = len(text) * digit_w + (len(text) - 1) * gap
    x0, y0 = (TEXTURE_SIZE - total) // 2, (TEXTURE_SIZE - digit_h) // 2

    def lit(px, py):
        for k, ch in enumerate(text):
            dx = px - (x0 + k * (digit_w + gap))
            dy = py - y0
            if not (0 <= dx < digit_w and 0 <= dy < digit_h):
                continue
            mid = digit_h // 2
            segs = SEGMENTS[ch]
            if "a" in segs and dy < thick:
                return True
            if "g" in segs and abs(dy - mid) < thick // 2:
                return True
            if "d" in segs and dy >= digit_h - thick:
                return True
            if "f" in segs and dx < thick and dy < mid:
                return True
            if "b" in segs and dx >= digit_w - thick and dy < mid:
                return True
            if "e" in segs and dx < thick and dy >= mid:
                return True
            if "c" in segs and dx >= digit_w - thick and dy >= mid:
                return True
        return False

    def pixel(px, py):
        border = px < 8 or py < 8 or px >= TEXTURE_SIZE - 8 or py >= TEXTURE_SIZE - 8
        return (200, 30, 30) if border else ((15, 15, 15) if lit(px, py) else (245, 245, 245))
    return pixel


def _mirrored(pixel):
    def flipped(px, py):
        return pixel(TEXTURE_SIZE - 1 - px, py)
    return flipped


def marks() -> list[int]:
    return list(range(MARK_FIRST, MARK_LAST + 1, MARK_EVERY))


def _tiles(x, y):
    grout = x % 32 < 2 or y % 32 < 2
    return (70, 120, 170) if grout else (235, 242, 248)


def install_art(bng: BeamNGpy) -> None:
    textures = {"hazard.png": _hazard, "concrete.png": _concrete, "tiles.png": _tiles}
    materials = {"jump_hazard": "hazard.png", "jump_concrete": "concrete.png"}
    for d in marks():
        textures[f"sign_{d}.png"] = _sign(d)
        materials[f"jump_sign_{d}"] = f"sign_{d}.png"
        textures[f"sign_{d}_back.png"] = _mirrored(_sign(d))
        materials[f"jump_sign_{d}_back"] = f"sign_{d}_back.png"
    world.install_art(bng, ART, textures, materials, TEXTURE_SIZE)


# ---------------------------------------------------------------- scene

def _road(rid: str, material: str, profile: Profile, lateral: float, width: float) -> MeshRoad:
    road = MeshRoad(material, side_material="jump_concrete", rid=rid,
                    default_width=width, default_depth=ROAD_DEPTH, texture_length=8)
    nodes = []
    s = 0.0
    while s < profile.s[-1]:
        x, z, _ = profile.at_s(s)
        nodes.append((x, lateral, z, width, ROAD_DEPTH))
        s += NODE_STEP
    x, z = profile.end()
    nodes.append((x, lateral, z, width, ROAD_DEPTH))
    road.add_nodes(*nodes)
    return road


def build_scenario(bng: BeamNGpy, inrun: Profile, landing: Profile) -> Scenario:
    scenario = Scenario(LEVEL, "big_jump")
    scenario.add_mesh_road(_road("jump_inrun", "track_editor_C_center", inrun, 0.0, INRUN_WIDTH))
    edge = INRUN_WIDTH / 2 + STRIPE_WIDTH / 2
    for side in (-1, 1):
        scenario.add_mesh_road(_road(f"jump_stripe_{side + 1}", "jump_hazard", inrun,
                                     side * edge, STRIPE_WIDTH))
    scenario.add_mesh_road(_road("jump_landing", "jump_concrete", landing, 0.0, LANDING_WIDTH))
    # A parked marshal's truck beside the outrun: the scenario needs a vehicle.
    end_x, _ = landing.end()
    scenario.add_vehicle(Vehicle("marshal", model="pickup", license="MARSHAL"),
                         pos=(end_x - 40, LANDING_WIDTH / 2 + 15, GROUND_Z + 0.5),
                         rot_quat=angle_to_quat((0, 0, 180)))
    scenario.make(bng)
    return scenario


def place_scenery(bng: BeamNGpy, inrun: Profile, landing: Profile) -> None:
    solid: list[world.Box] = []
    visual: list[world.Box] = []
    flat = (0, 0, 0, 1)
    end_x, _ = landing.end()
    visual.append(("jump_grass", (end_x - START_X + 300, 600, 1.0), "jump_grass_tint",
                   ((START_X + end_x) / 2, 0, GROUND_Z + 0.01 - 0.5), flat))

    # Tower under the start platform.
    plat_end = INRUN[0][1]
    top = START_Z - ROAD_DEPTH - 0.1
    solid.append(("jump_start_tower", (plat_end + 2, INRUN_WIDTH + 2 * STRIPE_WIDTH + 2, top), "jump_tower",
                  (START_X + plat_end / 2, 0, top / 2), flat))

    # Low walls along both in-run edges, following the slope.
    wall_lat = INRUN_WIDTH / 2 + STRIPE_WIDTH + 0.3
    s, n = 0.0, 0
    while s < inrun.s[-1]:
        step = min(WALL_STEP, inrun.s[-1] - s)
        x0, z0, _ = inrun.at_s(s)
        x1, z1, _ = inrun.at_s(s + step)
        length = math.hypot(x1 - x0, z1 - z0)
        pitch = math.atan2(z0 - z1, x1 - x0)
        for side in (-1, 1):
            solid.append((f"jump_wall_{n:03d}_{side + 1}", (length + 0.1, 0.4, WALL_HEIGHT), "jump_tower",
                          ((x0 + x1) / 2, side * wall_lat, (z0 + z1) / 2 + WALL_HEIGHT / 2),
                          world.quat_yaw_pitch(0.0, pitch)))
        s += step
        n += 1

    # Support posts under the in-run and the landing hill (visual only).
    for name, prof, lat, every in (("in", inrun, INRUN_WIDTH / 2 + STRIPE_WIDTH - 0.5, 10.0),
                                   ("land", landing, LANDING_WIDTH / 2 - 1.0, 15.0)):
        s = 0.0
        while s < prof.s[-1]:
            x, z, _ = prof.at_s(s)
            height = z - ROAD_DEPTH
            if height > 1.0 and not (name == "in" and x < START_X + plat_end + 1):
                for side in (-1, 1):
                    visual.append((f"jump_post_{name}_{int(s):04d}_{side + 1}", (0.8, 0.8, height),
                                   "jump_post", (x, side * lat, height / 2), flat))
            s += every

    # Distance lines across the landing hill and signs at both sides.
    for d in marks():
        if d >= landing.s[-1]:
            break
        x, z, angle = landing.at_s(d)
        pitch = math.radians(-angle)
        visual.append((f"jump_line_{d}", (0.6, LANDING_WIDTH, 0.02), "race_white",
                       (x, 0, z + 0.02), world.quat_yaw_pitch(0.0, pitch)))
        for side in (-1, 1):
            y = side * (LANDING_WIDTH / 2 + 3)
            # Two boards back to back. A box shows its texture mirrored on the
            # downhill face (the 200 sign read "002" from below), so the
            # downhill board carries a mirrored copy, which reads the right way.
            visual.append((f"jump_sign_{d}_{side + 1}", (0.15, 4.0, 2.5), f"jump_sign_{d}",
                           (x - 0.08, y, z + 2.2), flat))
            visual.append((f"jump_signback_{d}_{side + 1}", (0.15, 4.0, 2.5), f"jump_sign_{d}_back",
                           (x + 0.08, y, z + 2.2), flat))
            visual.append((f"jump_signpost_{d}_{side + 1}", (0.3, 0.3, 1.0), "jump_post",
                           (x, y, z + 0.5), flat))

    pool = decorate(solid, visual, inrun, landing)

    world.build_boxes(bng, solid, visual, [
        ("jump_grass_tint", "0.55 0.8 0.45 1", "/levels/GridMap/art/shapes/misc/Grass-02-D.dds"),
        ("jump_tower", "0.62 0.62 0.6 1", None),
        ("jump_post", "0.35 0.35 0.38 1", None),
        ("race_white", "0.95 0.95 0.95 1", None),
        ("race_black", "0.03 0.03 0.03 1", None),
        ("jump_seat_red", "0.75 0.12 0.12 1", None),
        ("jump_seat_white", "0.92 0.92 0.92 1", None),
        ("jump_stand", "0.55 0.55 0.58 1", None),
        ("jump_roof", "0.2 0.3 0.45 1", None),
        ("jump_lamp", "1.0 0.97 0.8 1", None),
        ("jump_glass", "0.15 0.3 0.45 1", None),
        ("jump_tiles", "1 1 1 1", f"{ART}/tiles.png"),
        ("jump_pool_floor", "0.2 0.55 0.85 1", f"{ART}/tiles.png"),
        ("jump_rock", "0.47 0.44 0.40 1", None),
        ("jump_hill", "0.36 0.52 0.32 1", None),
    ])
    world.build_cones(bng, mountains(inrun, landing))
    world.add_water(bng, "jump_pool_water", *pool)


def decorate(solid: list, visual: list, inrun: Profile, landing: Profile):
    """Grandstands, pool, judges' tower, light towers, start roof and the
    checkered lip. Returns the pool's water box (centre, size)."""
    flat = (0, 0, 0, 1)
    lip_x, lip_z = inrun.end()
    end_x, _ = landing.end()
    flat_x = next(x for x, z in zip(landing.x, landing.z) if z < 0.05)
    edge = LANDING_WIDTH / 2

    # Grandstands along both sides of the outrun: stepped rows, alternating
    # seat colours, a roof on posts.
    length = min(STAND_LENGTH, end_x - flat_x - 20)
    cx = (flat_x + end_x) / 2
    for side in (-1, 1):
        for row in range(STAND_ROWS):
            lat = side * (edge + STAND_GAP + STAND_DEPTH * (row + 0.5))
            h = STAND_RISE * (row + 1)
            solid.append((f"jump_stand_{side + 1}_{row}", (length, STAND_DEPTH, h), "jump_stand",
                          (cx, lat, h / 2), flat))
            seat = "jump_seat_red" if row % 2 == 0 else "jump_seat_white"
            visual.append((f"jump_seats_{side + 1}_{row}", (length, STAND_DEPTH * 0.6, 0.15), seat,
                           (cx, lat - side * STAND_DEPTH * 0.15, h + 0.08), flat))
        back = side * (edge + STAND_GAP + STAND_DEPTH * STAND_ROWS)
        roof_z = STAND_RISE * STAND_ROWS + 4.0
        visual.append((f"jump_stand_roof_{side + 1}", (length + 2, STAND_DEPTH * STAND_ROWS + 2, 0.4),
                       "jump_roof", (cx, back - side * STAND_DEPTH * STAND_ROWS / 2, roof_z), flat))
        for k in range(5):
            px = cx - length / 2 + length * k / 4
            visual.append((f"jump_stand_post_{side + 1}_{k}", (0.5, 0.5, roof_z), "jump_post",
                           (px, back, roof_z / 2), flat))

    # Pool past the end of the outrun, built above ground (the map's floor is
    # solid), with tiled walls and water inside.
    inner_x, inner_y = POOL_LENGTH, LANDING_WIDTH - 6
    px0 = end_x + 2
    pcx = px0 + inner_x / 2
    wall = POOL_WALL
    # No wall on the outrun side: it stopped cars at the lip instead of letting
    # them drive into the water. The water is its own block and needs no wall.
    for name, size, pos in (
            ("far", (wall, inner_y + 2 * wall, POOL_HEIGHT), (px0 + inner_x + wall / 2, 0, POOL_HEIGHT / 2)),
            ("left", (inner_x, wall, POOL_HEIGHT), (pcx, inner_y / 2 + wall / 2, POOL_HEIGHT / 2)),
            ("right", (inner_x, wall, POOL_HEIGHT), (pcx, -inner_y / 2 - wall / 2, POOL_HEIGHT / 2))):
        solid.append((f"jump_pool_{name}", size, "jump_tiles", pos, flat))
    visual.append(("jump_pool_bottom", (inner_x, inner_y, 0.05), "jump_pool_floor", (pcx, 0, 0.04), flat))
    water = ((pcx, 0, POOL_WATER / 2), (inner_x, inner_y, POOL_WATER))

    # Judges' tower beside the steep part of the landing hill.
    jx, jz, _ = landing.at_s(JUDGES_AT)
    jy = -(edge + 14)
    top = jz + 12
    solid.append(("jump_judges", (8, 8, top), "jump_stand", (jx, jy, top / 2), flat))
    visual.append(("jump_judges_glass", (8.4, 8.4, 3.0), "jump_glass", (jx, jy, top + 1.5), flat))
    visual.append(("jump_judges_roof", (9.5, 9.5, 0.5), "jump_roof", (jx, jy, top + 3.25), flat))

    # Light towers along both sides of the landing hill.
    s = 20.0
    while s < landing.s[-1] - 20:
        x, z, _ = landing.at_s(s)
        for side in (-1, 1):
            y = side * (edge + 7)
            h = z + LIGHT_HEIGHT
            visual.append((f"jump_light_{int(s):03d}_{side + 1}", (0.6, 0.6, h), "jump_post",
                           (x, y, h / 2), flat))
            visual.append((f"jump_lamp_{int(s):03d}_{side + 1}", (1.0, 4.0, 1.2), "jump_lamp",
                           (x, y - side * 1.5, h + 0.6), flat))
        s += LIGHT_EVERY

    # Roof over the start platform.
    plat_x0, plat_x1 = START_X, START_X + INRUN[0][1]
    half = INRUN_WIDTH / 2 + STRIPE_WIDTH + 1
    roof_z = START_Z + 6
    visual.append(("jump_start_roof", (plat_x1 - plat_x0 + 2, 2 * half + 2, 0.5), "jump_roof",
                   ((plat_x0 + plat_x1) / 2, 0, roof_z), flat))
    for k, px in enumerate((plat_x0, plat_x1)):
        for side in (-1, 1):
            visual.append((f"jump_start_post_{k}_{side + 1}", (0.5, 0.5, 6), "jump_post",
                           (px, side * half, START_Z + 3), flat))

    # Checkered band across the takeoff lip (visual: a solid step would trip cars).
    cols = int(INRUN_WIDTH + 2 * STRIPE_WIDTH)
    for row in range(2):
        for col in range(cols):
            mat = "race_white" if (row + col) % 2 == 0 else "race_black"
            visual.append((f"jump_lipcheck_{row}_{col:02d}", (1, 1, 0.01), mat,
                           (lip_x - 1.5 + row, col - (cols - 1) / 2, lip_z + 0.015), flat))
    return water


def mountains(inrun: Profile, landing: Profile) -> list[world.Cone]:
    """A ring of rock and grass cones well away from the course."""
    end_x, _ = landing.end()
    cx = (START_X + end_x) / 2
    rng = random.Random(MOUNTAIN_SEED)  # the same mountains every run
    cones = []
    for k in range(MOUNTAINS):
        a = 2 * math.pi * (k + rng.uniform(-0.3, 0.3)) / MOUNTAINS
        dist = rng.uniform(*MOUNTAIN_DISTANCE)
        r = rng.uniform(*MOUNTAIN_RADIUS)
        h = rng.uniform(*MOUNTAIN_HEIGHT)
        mat = "jump_rock" if k % 3 else "jump_hill"
        # createCone centres the cone on its middle, so lift it by half its height.
        cones.append((f"jump_mountain_{k:02d}", r, h, mat,
                      (cx + dist * math.cos(a), dist * math.sin(a) * 0.8, GROUND_Z + h / 2)))
    return cones


# ---------------------------------------------------------------- jumps

def random_cars(bng: BeamNGpy, count: int) -> list[tuple[str, str, str]]:
    """(model, part config path, name) for `count` random cars and trucks."""
    lua = r"""
local out = {}
for _, c in pairs(core_vehicles.getConfigList().configs or {}) do
  local t = c.aggregates and c.aggregates.Type or {}
  if (t.Car or t.Truck) and c.Power then
    table.insert(out, c.model_key .. '|' .. c.key .. '|' .. (c.Name or c.key))
  end
end
return table.concat(out, '\n')
"""
    rows = [line.split("|", 2) for line in str(bng.control.queue_lua_command(lua, response=True)).splitlines()]
    rows = sorted(r for r in rows if len(r) == 3 and r[0] not in NOT_CARS)
    picks = random.sample(rows, min(count, len(rows)))
    return [(m, f"vehicles/{m}/{k}.pc", name) for m, k, name in picks]


def remove_jump_cars(bng: BeamNGpy) -> None:
    """Delete any jump cars still in the world (the marshal's truck stays)."""
    bng.control.queue_lua_command("""
local doomed = {}
for i = 0, be:getObjectCount() - 1 do
  local v = be:getObject(i)
  if v and string.sub(v:getName() or '', 1, 4) == 'jump' then table.insert(doomed, v) end
end
for _, v in ipairs(doomed) do v:delete() end""")


def _vehicle_lua(car: Vehicle, chunk: str) -> str:
    r = car.queue_lua_command(chunk, response=True)
    return str(r.get("result") if isinstance(r, dict) else r)


def jump(bng: BeamNGpy, n: int, model: str, config: str, name: str,
         inrun: Profile, landing: Profile) -> dict:
    """Run one car down the ramp; return its result."""
    lip_x, lip_z = inrun.end()
    vid = f"jump{n:02d}"
    remove_jump_cars(bng)  # a run that was cut off can leave a car on the platform
    car = Vehicle(vid, model=model, part_config=config, license=f"JUMP {n}")
    start_z, _ = inrun.at_x(SPAWN_X)
    bng.vehicles.spawn(car, pos=(SPAWN_X, 0, start_z + 0.8), rot_quat=world.spawn_rotation(0.0))
    world.install_camera(bng, [vid], [])
    # The game's own orbit camera on the in-run: it is attached to the car and
    # follows the slope. A free chase camera on a flat offset sat inside the
    # 30 degree ramp behind the car. raceCam takes over again at the lip.
    bng.control.queue_lua_command("raceCam.stop()")
    bng.vehicles.switch(car)
    bng.camera.set_player_mode(car, "orbit", {})
    time.sleep(2.0)  # let it settle on the platform
    bng.ui.display_message(f"Jump {n}: {name}")
    print(f"Jump {n}: {name}")
    # Full throttle: spawned cars ignore direct throttle input (control() showed
    # throttle 1 in gear and 0 m/s), so the AI drives, told to hold an
    # unreachable speed. Its path runs on past the lip, or it would brake for
    # the end of the path; it is switched off at the lip.
    nodes = []
    x = SPAWN_X + AI_FIRST_NODE
    while x < lip_x + 150:
        z = (inrun if x <= lip_x else landing).at_x(x)[0]
        nodes.append(f"{{x={x:.1f}, y=0, z={z + 0.05:.2f}, r=2}}")
        x += 6
    car.queue_lua_command(
        'input.event("parkingbrake", 0, 1) '
        f"ai.driveUsingPath{{script = {{{', '.join(nodes)}}}, avoidCars = 'off', aggression = 1, "
        f"routeSpeed = {FULL_THROTTLE_SPEED}, routeSpeedMode = 'set'}}")

    lip_spot = (lip_x + 12, 24.0, lip_z + 2)
    # Last part: a free camera at eye level on the ground beside the outrun,
    # looking up the hill as the car comes down and lands.
    flat_x = next(lx for lx, lz in zip(landing.x, landing.z) if lz < 0.05)
    orbit_again_x = landing.at_s(ORBIT_AGAIN_AT)[0]
    land_spot = (flat_x + GROUND_CAM_AHEAD, LANDING_WIDTH / 2 + GROUND_CAM_SIDE,
                 GROUND_Z + GROUND_CAM_HEIGHT)
    phase = "inrun"
    airborne = False
    distance = None
    top_speed = 0.0
    history: deque = deque()
    still_since = None
    t0 = time.monotonic()
    while time.monotonic() - t0 < RUN_TIMEOUT:
        now = time.monotonic() - t0
        positions, _ = car_positions(bng)
        if vid not in positions:
            time.sleep(0.05)
            continue
        x, y, z = positions[vid]
        # Speed over at least SPEED_WINDOW seconds: frames reach us unevenly, and
        # a late sample followed by an early one read as nearly 500 km/h.
        history.append((now, (x, y, z)))
        while len(history) > 2 and now - history[1][0] >= SPEED_WINDOW:
            history.popleft()
        then, then_pos = history[0]
        speed = math.dist(then_pos, (x, y, z)) / (now - then) if now - then >= SPEED_WINDOW else 0.0
        if phase in ("inrun", "lip") and speed < MAX_BELIEVABLE_SPEED:
            top_speed = max(top_speed, speed)

        if phase == "inrun" and x > lip_x - 50:
            phase = "lip"
            bng.control.queue_lua_command(
                f"raceCam.setSpot({lip_spot[0]:.1f}, {lip_spot[1]:.1f}, {lip_spot[2]:.1f}, '{vid}')")
        if phase == "lip" and x > lip_x:
            phase = "flight"
            car.ai.set_mode("disabled")
        if phase == "flight" and x > lip_x + GROUND_CAM_AFTER:
            phase = "landing"
            bng.control.queue_lua_command(
                f"raceCam.setSpot({land_spot[0]:.1f}, {land_spot[1]:.1f}, {land_spot[2]:.1f}, '{vid}')")
        if phase == "landing" and x > orbit_again_x:  # back on the car's own orbit camera
            phase = "rollout"
            bng.control.queue_lua_command("raceCam.stop()")
            bng.camera.set_player_mode(car, "orbit", {})

        if x > lip_x and distance is None:
            surface, along = landing.at_x(x)
            height = z - surface
            if height > AIRBORNE:
                airborne = True
            elif airborne and height < TOUCHDOWN:
                distance = along
                bng.ui.display_message(f"{name}: {distance:.1f} m")
                print(f"  lands at {distance:.1f} m")

        if x > lip_x and speed < STOPPED_SPEED:
            still_since = still_since if still_since is not None else now
            if now - still_since > STOPPED_FOR:
                break
        else:
            still_since = None
        time.sleep(0.05)

    up = float(_vehicle_lua(car, "return tostring(obj:getDirectionVectorUp().z)") or 0)
    damage = float(_vehicle_lua(car, "return tostring(beamstate.damage or 0)") or 0)
    park(car)
    time.sleep(1.0)
    bng.vehicles.despawn(car)
    result = {"name": name, "distance": distance, "upright": up > 0.5,
              "damage": damage, "top_speed": top_speed}
    print(f"  {'upright' if result['upright'] else 'wrecked'}, damage {damage:,.0f}, "
          f"top speed {top_speed * 3.6:.0f} km/h")
    return result


def scoreboard(results: list[dict]) -> str:
    ranked = sorted(results, key=lambda r: -(r["distance"] or -1))
    lines = []
    for i, r in enumerate(ranked, 1):
        dist = f"{r['distance']:6.1f} m" if r["distance"] is not None else "  no jump"
        lines.append(f"{i:2d}. {dist}  {r['name']}  ({'upright' if r['upright'] else 'wrecked'})")
    return "\n".join(lines)


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--host", default=default_host(),
                   help="Host where BeamNG listens (default: env BNG_HOST, else found automatically)")
    p.add_argument("--port", type=int, default=int(os.environ.get("BNG_PORT", "25252")))
    p.add_argument("--no-launch", action="store_true",
                   help="Accepted to match beamng-race; this always connects to a running game")
    p.add_argument("--jumps", type=int, default=10, help="How many random cars jump")
    p.add_argument("--seed", type=int, default=None, help="Random seed for the car picks")
    return p.parse_args()


def main() -> None:
    args = parse_args()
    random.seed(args.seed)
    inrun, landing = build_course()

    bng = BeamNGpy(args.host, args.port)
    bng.open(launch=False)
    try:
        install_art(bng)
        scenario = build_scenario(bng, inrun, landing)
        bng.scenario.load(scenario)
        bng.scenario.start()
        # Otherwise the game camera sits on the marshal's truck, under the hill.
        bng.camera.set_free(*OVERVIEW_CAMERA)
        place_scenery(bng, inrun, landing)
        cars = random_cars(bng, args.jumps)
        print("Today's jumpers:", ", ".join(name for _, _, name in cars))
        results = []
        for n, (model, config, name) in enumerate(cars, 1):
            try:
                results.append(jump(bng, n, model, config, name, inrun, landing))
            except (BNGError, ValueError) as exc:  # one broken car must not end the show
                print(f"  jump {n} ({name}) failed: {exc}")
        board = scoreboard(results)
        print("\nScoreboard\n" + board)
        best = next((r for r in sorted(results, key=lambda r: -(r["distance"] or -1))), None)
        if best and best["distance"] is not None:
            bng.ui.display_message(f"Longest jump: {best['name']}, {best['distance']:.1f} m")
        bng.control.queue_lua_command("raceCam.stop()")
    finally:
        bng.disconnect()
