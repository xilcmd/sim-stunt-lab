"""Big jump: random cars, one at a time, launch off a giant ski-jump ramp.

Inspired by DestructionNation's "Big Ramp Jumps with Random Cars" video: a
tall in-run with hazard-striped edges, a takeoff lip, and a long steep landing
hill with distance markers. Each car drives full throttle down the in-run; the
jump is measured along the landing hill from the knoll, like ski jumping.
"""

import argparse
import json
import math
import os
import random
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path

from beamngpy import BeamNGpy, MeshRoad, Scenario, Vehicle
from beamngpy.logging import BNGError

from beamng_demo import default_host, world
from beamng_demo.world import car_positions, park

GROUND_Z = 0.0  # course coordinates: the outrun surface is at z = 0


@dataclass(frozen=True)
class Site:
    """Where the course stands. Course coordinates are u along the course
    (toward the outrun), v to its left, z above the outrun; a site turns them
    into map coordinates."""
    level: str
    x: float
    y: float
    heading: float      # degrees, direction of travel; 0 = map +x
    base_z: float       # map height of the outrun surface
    flat_floor: bool    # empty map: add a grass slab and cone mountains
    # "pool": a pool on the end of the outrun. "ocean": no pool, the outrun just
    # ends above the sea, and cars that do not stop drop into it.
    finish: str = "pool"
    sea_z: float | None = None  # map height of the sea (needed for "ocean")

    @property
    def yaw(self) -> float:
        return math.radians(self.heading)

    @property
    def art(self) -> str:
        return f"/levels/{self.level}/art/bigjump"  # game path; files go in the user folder

    def world(self, u: float, v: float, z: float = 0.0) -> tuple[float, float, float]:
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return (self.x + u * c - v * s, self.y + u * s + v * c, self.base_z + z)

    def local(self, x: float, y: float, z: float) -> tuple[float, float, float]:
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        dx, dy = x - self.x, y - self.y
        return dx * c + dy * s, -dx * s + dy * c, z - self.base_z

    def direction(self, du: float, dv: float, dz: float) -> tuple[float, float, float]:
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return du * c - dv * s, du * s + dv * c, dz

    def rot(self, pitch: float = 0.0) -> tuple[float, float, float, float]:
        """Box rotation along the course, tipped down by `pitch` radians."""
        return world.quat_yaw_pitch(self.yaw, pitch)


SITES = {
    "smallgrid": Site("smallgrid", 0.0, 0.0, 0.0, 0.0, flat_floor=True),
    # Found by scoring 74,501 placements on a 32 m height grid (course close
    # above the hillside, sea ahead of the finish), then raised 3 m after a
    # 4 m check in the game found something taller under the outrun's edge.
    "italy": Site("italy", -576.0, 832.0, 30.0, 186.4, flat_floor=False),
}
SITES_FILE = Path(__file__).with_name("sites.json")  # spots found by beamng-bigjump-scout


def load_sites() -> dict[str, Site]:
    sites = dict(SITES)
    if SITES_FILE.exists():
        for name, d in json.loads(SITES_FILE.read_text()).items():
            sites[name] = Site(d["level"], d["x"], d["y"], d["heading"], d["base_z"], flat_floor=False,
                               finish=d.get("finish", "pool"), sea_z=d.get("sea_z"))
    return sites

# Profiles are built from exact straight lines and circular arcs, in the u-z
# plane along v = 0, toward +u. ("line", length, angle) and
# ("arc", radius, from angle, to angle); angles in degrees, positive is uphill.
# With a +10 degree lip and a 125 m start, test cars flew about 180 m, past the
# end of the steep landing slope, and landed on the flat. A level lip keeps the
# flight closer to the slope, and the higher start makes the landing hill
# longer without changing the in-run drop.
# The in-run sits LIP_GAP metres back from the landing hill: an open gap
# between the lip and the knoll. The landing hill, stands and pool stay put.
LIP_GAP = 75.0
# The in-run is INRUN_RAISE higher than the landing needs: moved back, it cut
# into the rising Italy hillside. 21 m leaves 1.7 m of clearance at a 75 m gap.
# KNOLL_DROP grows by the same amount, so the landing hill, stands and pool keep
# their old heights. A 100 m gap needed 28 m and was too harsh: most cars landed
# within 30 m of the knoll and a truck fell into the gap.
INRUN_RAISE = 21.0
# The 30 degree in-run is INRUN_EXTRA metres longer than the first design. The
# start moves back and up to match, so the lip stays put.
INRUN_EXTRA = 50.0
START_X = -15.0 - LIP_GAP - INRUN_EXTRA * math.cos(math.radians(30))
START_Z = 150.0 + INRUN_RAISE + INRUN_EXTRA * math.sin(math.radians(30))
INRUN = [
    ("line", 27, 0),            # start platform (long enough for a truck)
    ("arc", 40, 0, -30),
    ("line", 110 + INRUN_EXTRA, -30),  # the in-run
    ("arc", 100, -30, 0),       # transition into the lip
    ("line", 6, 0),             # takeoff lip, level
]
KNOLL_DROP = 4.0 + INRUN_RAISE  # landing hill starts this far below the lip
POOL_LENGTH, POOL_WALL, POOL_HEIGHT, POOL_WATER = 34.0, 0.8, 1.6, 1.3
# Where the pool starts, in metres after the outrun turns flat. Wrecked test cars
# came to rest 105-122 m along the flat; with the pool at 267 m none reached it.
# A "winning" run is a wreck that still limps into the water.
POOL_AFTER_FLAT = 90.0
OUTRUN_LENGTH = POOL_AFTER_FLAT + POOL_LENGTH + 2.0  # the pool sits on the end of the outrun
LANDING = [
    ("line", 15, -5),           # knoll
    ("arc", 120, -5, -33),
    ("line", None, -33),        # length set so the outrun lands on the ground
    ("arc", 180, -33, 0),
    ("line", OUTRUN_LENGTH, 0), # outrun; the pool sits on its last 36 m
]
INRUN_WIDTH = 8.0
STRIPE_WIDTH = 1.0              # hazard stripes along both in-run edges
LANDING_WIDTH = 40.0
ROAD_DEPTH = 1.5
NODE_STEP = 3.0                 # metres between mesh road nodes
WALL_HEIGHT = 1.0
WALL_STEP = 6.0
MARK_FIRST, MARK_EVERY, MARK_LAST = 50, 25, 300  # distance lines and signs, metres

SPAWN_X = START_X + 15.0        # on the start platform
FULL_THROTTLE_SPEED = 100.0     # m/s the AI is told to hold: more than any car reaches
AI_FIRST_NODE = 12.0            # metres ahead of the car; at 6 m the car passed it at
                                # once and the AI backed up and stopped
OVERVIEW_CAMERA = ((90.0, 260.0, 130.0), (0.05, -0.92, -0.38))  # side view of the course
# Cameras: the car's orbit camera on the in-run, a free camera beside the lip
# for the takeoff and flight, then the orbit camera again from ORBIT_AGAIN_AT
# metres down the hill. A ground camera beside the outrun was dropped: with the
# shorter outrun the stands and their roof hid the car.
LIP_CAM_UP = 6.0                # lip camera height above the lip
LIP_CAM_AHEAD = LIP_GAP + 12.0            # ...and how far past the lip it stands (at 12 m, over the gap,
                                # the landing hill hid the landing)
ORBIT_AGAIN_AT = 100.0          # metres down the landing hill: back to the orbit camera
TREE_STEP = 8.0                 # metres between tree-clearing circles along the course
TREE_CLEAR_INRUN = 15.0         # circle radius along the in-run (its rails included)
TREE_CLEAR_LANDING = 45.0       # ...and along the landing hill and outrun (stands included)
MAX_BELIEVABLE_SPEED = 150.0    # m/s; faster readings are position glitches
SPEED_WINDOW = 0.5              # seconds to average speed over
MS_TO_MPH = 2.236936            # metres per second to miles per hour
AIRBORNE = 3.0                  # metres above the surface that counts as flying
TOUCHDOWN = 1.6                 # ... and back below this counts as landing
RUN_TIMEOUT = 35.0
STOPPED_SPEED = 0.7
STOPPED_FOR = 2.5
NOT_CARS = {"unicycle"}         # models that are not drivable cars

TEXTURE_SIZE = 256

# Decorations
STAND_LENGTH, STAND_ROWS, STAND_DEPTH, STAND_RISE, STAND_GAP = 140.0, 6, 2.6, 1.3, 6.0
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
    probe = Profile(lip_x + LIP_GAP, start_z, [s if s[1] is not None else ("line", 0, s[2]) for s in LANDING])
    drop_left = probe.end()[1] - (GROUND_Z + 0.02)
    steep = next(s[2] for s in LANDING if s[1] is None)
    length = drop_left / -math.sin(math.radians(steep))
    if length < 0:
        raise ValueError("START_Z is too low for this landing hill")
    landing = Profile(lip_x + LIP_GAP, start_z, [s if s[1] is not None else ("line", length, s[2])
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


def install_art(bng: BeamNGpy, site: Site) -> None:
    textures = {"hazard.png": _hazard, "concrete.png": _concrete, "tiles.png": _tiles}
    materials = {"jump_hazard": "hazard.png", "jump_concrete": "concrete.png"}
    for d in marks():
        textures[f"sign_{d}.png"] = _sign(d)
        materials[f"jump_sign_{d}"] = f"sign_{d}.png"
        textures[f"sign_{d}_back.png"] = _mirrored(_sign(d))
        materials[f"jump_sign_{d}_back"] = f"sign_{d}_back.png"
    world.install_art(bng, site.art, textures, materials, TEXTURE_SIZE)


# ---------------------------------------------------------------- scene

def _road(site: Site, rid: str, material: str, profile: Profile, lateral: float,
          width: float) -> MeshRoad:
    road = MeshRoad(material, side_material="jump_concrete", rid=rid,
                    default_width=width, default_depth=ROAD_DEPTH, texture_length=8)
    stations = [profile.at_s(s)[:2] for s in _frange(0.0, profile.s[-1], NODE_STEP)]
    stations.append(profile.end())
    road.add_nodes(*[(*site.world(u, lateral, z), width, ROAD_DEPTH) for u, z in stations])
    return road


def _frange(start: float, stop: float, step: float):
    v = start
    while v < stop:
        yield v
        v += step


def build_scenario(bng: BeamNGpy, site: Site, inrun: Profile, landing: Profile) -> Scenario:
    scenario = Scenario(site.level, "big_jump")
    scenario.add_mesh_road(_road(site, "jump_inrun", "track_editor_C_center", inrun, 0.0, INRUN_WIDTH))
    edge = INRUN_WIDTH / 2 + STRIPE_WIDTH / 2
    for side in (-1, 1):
        scenario.add_mesh_road(_road(site, f"jump_stripe_{side + 1}", "jump_hazard", inrun,
                                     side * edge, STRIPE_WIDTH))
    scenario.add_mesh_road(_road(site, "jump_landing", "jump_concrete", landing, 0.0, LANDING_WIDTH))
    # The scenario needs a vehicle: a marshal's truck parked at the back of the
    # start platform, which is clear of the ground on any map.
    scenario.add_vehicle(Vehicle("marshal", model="pickup", license="MARSHAL"),
                         pos=site.world(START_X + 3, 0, START_Z + 0.6),
                         rot_quat=world.spawn_rotation(site.yaw))
    scenario.make(bng)
    return scenario


def clear_trees(bng: BeamNGpy, site: Site, inrun: Profile, landing: Profile) -> int:
    """Remove the map's trees along the course; returns how many.

    Trees live in the map's Forest object, and castRayStatic passes through
    them, so the height fit could not see them and they poked up through the
    track. Circles every TREE_STEP metres along the course cover its width
    plus a margin (the stands beside the outrun included)."""
    lip_x, end_x = inrun.end()[0], landing.end()[0]
    circles = []
    for u in _frange(START_X - 10, end_x + 10, TREE_STEP):
        x, y, _ = site.world(u, 0)
        circles.append((x, y, TREE_CLEAR_INRUN if u <= lip_x else TREE_CLEAR_LANDING))
    calls = "\n".join(
        f"do local c = vec3({x:.1f}, {y:.1f}, core_terrain.getTerrainHeight(vec3({x:.1f}, {y:.1f}, 0)) or 0) "
        f"for _, it in ipairs(d:getItemsCircle(c, {r:.1f}) or {{}}) do d:removeItem(it) n = n + 1 end end"
        for x, y, r in circles)
    lua = f"""
local f = core_forest and core_forest.getForestObject and core_forest.getForestObject()
if not f then return '0' end
local d, n = f:getData(), 0
{calls}
return tostring(n)"""
    return int(float(str(bng.control.queue_lua_command(lua, response=True)) or 0))


def ground_heights(bng: BeamNGpy, points: list[tuple[float, float, float]]) -> list[float]:
    """Map height of whatever is below each (x, y, from_z) point: terrain, a
    building, or our own road. Rays start at from_z, so they miss anything above."""
    calls = " ".join(
        f"do local d = castRayStatic(vec3({x:.2f}, {y:.2f}, {z:.2f}), vec3(0, 0, -1), 2000) "
        f"table.insert(o, d and string.format('%.2f', {z:.2f} - d) or 'x') end"
        for x, y, z in points)
    raw = str(bng.control.queue_lua_command(f"local o = {{}} {calls} return table.concat(o, ' ')",
                                            response=True)).split()
    return [float(h) if h != "x" else z - 100.0 for h, (_, _, z) in zip(raw, points)]


# A box that stands on the ground: (name, (size u, size v), material, (u, v), top z).
Pillar = tuple[str, tuple[float, float], str, tuple[float, float], float]


def stand_on_ground(bng: BeamNGpy, site: Site, pillars: list[Pillar]) -> list[world.Box]:
    """Turn pillars into boxes that reach down from their top to the ground
    below them, so they stand on terrain on a real map (and on the floor of the
    empty one)."""
    tops = [site.world(u, v, top - 0.05) for _, _, _, (u, v), top in pillars]
    grounds = ground_heights(bng, tops)
    boxes = []
    for (name, (su, sv), mat, (u, v), top), g in zip(pillars, grounds):
        x, y, z_top = site.world(u, v, top)
        bottom = g - 0.5  # a little into the ground, so no gap shows
        if z_top - bottom < 0.6:  # the ground is at or above its top: nothing to build
            continue
        boxes.append((name, (su, sv, z_top - bottom), mat, (x, y, (z_top + bottom) / 2), site.rot()))
    return boxes


def place_scenery(bng: BeamNGpy, site: Site, inrun: Profile, landing: Profile) -> None:
    solid: list[world.Box] = []
    visual: list[world.Box] = []
    solid_pillars: list[Pillar] = []
    visual_pillars: list[Pillar] = []
    end_x, _ = landing.end()
    if site.flat_floor:
        visual.append(("jump_grass", (end_x - START_X + 300, 600, 1.0), "jump_grass_tint",
                       site.world((START_X + end_x) / 2, 0, 0.01 - 0.5), site.rot()))

    # Tower under the start platform.
    plat_end = INRUN[0][1]
    solid_pillars.append(("jump_start_tower", (plat_end + 2, INRUN_WIDTH + 2 * STRIPE_WIDTH + 2),
                          "jump_tower", (START_X + plat_end / 2, 0), START_Z - ROAD_DEPTH - 0.1))

    # Low walls along both in-run edges, following the slope.
    wall_lat = INRUN_WIDTH / 2 + STRIPE_WIDTH + 0.3
    for n, s in enumerate(_frange(0.0, inrun.s[-1], WALL_STEP)):
        step = min(WALL_STEP, inrun.s[-1] - s)
        x0, z0, _ = inrun.at_s(s)
        x1, z1, _ = inrun.at_s(s + step)
        length = math.hypot(x1 - x0, z1 - z0)
        pitch = math.atan2(z0 - z1, x1 - x0)
        for side in (-1, 1):
            solid.append((f"jump_wall_{n:03d}_{side + 1}", (length + 0.1, 0.4, WALL_HEIGHT), "jump_tower",
                          site.world((x0 + x1) / 2, side * wall_lat, (z0 + z1) / 2 + WALL_HEIGHT / 2),
                          site.rot(pitch)))

    # Support posts under the in-run and the landing hill, down to the ground.
    for name, prof, lat, every in (("in", inrun, INRUN_WIDTH / 2 + STRIPE_WIDTH - 0.5, 10.0),
                                   ("land", landing, LANDING_WIDTH / 2 - 1.0, 15.0)):
        for s in _frange(0.0, prof.s[-1], every):
            x, z, _ = prof.at_s(s)
            if name == "in" and x < START_X + plat_end + 1:
                continue
            for side in (-1, 1):
                visual_pillars.append((f"jump_post_{name}_{int(s):04d}_{side + 1}", (0.8, 0.8), "jump_post",
                                       (x, side * lat), z - ROAD_DEPTH))

    # Distance lines across the landing hill and signs at both sides.
    for d in marks():
        if d >= landing.s[-1]:
            break
        x, z, angle = landing.at_s(d)
        visual.append((f"jump_line_{d}", (0.6, LANDING_WIDTH, 0.02), "race_white",
                       site.world(x, 0, z + 0.02), site.rot(math.radians(-angle))))
        for side in (-1, 1):
            y = side * (LANDING_WIDTH / 2 + 3)
            # Two boards back to back. A box shows its texture mirrored on the
            # downhill face (the 200 sign read "002" from below), so the
            # downhill board carries a mirrored copy, which reads the right way.
            visual.append((f"jump_sign_{d}_{side + 1}", (0.15, 4.0, 2.5), f"jump_sign_{d}",
                           site.world(x - 0.08, y, z + 2.2), site.rot()))
            visual.append((f"jump_signback_{d}_{side + 1}", (0.15, 4.0, 2.5), f"jump_sign_{d}_back",
                           site.world(x + 0.08, y, z + 2.2), site.rot()))
            visual_pillars.append((f"jump_signpost_{d}_{side + 1}", (0.3, 0.3), "jump_post",
                                   (x, y), z + 0.95))

    water = decorate(solid, visual, solid_pillars, visual_pillars, site, inrun, landing)

    solid += stand_on_ground(bng, site, solid_pillars)
    visual += stand_on_ground(bng, site, visual_pillars)
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
        ("jump_tiles", "1 1 1 1", f"{site.art}/tiles.png"),
        ("jump_pool_floor", "0.2 0.55 0.85 1", f"{site.art}/tiles.png"),
        ("jump_rock", "0.47 0.44 0.40 1", None),
        ("jump_hill", "0.36 0.52 0.32 1", None),
    ])
    if site.flat_floor:  # a real map has real mountains
        world.build_cones(bng, mountains(site, landing))
    if water:
        world.add_water(bng, "jump_pool_water", *water)


def pool_extent(landing: Profile) -> tuple[float, float, float]:
    """The pool's water, in course coordinates: (near u, far u, half width)."""
    near = landing.end()[0] - POOL_LENGTH - 2
    return near, near + POOL_LENGTH, (LANDING_WIDTH - 6) / 2


def pool_result(landing: Profile, u: float, v: float) -> str:
    """Where a car came to rest: 'pool' (in the water: the win), 'overshot'
    (past the far wall), 'short', or 'off the side'."""
    near, far, half = pool_extent(landing)
    if abs(v) > LANDING_WIDTH / 2 + 1:
        return "off the side"
    if u < near:
        return "short"
    if u > far + POOL_WALL:
        return "overshot"
    return "pool" if abs(v) <= half else "off the side"


def ocean_result(site: Site, landing: Profile, u: float, v: float, z: float) -> str:
    """Where a car came to rest on an ocean finish: 'ocean' (in the sea: the
    win), 'rocks' (off the end, but on the shore), 'stopped' (on the track),
    or 'off the side'."""
    if z < site.sea_z - site.base_z + 1.0:
        return "ocean"
    if abs(v) > LANDING_WIDTH / 2 + 1:
        return "off the side"
    return "rocks" if u > landing.end()[0] else "stopped"


def decorate(solid: list, visual: list, solid_pillars: list, visual_pillars: list,
             site: Site, inrun: Profile, landing: Profile):
    """Grandstands, pool, judges' tower, light towers, start roof and the
    checkered lip. Returns the pool's water block (centre, size, heading), or
    None on an ocean finish (no pool: the outrun is plain track to its end)."""
    lip_x, lip_z = inrun.end()
    flat_x = next(x for x, z in zip(landing.x, landing.z) if z < 0.05)
    edge = LANDING_WIDTH / 2

    # Pool on the last part of the outrun, so the road is its floor on any map;
    # no wall on the outrun side, so cars can drive into the water.
    px0, _, half_y = pool_extent(landing)
    water = None
    if site.finish == "pool":
        inner_x, inner_y = POOL_LENGTH, 2 * half_y
        pcx = px0 + inner_x / 2
        wall = POOL_WALL
        for name, size, (u, v) in (
                ("far", (wall, inner_y + 2 * wall), (px0 + inner_x + wall / 2, 0)),
                ("left", (inner_x, wall), (pcx, inner_y / 2 + wall / 2)),
                ("right", (inner_x, wall), (pcx, -inner_y / 2 - wall / 2))):
            solid_pillars.append((f"jump_pool_{name}", size, "jump_tiles", (u, v), POOL_HEIGHT))
        visual.append(("jump_pool_bottom", (inner_x, inner_y, 0.05), "jump_pool_floor",
                       site.world(pcx, 0, 0.04), site.rot()))
        water = (site.world(pcx, 0, POOL_WATER / 2), (inner_x, inner_y, POOL_WATER), site.heading)

    # Grandstands along both sides of the outrun, before the pool: stepped rows,
    # alternating seat colours, a roof on posts.
    length = min(STAND_LENGTH, px0 - flat_x - 10)
    cx = (flat_x + px0) / 2
    for side in (-1, 1):
        for row in range(STAND_ROWS):
            lat = side * (edge + STAND_GAP + STAND_DEPTH * (row + 0.5))
            h = STAND_RISE * (row + 1)
            solid_pillars.append((f"jump_stand_{side + 1}_{row}", (length, STAND_DEPTH), "jump_stand",
                                  (cx, lat), h))
            seat = "jump_seat_red" if row % 2 == 0 else "jump_seat_white"
            visual.append((f"jump_seats_{side + 1}_{row}", (length, STAND_DEPTH * 0.6, 0.15), seat,
                           site.world(cx, lat - side * STAND_DEPTH * 0.15, h + 0.08), site.rot()))
        back = side * (edge + STAND_GAP + STAND_DEPTH * STAND_ROWS)
        roof_z = STAND_RISE * STAND_ROWS + 4.0
        visual.append((f"jump_stand_roof_{side + 1}", (length + 2, STAND_DEPTH * STAND_ROWS + 2, 0.4),
                       "jump_roof", site.world(cx, back - side * STAND_DEPTH * STAND_ROWS / 2, roof_z),
                       site.rot()))
        for k in range(5):
            visual_pillars.append((f"jump_stand_post_{side + 1}_{k}", (0.5, 0.5), "jump_post",
                                   (cx - length / 2 + length * k / 4, back), roof_z))

    # Judges' tower beside the steep part of the landing hill.
    jx, jz, _ = landing.at_s(JUDGES_AT)
    jy = -(edge + 14)
    top = jz + 12
    solid_pillars.append(("jump_judges", (8, 8), "jump_stand", (jx, jy), top))
    visual.append(("jump_judges_glass", (8.4, 8.4, 3.0), "jump_glass", site.world(jx, jy, top + 1.5), site.rot()))
    visual.append(("jump_judges_roof", (9.5, 9.5, 0.5), "jump_roof", site.world(jx, jy, top + 3.25), site.rot()))

    # Light towers along both sides of the landing hill.
    for s in _frange(20.0, landing.s[-1] - 20, LIGHT_EVERY):
        x, z, _ = landing.at_s(s)
        for side in (-1, 1):
            y = side * (edge + 7)
            h = z + LIGHT_HEIGHT
            visual_pillars.append((f"jump_light_{int(s):03d}_{side + 1}", (0.6, 0.6), "jump_post", (x, y), h))
            visual.append((f"jump_lamp_{int(s):03d}_{side + 1}", (1.0, 4.0, 1.2), "jump_lamp",
                           site.world(x, y - side * 1.5, h + 0.6), site.rot()))

    # Roof over the start platform.
    plat_x0, plat_x1 = START_X, START_X + INRUN[0][1]
    half = INRUN_WIDTH / 2 + STRIPE_WIDTH + 1
    roof_z = START_Z + 6
    visual.append(("jump_start_roof", (plat_x1 - plat_x0 + 2, 2 * half + 2, 0.5), "jump_roof",
                   site.world((plat_x0 + plat_x1) / 2, 0, roof_z), site.rot()))
    for k, px in enumerate((plat_x0, plat_x1)):
        for side in (-1, 1):
            visual.append((f"jump_start_post_{k}_{side + 1}", (0.5, 0.5, 6), "jump_post",
                           site.world(px, side * half, START_Z + 3), site.rot()))

    # Checkered band across the takeoff lip (visual: a solid step would trip cars).
    cols = int(INRUN_WIDTH + 2 * STRIPE_WIDTH)
    for row in range(2):
        for col in range(cols):
            mat = "race_white" if (row + col) % 2 == 0 else "race_black"
            visual.append((f"jump_lipcheck_{row}_{col:02d}", (1, 1, 0.01), mat,
                           site.world(lip_x - 1.5 + row, col - (cols - 1) / 2, lip_z + 0.015), site.rot()))
    return water


def mountains(site: Site, landing: Profile) -> list[world.Cone]:
    """A ring of rock and grass cones well away from the course (empty map only)."""
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
                      site.world(cx + dist * math.cos(a), dist * math.sin(a) * 0.8, h / 2)))
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


def jump(bng: BeamNGpy, site: Site, n: int, model: str, config: str, name: str,
         inrun: Profile, landing: Profile) -> dict:
    """Run one car down the ramp; return its result."""
    lip_x, lip_z = inrun.end()
    vid = f"jump{n:02d}"
    remove_jump_cars(bng)  # a run that was cut off can leave a car on the platform
    car = Vehicle(vid, model=model, part_config=config, license=f"JUMP {n}")
    start_z, _ = inrun.at_x(SPAWN_X)
    bng.vehicles.spawn(car, pos=site.world(SPAWN_X, 0, start_z + 0.8), rot_quat=world.spawn_rotation(site.yaw))
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
        wx, wy, wz = site.world(x, 0, z + 0.05)
        nodes.append(f"{{x={wx:.2f}, y={wy:.2f}, z={wz:.2f}, r=2}}")
        x += 6
    car.queue_lua_command(
        'input.event("parkingbrake", 0, 1) '
        f"ai.driveUsingPath{{script = {{{', '.join(nodes)}}}, avoidCars = 'off', aggression = 1, "
        f"routeSpeed = {FULL_THROTTLE_SPEED}, routeSpeedMode = 'set'}}")

    lip_spot = site.world(lip_x + LIP_CAM_AHEAD, 24.0, lip_z + LIP_CAM_UP)
    orbit_again_x = landing.at_s(ORBIT_AGAIN_AT)[0]
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
        x, y, z = site.local(*positions[vid])  # course coordinates from here on
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
        if phase == "flight" and x > orbit_again_x:  # back on the car's own orbit camera
            phase = "rollout"
            bng.control.queue_lua_command("raceCam.stop()")
            bng.camera.set_player_mode(car, "orbit", {})

        if x >= landing.x[0] and distance is None:  # a car that drops into the gap gets no distance
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
    positions, _ = car_positions(bng)
    u, v, z = site.local(*positions[vid]) if vid in positions else (0.0, 0.0, 0.0)
    finish = ocean_result(site, landing, u, v, z) if site.finish == "ocean" else pool_result(landing, u, v)
    if finish in WINS:
        bng.ui.display_message(f"SPLASH! {name} made it into the {finish}")
    park(car)
    time.sleep(2.0 if finish in WINS else 1.0)
    bng.vehicles.despawn(car)
    result = {"name": name, "distance": distance, "upright": up > 0.5,
              "damage": damage, "top_speed": top_speed, "finish": finish}
    print(f"  {'upright' if result['upright'] else 'wrecked'}, damage {damage:,.0f}, "
          f"top speed {top_speed * MS_TO_MPH:.0f} mph, ended: {FINISH_LABELS[finish]}")
    return result


FINISH_LABELS = {"pool": "IN THE POOL", "overshot": "overshot the pool",
                 "short": "short of the pool", "off the side": "off the side",
                 "ocean": "IN THE OCEAN", "rocks": "on the rocks", "stopped": "stopped on track"}
WINS = ("pool", "ocean")


def scoreboard(results: list[dict]) -> str:
    ranked = sorted(results, key=lambda r: -(r["distance"] or -1))
    lines = []
    for i, r in enumerate(ranked, 1):
        dist = f"{r['distance']:6.1f} m" if r["distance"] is not None else "  no jump"
        state = "upright" if r["upright"] else "wrecked"
        lines.append(f"{i:2d}. {dist}  {FINISH_LABELS[r['finish']]:17s}  {r['name']}  ({state})")
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
    p.add_argument("--type", "--map", dest="site", choices=sorted(load_sites()), default="italy",
                   help="Setting: where the course stands (default: a hillside in Italy above "
                        "the sea). Add more with beamng-bigjump-scout <map>")
    return p.parse_args()


def main() -> None:
    args = parse_args()
    random.seed(args.seed)
    inrun, landing = build_course()
    site = load_sites()[args.site]

    bng = BeamNGpy(args.host, args.port)
    bng.open(launch=False)
    try:
        install_art(bng, site)
        scenario = build_scenario(bng, site, inrun, landing)
        bng.scenario.load(scenario)
        bng.scenario.start()
        # A pause left by any earlier script survives loading a scenario; a paused
        # car only gets placed by the AI and never moves.
        bng.control.resume()
        # The marshal's truck was only needed to load the scenario; it showed up
        # in the first shot of every run, parked across the start platform.
        bng.control.queue_lua_command(
            "local m = scenetree.findObject('marshal') if m then m:delete() end")
        # A side view of the whole course while it is built.
        pos, aim = OVERVIEW_CAMERA
        bng.camera.set_free(site.world(*pos), site.direction(*aim))
        if not site.flat_floor:
            print(f"Cleared {clear_trees(bng, site, inrun, landing)} trees along the course")
        place_scenery(bng, site, inrun, landing)
        cars = random_cars(bng, args.jumps)
        print("Today's jumpers:", ", ".join(name for _, _, name in cars))
        results = []
        for n, (model, config, name) in enumerate(cars, 1):
            try:
                results.append(jump(bng, site, n, model, config, name, inrun, landing))
            except (BNGError, ValueError) as exc:  # one broken car must not end the show
                print(f"  jump {n} ({name}) failed: {exc}")
        board = scoreboard(results)
        print("\nScoreboard\n" + board)
        in_pool = sum(r["finish"] in WINS for r in results)
        print(f"{in_pool} of {len(results)} made it into the {site.finish}")
        best = next((r for r in sorted(results, key=lambda r: -(r["distance"] or -1))), None)
        if best and best["distance"] is not None:
            bng.ui.display_message(f"Longest jump: {best['name']}, {best['distance']:.1f} m. "
                                   f"{in_pool} of {len(results)} made it into the {site.finish}")
        bng.control.queue_lua_command("raceCam.stop()")
    finally:
        bng.disconnect()
