"""Soapbox race: six cars race down a giant toy-track ramp.

Inspired by Car Pal's "the Soapbox race I always wanted to do as a kid": a start
gate with lane dividers inside a building, a wide orange chevron ramp out of a
window and over a crest, down a pitched house roof, then a narrow ramp to the
ground. BeamNG cannot shrink cars, so the track is built big instead.
"""

import argparse
import itertools
import math
import os
import random
import time

from beamngpy import BeamNGpy, MeshRoad, Scenario, Vehicle, angle_to_quat

from beamng_demo import default_host, world
from beamng_demo.highlights import SHOTS_NO_BLIMP, Spotter, play_reel
from beamng_demo.world import Director, car_positions, park

LEVEL = "smallgrid"
ART = f"/levels/{LEVEL}/art/soapbox"  # game path; the files go in the user folder

# Track centreline control points: x, y, top-of-surface z, width, and the section
# that starts at that point (it picks the surface material). Cars run toward +x.
# Everything past the window is four times as long as the first design, at the
# same heights, so the slopes are a quarter as steep; the cars drive with their
# engines. The runout after the ramp keeps its length.
CONTROL = [
    (-130, 0, 80.0, 38, "start"),  # start gate, on a solid starting block
    (-90, 0, 69.5, 38, "wide"),
    (-46, 0, 59.0, 38, "wide"),    # in through the building's back window
    (-2, 0, 47.5, 38, "wide"),     # out through its front window
    # The hill comes soon after the building, while the cars are still slow:
    # further down the ramp they reached it fast and flew off the far side.
    (100, 0, 40.0, 32, "wide"),    # bottom of the dip
    (228, 0, 50.5, 26, "wide"),    # crest: climbs 10.5 m from the dip (5x the first 2.1 m)
    (356, 0, 40.0, 24, "wide"),    # mirror of the dip; HILL shapes the hump between
    (580, 0, 29.0, 26, "wide"),    # long even run down to the roof (HILL shapes it too)
    (798, 0, 18.6, 28, "roof"),    # onto the house roof
    (950, 0, 11.2, 28, "narrow"),  # eaves
    # A 22 m funnel from 28 m to 12 m squeezed the outside cars together and the
    # AI braked to a stop there; this one narrows over about 260 m.
    (1054, 1, 8.8, 18, "narrow"),
    (1214, 8, 6.4, 13, "narrow"),  # gentle S-bend
    (1398, -5, 4.0, 13, "narrow"),
    (1546, 0, 0.3, 14, "runout"),
    (1604, 0, 0.02, 16, "runout"),
    (1734, 0, 0.02, 16, "runout"),
    (1974, 0, 0.02, 16, "runout"),
]
FINISH_X = 1574.0      # on the ground, just past the bottom of the ramp

# The first hill is a symmetric cosine hump, so the drop after the crest is the
# mirror of the climb. The spline alone made them differ by up to 1.2 m, because
# its shape depends on the points around the hill. Values match CONTROL.
HILL_CREST_X = 228.0
HILL_HALF_LENGTH = 128.0   # crest to dip, and crest to mirror point
HILL_BASE_Z = 40.0
HILL_HEIGHT = 10.5
HILL_BLEND = 24.0          # metres to ease the flat start of the hump into the track
ROOF_EASE = 40.0           # metres from the flat foot of the hill into the roof slope
EAVES_BLEND = 20.0         # metres to hand back to the spline after the roof

# Crest camera: while the leader is from CREST_CAM_BEFORE before the first
# hill's crest to CREST_CAM_AFTER past it, film from a fixed spot beside the
# track that looks back up at the top.
CREST_SEARCH = (100, 360)  # x range that holds the first hill
CREST_CAM_BEFORE = 120.0
CREST_CAM_AFTER = 30.0
CREST_CAM_AHEAD = 40.0     # spot is this far past the crest...
CREST_CAM_SIDE = 12.0      # ...this far out from the track edge...
CREST_CAM_UP = 3.0         # ...and this far above the crest
CAMERA_SHOTS = ("tv", "chase")  # no blimp: it was too high to see the crest
CRASH_WALL_AFTER = 120.0  # metres past the finish
RUNOUT_WALL_HEIGHT = 1.2
WALL_START_AFTER = 20.0   # side walls start this far past the finish line
WALL_FLARE = 6.0          # metres wider at the open end of the side walls
WALL_FLARE_LENGTH = 30.0  # over which the flare closes in to the track edge
SAMPLE = 2.0           # metres between centreline samples
ROAD_DEPTH = 1.0

# The blue section is a U-shaped channel: a parabola across, edges U_RISE metres
# above the middle. Mesh roads are always flat across, so it is built as custom
# ProceduralMesh chunks. The U fades in and out over U_TAPER metres so it meets
# the flat roof and runout without a step.
U_SECTION = "narrow"
U_RISE = 1.2
U_TAPER = 30.0
U_ACROSS = 9          # surface points across the channel
U_RING = 4.0          # metres between cross-sections
U_CHUNK = 25          # cross-sections per mesh object
GROUND_Z = 0.0

# The building the orange ramp passes through: back and front walls at these x,
# with a window where the ramp crosses each one.
BUILDING_BACK_X = -47.0
BUILDING_FRONT_X = -1.0
BUILDING_STORIES = 5
WINDOW_HEIGHT = 4.5    # clear height of each window above the ramp
BUILDING_ABOVE = 8.0   # building top above the back window

SLOT_WIDTH = 3.0       # start gate lane width; 2.6 m left 0.35 m a side and a car wedged
SLOT_START = 10.0      # metres from the top of the start slope where cars sit
DIVIDER_HEIGHT = 0.8
DIVIDER_END = 40.0     # dividers run from the top of the slope to here

AGGRESSION = 0.8       # with engines on, higher values overdrive on the way down
AGGRESSION_SPREAD = 0.03
STUCK_SECONDS = 10.0
FALL_DROP = 3.0        # metres below the track surface that counts as falling off
FINISH_GRACE = 30.0
TIME_LIMIT = 150.0

CARS = (
    # vid, model, part config, colour (RGB 0..1), caption label. The six cars
    # with the closest lap times on the stock car oval.
    ("sb01", "sbr", "vehicles/sbr/track.pc", (0.85, 0.10, 0.10), "Red SBR4"),
    ("sb02", "etkc", "vehicles/etkc/race_DCT.pc", (0.92, 0.92, 0.95), "White GT-IV"),
    ("sb03", "sunburst2", "vehicles/sunburst2/trackday_M.pc", (0.95, 0.75, 0.10), "Yellow Sunburst"),
    ("sb04", "etkc", "vehicles/etkc/race_DCT.pc", (0.62, 0.64, 0.68), "Silver GT-IV"),
    ("sb05", "etk800", "vehicles/etk800/rennspecht_8r_A.pc", (0.10, 0.30, 0.85), "Blue Rennspecht"),
    ("sb06", "etkc", "vehicles/etkc/kc6x_trackday_A.pc", (0.10, 0.60, 0.25), "Green Kc6x"),
)
LABELS = {c[0]: c[4] for c in CARS}

SURFACE = {  # section -> material
    "start": "soap_wood",
    "wide": "soap_chevron_orange",
    "roof": "soap_roof",
    "narrow": "soap_chevron_blue",
    "runout": "track_editor_C_center",
}


# ---------------------------------------------------------------- geometry

def _smooth(points: list[tuple], steps: int) -> list[tuple]:
    """Cubic Hermite through the points, with each tangent scaled to its own
    segment's length. Plain Catmull-Rom treats every segment as the same length,
    so a 42 m start slope next to a 168 m ramp bent from 27 degrees to 6."""
    def dist(a, b):
        return math.hypot(b[0] - a[0], b[1] - a[1]) or 1e-9

    out = []
    n = len(points)
    for i in range(n - 1):
        p0, p1, p2 = points[max(i - 1, 0)], points[i], points[i + 1]
        p3 = points[min(i + 2, n - 1)]
        d01, d12, d23 = dist(p0, p1), dist(p1, p2), dist(p2, p3)
        m1 = [(c - a) * d12 / (d01 + d12) for a, c in zip(p0, p2)] if i > 0 else \
             [c - b for b, c in zip(p1, p2)]
        m2 = [(d - b) * d12 / (d12 + d23) for b, d in zip(p1, p3)] if i < n - 2 else \
             [c - b for b, c in zip(p1, p2)]
        for k in range(steps):
            t = k / steps
            h00, h10 = 2 * t ** 3 - 3 * t * t + 1, t ** 3 - 2 * t * t + t
            h01, h11 = -2 * t ** 3 + 3 * t * t, t ** 3 - t * t
            out.append(tuple(h00 * b + h10 * u + h01 * c + h11 * v
                             for b, c, u, v in zip(p1, p2, m1, m2)))
    out.append(points[-1])
    return out


def _smoothstep(w: float) -> float:
    return w * w * (3 - 2 * w)


def _hill(x: float, spline_z: float) -> float:
    """Height at x with the first hill's cosine hump, and an even run down the
    roof after it, in place of the spline.

    Before the hump the flat end eases into the spline over HILL_BLEND metres.
    After it the track curves gently into one steady slope down to the eaves:
    the spline levelled off at the foot of the hill and then dropped at up to
    8 degrees before the roof, a crest sharp enough to throw the cars.
    """
    d = x - HILL_CREST_X
    base, half = HILL_BASE_Z, HILL_HALF_LENGTH
    eaves_x, eaves_z = next((c[0], c[2]) for c in CONTROL if c[4] == "narrow")
    run_d = eaves_x - HILL_CREST_X  # crest to eaves
    slope = (base - eaves_z) / (run_d - half - ROOF_EASE / 2)  # steady slope after the ease
    if d < -half - HILL_BLEND:
        return spline_z
    if d < -half:
        return base + (spline_z - base) * _smoothstep((-d - half) / HILL_BLEND)
    if abs(d) <= half:
        return base + HILL_HEIGHT * (1 + math.cos(math.pi * d / half)) / 2
    if half < d <= half + ROOF_EASE:  # from flat into the steady slope
        e = d - half
        return base - slope * e * e / (2 * ROOF_EASE)
    line = base - slope * ROOF_EASE / 2 - slope * (d - half - ROOF_EASE)
    if d <= run_d:
        return line
    if d <= run_d + EAVES_BLEND:  # hand back to the spline past the eaves
        return line + (spline_z - line) * _smoothstep((d - run_d) / EAVES_BLEND)
    return spline_z


class Track:
    """The centreline sampled every SAMPLE metres of horizontal distance."""

    def __init__(self):
        numeric = [c[:4] for c in CONTROL]
        dense = _smooth(numeric, 40)
        dist = [0.0]
        for a, b in itertools.pairwise(dense):
            dist.append(dist[-1] + math.hypot(b[0] - a[0], b[1] - a[1]))
        self.length = dist[-1]
        self.s, self.x, self.y, self.z, self.w, self.section = [], [], [], [], [], []
        j = 0
        s = 0.0
        while s <= self.length:
            while dist[j + 1] < s:
                j += 1
            t = (s - dist[j]) / max(dist[j + 1] - dist[j], 1e-9)
            x, y, z, w = (a + (b - a) * t for a, b in zip(dense[j], dense[j + 1]))
            self.s.append(s)
            self.x.append(x)
            self.y.append(y)
            z = _hill(x, z)
            self.z.append(max(z, GROUND_Z + 0.02))  # smoothing can dip below the grass
            self.w.append(w)
            self.section.append(self._section_at(x))
            s += SAMPLE
        u_run = [s for s, sec in zip(self.s, self.section) if sec == U_SECTION]
        self.u_start, self.u_end = u_run[0], u_run[-1]

    @staticmethod
    def _section_at(x: float) -> str:
        name = CONTROL[0][4]
        for cx, _, _, _, sec in CONTROL:
            if x >= cx - 1e-6:
                name = sec
        return name

    def index(self, s: float) -> int:
        return min(max(round(s / SAMPLE), 0), len(self.s) - 1)

    def heading(self, i: int) -> float:
        a, b = max(i - 1, 0), min(i + 1, len(self.s) - 1)
        return math.atan2(self.y[b] - self.y[a], self.x[b] - self.x[a])

    def slope(self, i: int) -> float:
        """Downhill angle in radians (positive going down)."""
        a, b = max(i - 1, 0), min(i + 1, len(self.s) - 1)
        return math.atan2(self.z[a] - self.z[b], self.s[b] - self.s[a])

    def rise(self, s: float, lateral: float) -> float:
        """Height of the U channel's surface above the centreline at `lateral`."""
        i = self.index(s)
        if self.section[i] != U_SECTION:
            return 0.0
        into, left = s - self.u_start, self.u_end - s
        taper = min(1.0, max(0.0, min(into, left) / U_TAPER))
        taper = taper * taper * (3 - 2 * taper)
        half = self.w[i] / 2
        return U_RISE * taper * min(1.0, (lateral / half) ** 2)

    def point(self, s: float, lateral: float = 0.0) -> tuple[float, float, float]:
        """Surface point `lateral` metres left of the centreline at arc length s."""
        i = self.index(s)
        h = self.heading(i)
        return (self.x[i] - lateral * math.sin(h), self.y[i] + lateral * math.cos(h),
                self.z[i] + self.rise(s, lateral))

    def s_at_x(self, x: float) -> float:
        return min(self.s, key=lambda s: abs(self.x[self.index(s)] - x))

    def nearest(self, pos) -> int:
        """Sample index nearest to `pos` in plan view."""
        return min(range(len(self.s)),
                   key=lambda i: (self.x[i] - pos[0]) ** 2 + (self.y[i] - pos[1]) ** 2)


# ---------------------------------------------------------------- textures

def _chevron(color):
    # u runs across the road, v along it; three V-shaped stripes per tile.
    def pixel(x, y):
        u, v = x / 256, y / 256
        return color if (v * 3 + abs(u - 0.5) * 2.4) % 1.0 < 0.5 else (245, 245, 240)
    return pixel


def _roof(x, y):
    row = y // 16
    shift = 16 if row % 2 else 0
    edge = (y % 16 < 2) or ((x + shift) % 32 < 2)
    return (60, 30, 25) if edge else (130 + (row * 37) % 25, 45, 35)


def _wood(x, y):
    plank = x // 32
    grain = int(12 * math.sin(y / 9.0 + plank * 1.7))
    gap = x % 32 < 2
    return (90, 55, 30) if gap else (180 + grain, 125 + grain, 70 + grain // 2)


TEXTURES = {
    "chevron_orange.png": _chevron((240, 130, 30)),
    "chevron_blue.png": _chevron((40, 110, 210)),
    "roof.png": _roof,
    "wood.png": _wood,
}
MATERIALS = {  # material name -> texture file
    "soap_chevron_orange": "chevron_orange.png",
    "soap_chevron_blue": "chevron_blue.png",
    "soap_roof": "roof.png",
    "soap_wood": "wood.png",
}


def install_art(bng: BeamNGpy) -> None:
    world.install_art(bng, ART, TEXTURES, MATERIALS)


# ---------------------------------------------------------------- scene

def slot_lateral(k: int) -> float:
    return (k - (len(CARS) - 1) / 2) * SLOT_WIDTH


def build_scenario(bng: BeamNGpy, track: Track, order: list[tuple]) -> tuple[Scenario, list]:
    scenario = Scenario(LEVEL, "soapbox_race")
    # One mesh road per section so each can have its own surface. Neighbours
    # share their boundary sample, so there is no gap.
    runs: list[list[int]] = []
    for i, sec in enumerate(track.section):
        if not runs or track.section[runs[-1][-1]] != sec:
            if runs:
                runs[-1].append(i)
            runs.append([i])
        else:
            runs[-1].append(i)
    for n, run in enumerate(runs):
        sec = track.section[run[0]]
        if sec == U_SECTION:  # built as a custom mesh in place_scenery
            continue
        road = MeshRoad(SURFACE[sec], side_material="soap_wood", rid=f"soap_{n}_{sec}",
                        default_width=10, default_depth=ROAD_DEPTH, texture_length=8)
        road.add_nodes(*[(track.x[i], track.y[i], track.z[i], track.w[i], ROAD_DEPTH)
                         for i in run[::2] + ([run[-1]] if len(run) % 2 == 0 else [])])
        scenario.add_mesh_road(road)

    cars = []
    s = SLOT_START
    i = track.index(s)
    for k, (vid, model, config, color, _) in enumerate(order):
        x, y, z = track.point(s, slot_lateral(k))
        car = Vehicle(vid, model=model, part_config=config, color=color, license=vid.upper())
        rot = world.spawn_rotation(track.heading(i), track.slope(i))
        scenario.add_vehicle(car, pos=(x, y, z + 0.8), rot_quat=rot)
        cars.append((car, slot_lateral(k)))
    scenario.make(bng)
    return scenario, cars


def u_channel(track: Track) -> list[world.Mesh]:
    """The blue section as solid chunks: curved top, flat bottom, closed sides."""
    run = [i for i in range(len(track.s)) if track.section[i] == U_SECTION]
    s0, s1 = track.s[run[0]], track.s[run[-1]]
    count = max(int((s1 - s0) / U_RING), 1)
    rings = [s0 + (s1 - s0) * k / count for k in range(count + 1)]

    def section(s):
        """Cross-section points: top surface left to right, then bottom right, left."""
        i = track.index(s)
        half = track.w[i] / 2
        top = [track.point(s, half - 2 * half * k / (U_ACROSS - 1)) for k in range(U_ACROSS)]
        bottom_z = track.z[i] - ROAD_DEPTH
        right, left = top[-1], top[0]
        return top + [(right[0], right[1], bottom_z), (left[0], left[1], bottom_z)]

    meshes = []
    for c, first in enumerate(range(0, count, U_CHUNK)):
        last = min(first + U_CHUNK, count)
        mb = world.MeshBuilder(f"soap_uchannel_{c:02d}", SURFACE[U_SECTION], track.point(rings[first]))
        for r in range(first, last):
            sa, sb = rings[r], rings[r + 1]
            pa, pb = section(sa), section(sb)
            va, vb = sa / 8.0, sb / 8.0  # texture repeats every 8 m, like the mesh roads
            i = track.index(sa)
            h = track.heading(i)
            side = (-math.sin(h), math.cos(h), 0.0)  # left
            for k in range(U_ACROSS - 1):  # curved top
                ua, ub = k / (U_ACROSS - 1), (k + 1) / (U_ACROSS - 1)
                mb.quad([pa[k], pa[k + 1], pb[k + 1], pb[k]],
                        [(ua, va), (ub, va), (ub, vb), (ua, vb)], (0, 0, 1))
            right, bottom_r, bottom_l = U_ACROSS - 1, U_ACROSS, U_ACROSS + 1
            flat_tex = [(0, va), (1, va), (1, vb), (0, vb)]
            mb.quad([pa[right], pa[bottom_r], pb[bottom_r], pb[right]], flat_tex,
                    tuple(-x for x in side))
            mb.quad([pa[bottom_r], pa[bottom_l], pb[bottom_l], pb[bottom_r]], flat_tex, (0, 0, -1))
            mb.quad([pa[bottom_l], pa[0], pb[0], pb[bottom_l]], flat_tex, side)
        meshes.append(mb.mesh())
    return meshes


def place_scenery(bng: BeamNGpy, track: Track) -> None:
    solid: list[world.Box] = []
    visual: list[world.Box] = []
    flat = (0, 0, 0, 1)

    # Grass slab over the grid floor.
    visual.append(("soap_grass", (1000, 500, 1.0), "soap_grass_tint", (300, 0, GROUND_Z + 0.01 - 0.5), flat))

    # Start gate: lane dividers down the start slope.
    i0, i1 = track.index(0.0), track.index(DIVIDER_END)
    mid = track.index(DIVIDER_END / 2)
    run = track.s[i1] - track.s[i0]
    length = math.hypot(run, track.z[i0] - track.z[i1])
    pitch = math.atan2(track.z[i0] - track.z[i1], run)
    for k in range(len(CARS) + 1):
        lateral = (k - len(CARS) / 2) * SLOT_WIDTH
        x, y, z = track.point(track.s[mid], lateral)
        solid.append((f"soap_div_{k:02d}", (length, 0.2, DIVIDER_HEIGHT), "soap_divider",
                      (x, y, z + DIVIDER_HEIGHT / 2), world.quat_yaw_pitch(track.heading(mid), pitch)))

    # Starting block: a solid block under the start gate.
    gate_end = track.index(DIVIDER_END)
    block_top = track.z[gate_end] - ROAD_DEPTH - 0.2
    gx0, gx1 = track.x[0] - 2, track.x[gate_end]
    solid.append(("soap_start_block", (gx1 - gx0, CONTROL[0][3] + 2, block_top), "soap_brick",
                  ((gx0 + gx1) / 2, 0, block_top / 2), flat))

    # Five-story building the ramp passes through, with a window in the back
    # wall where the ramp comes in and one in the front wall where it leaves.
    half_w = CONTROL[0][3] / 2 + 3
    back_z = track.z[track.nearest((BUILDING_BACK_X, 0))]
    front_z = track.z[track.nearest((BUILDING_FRONT_X, 0))]
    top = back_z + BUILDING_ABOVE
    depth = BUILDING_FRONT_X - BUILDING_BACK_X
    cx = (BUILDING_BACK_X + BUILDING_FRONT_X) / 2
    for wall, wx, wz in (("back", BUILDING_BACK_X, back_z), ("front", BUILDING_FRONT_X, front_z)):
        low = wz - ROAD_DEPTH - 0.3  # stops just under the ramp
        high = wz + WINDOW_HEIGHT
        solid += [
            (f"soap_bld_{wall}_low", (1, 2 * half_w, low), "soap_brick", (wx, 0, low / 2), flat),
            (f"soap_bld_{wall}_high", (1, 2 * half_w, top - high), "soap_brick",
             (wx, 0, (top + high) / 2), flat),
        ]
    solid += [
        ("soap_bld_left", (depth, 1, top), "soap_brick", (cx, half_w, top / 2), flat),
        ("soap_bld_right", (depth, 1, top), "soap_brick", (cx, -half_w, top / 2), flat),
        ("soap_bld_roof", (depth + 1, 2 * half_w + 1, 1), "soap_brick", (cx, 0, top), flat),
    ]
    # Floor bands around the outside mark the stories (visual only), skipping
    # any that would cross a window.
    for k in range(1, BUILDING_STORIES):
        bz = top * k / BUILDING_STORIES
        if any(wz - ROAD_DEPTH - 0.5 < bz < wz + WINDOW_HEIGHT + 0.5 for wz in (back_z, front_z)):
            continue
        visual += [
            (f"soap_band_{k}_l", (depth + 1.4, 0.8, 0.6), "soap_band", (cx, half_w, bz), flat),
            (f"soap_band_{k}_r", (depth + 1.4, 0.8, 0.6), "soap_band", (cx, -half_w, bz), flat),
            (f"soap_band_{k}_b", (0.8, 2 * half_w + 1.4, 0.6), "soap_band", (BUILDING_BACK_X, 0, bz), flat),
            (f"soap_band_{k}_f", (0.8, 2 * half_w + 1.4, 0.6), "soap_band", (BUILDING_FRONT_X, 0, bz), flat),
        ]

    # House under the roof section.
    roof = [i for i, sec in enumerate(track.section) if sec == "roof"]
    hx0, hx1 = track.x[roof[0]], track.x[roof[-1]]
    eave = min(track.z[i] for i in roof) - ROAD_DEPTH - 0.2
    roof_width = next(c[3] for c in CONTROL if c[4] == "roof")
    solid.append(("soap_house", (hx1 - hx0, roof_width - 2, eave), "soap_siding",
                  ((hx0 + hx1) / 2, 0, eave / 2), flat))

    # Runout: side walls up to a crash wall, so finished cars cannot wander off
    # across the map. They start past the finish line and flare out at the open
    # end: cars that run wide off the ramp went round a wall that started at
    # the bottom of the ramp and were trapped behind it, or hit its blunt end.
    fs = track.s_at_x(FINISH_X)
    wall_s = fs + CRASH_WALL_AFTER
    h = RUNOUT_WALL_HEIGHT
    s = fs + WALL_START_AFTER

    def lateral(at: float) -> float:
        flare = WALL_FLARE * max(0.0, 1 - (at - fs - WALL_START_AFTER) / WALL_FLARE_LENGTH)
        return track.w[track.index(at)] / 2 + 0.5 + flare

    n = 0
    while s < wall_s:
        step = min(6.0, wall_s - s)
        for side in (-1, 1):
            x0, y0, z0 = track.point(s, side * lateral(s))
            x1, y1, _ = track.point(s + step, side * lateral(s + step))
            solid.append((f"soap_runwall_{n:03d}_{side + 1}",
                          (math.hypot(x1 - x0, y1 - y0) + 0.1, 0.6, h), "soap_tirewall",
                          ((x0 + x1) / 2, (y0 + y1) / 2, z0 + h / 2),
                          angle_to_quat((0, 0, math.degrees(math.atan2(y1 - y0, x1 - x0))))))
        s += step
        n += 1
    wi = track.index(wall_s)
    x, y, z = track.point(wall_s)
    solid.append(("soap_crashwall", (1.5, track.w[wi] + 2, h + 0.6), "soap_tirewall",
                  (x, y, z + (h + 0.6) / 2), angle_to_quat((0, 0, math.degrees(track.heading(wi))))))

    # Support posts under the ramps (visual only).
    for i in range(0, len(track.s), int(16 / SAMPLE)):
        if track.section[i] in ("start", "roof", "runout"):
            continue
        if BUILDING_BACK_X - 1 < track.x[i] < BUILDING_FRONT_X + 1:  # inside the building
            continue
        height = track.z[i] - ROAD_DEPTH
        if height < 1.0:
            continue
        for side in (-1, 1):
            x, y, _ = track.point(track.s[i], side * (track.w[i] / 2 - 1.0))
            visual.append((f"soap_post_{i:04d}_{side + 1}", (0.7, 0.7, height), "soap_post",
                           (x, y, height / 2), flat))

    # Checkered finish line across the runout (visual only).
    fi = track.index(fs)
    cols = int(track.w[fi])
    for row in range(2):
        for col in range(cols):
            x, y, z = track.point(fs + row - 0.5, col - (cols - 1) / 2)
            mat = "race_white" if (row + col) % 2 == 0 else "race_black"
            visual.append((f"soap_line_{row}_{col:02d}", (1, 1, 0.01), mat, (x, y, z + 0.005),
                           angle_to_quat((0, 0, math.degrees(track.heading(fi))))))

    world.build_meshes(bng, u_channel(track))  # made solid by build_boxes' collision rebuild
    world.build_boxes(bng, solid, visual, [
        ("soap_grass_tint", "0.55 0.8 0.45 1", "/levels/GridMap/art/shapes/misc/Grass-02-D.dds"),
        ("soap_divider", "0.75 0.45 0.2 1", None),
        ("soap_brick", "0.55 0.27 0.2 1", None),
        ("soap_siding", "0.55 0.7 0.75 1", None),
        ("soap_post", "0.1 0.45 0.45 1", None),
        ("soap_band", "0.85 0.82 0.75 1", None),
        ("soap_tirewall", "0.12 0.12 0.13 1", None),
        ("race_white", "0.95 0.95 0.95 1", None),
        ("race_black", "0.03 0.03 0.03 1", None),
    ])


def first_crest(track: Track) -> tuple[float, tuple[float, float, float]]:
    """Arc length of the first hill's crest, and a camera spot that looks back up
    at it: beside the track, a little past the top and just above it."""
    hill = [i for i in range(len(track.s)) if CREST_SEARCH[0] < track.x[i] < CREST_SEARCH[1]]
    top = max(hill, key=lambda i: track.z[i])
    s = track.s[top]
    i = track.index(s + CREST_CAM_AHEAD)
    x, y, _ = track.point(track.s[i], track.w[i] / 2 + CREST_CAM_SIDE)
    return s, (x, y, track.z[top] + CREST_CAM_UP)


def camera_towers(track: Track) -> list[tuple[float, float, float]]:
    def beside(x, side, height):
        i = track.index(track.s_at_x(x))
        px, py, pz = track.point(track.s[i], side * (track.w[i] / 2 + 18))
        return (px, py, pz + height)
    start = (track.x[0] - 6, 0.0, CONTROL[0][2] + 6)  # behind and above the gate
    return [start, beside(-70, 1, 8), beside(60, 1, 10), beside(200, -1, 9), beside(420, 1, 8),
            beside(640, -1, 8),
            beside(882, -1, 10), beside(1122, 1, 7), beside(1322, -1, 6), beside(1482, 1, 5),
            beside(FINISH_X, 1, 4)]


def drive(car: Vehicle, track: Track, start_s: float, lane: float, aggression: float) -> None:
    """Give the car its own corridor down the track, at the same share of the width
    as its start slot, so nobody has to cut across the field."""
    u = lane / (track.w[0] / 2 - 1.5)
    finish_s = track.s_at_x(FINISH_X)
    nodes = []
    s = start_s + 6
    while s < finish_s + CRASH_WALL_AFTER - 15:
        i = track.index(s)
        half = track.w[i] / 2 - 1.5
        x, y, z = track.point(s, u * half)
        r = 0.8 if s < DIVIDER_END else max(1.2, min(2.5, half / 4))
        nodes.append(f"{{x={x:.2f}, y={y:.2f}, z={z + 0.02:.2f}, r={r:.1f}}}")
        s += 6
    car.queue_lua_command(
        'input.event("parkingbrake", 0, 1) '
        f"ai.driveUsingPath{{script = {{{', '.join(nodes)}}}, "
        f"avoidCars = 'on', aggression = {aggression:.3f}}}")


# ---------------------------------------------------------------- race

def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--host", default=default_host(),
                   help="Host where BeamNG listens (default: env BNG_HOST, else found automatically)")
    p.add_argument("--port", type=int, default=int(os.environ.get("BNG_PORT", "25252")))
    p.add_argument("--no-launch", action="store_true",
                   help="Accepted to match beamng-race; this always connects to a running game")
    p.add_argument("--aggression", type=float, default=AGGRESSION,
                   help="AI aggression, 0.3 (calm) to 1.0; low values brake on the way down")
    p.add_argument("--shot-seconds", type=float, default=6.0, help="Seconds per camera shot")
    p.add_argument("--seed", type=int, default=None, help="Random seed for the gate order")
    p.add_argument("--no-highlights", dest="highlights", action="store_false",
                   help="Do not record a replay or play the highlight reel")
    p.add_argument("--reel-countdown", type=float, default=5.0,
                   help="Seconds to start screen recording before the reel plays")
    return p.parse_args()


def main() -> None:
    args = parse_args()
    random.seed(args.seed)
    track = Track()
    order = list(CARS)
    random.shuffle(order)

    bng = BeamNGpy(args.host, args.port)
    bng.open(launch=False)
    try:
        install_art(bng)
        scenario, cars = build_scenario(bng, track, order)
        bng.scenario.load(scenario)
        bng.scenario.start()
        bng.control.pause()
        for car, _ in cars:  # held in the gate until GO
            car.queue_lua_command('input.event("parkingbrake", 1, 1)')
        place_scenery(bng, track)
        vids = [car.vid for car, _ in cars]
        world.install_camera(bng, vids, camera_towers(track))
        bng.control.queue_lua_command(f"raceCam.setShot('tv', '{vids[len(vids) // 2]}')")
        director = Director(bng, args.shot_seconds, CAMERA_SHOTS)
        crest_s, crest_spot = first_crest(track)
        crest_cam = "waiting"  # -> "on" -> "done"
        spotter = Spotter(LABELS, show_laps=False)
        print("Gate order:", ", ".join(LABELS[v] for v in vids))

        replay_file = None
        if args.highlights:
            replay_file = bng.control.queue_lua_command(
                "local r = core_replay.startRecording() return r.success and r.filename or ''",
                response=True) or None
        bng.control.resume()
        for n in (3, 2, 1):
            bng.ui.display_message(f"{n}...")
            time.sleep(1)
        bng.ui.display_message("GO!")
        aggression = {}
        for k, (car, lane) in enumerate(cars):
            aggression[car.vid] = min(1.0, args.aggression + random.uniform(
                -AGGRESSION_SPREAD, AGGRESSION_SPREAD))
            drive(car, track, SLOT_START, lane, aggression[car.vid])

        finish_s = track.s_at_x(FINISH_X)
        progress = {v: SLOT_START for v in vids}
        best_time = {v: 0.0 for v in vids}
        best_rt = {v: 0.0 for v in vids}
        finished: list[str] = []
        finish_time: dict[str, float] = {}
        out: list[str] = []
        t0 = time.monotonic()
        last_print = 0.0
        while len(finished) + len(out) < len(cars):
            now = time.monotonic() - t0
            positions, rt = car_positions(bng)
            for car, _ in cars:
                v = car.vid
                if v in finished or v in out or v not in positions:
                    continue
                pos = positions[v]
                i = track.nearest(pos)
                s = track.s[i]
                if pos[2] < track.z[i] - FALL_DROP and track.z[i] > FALL_DROP + 1:
                    out.append(v)
                    park(car)
                    spotter.crash(rt, v, "falls off the ramp!")
                    print(f"  {now:6.1f}s  {LABELS[v]} falls off the ramp")
                    continue
                if s >= finish_s:
                    finished.append(v)
                    finish_time[v] = now
                    car.ai.set_mode("stopping")
                    spotter.finish(rt, v, len(finished))
                    print(f"  {now:6.1f}s  {LABELS[v]} finishes P{len(finished)}")
                elif s > progress[v] + 2:
                    progress[v], best_time[v], best_rt[v] = s, now, rt
                elif now - best_time[v] > STUCK_SECONDS:
                    out.append(v)
                    park(car)
                    spotter.crash(best_rt[v], v, "is stuck!")
                    print(f"  {now:6.1f}s  {LABELS[v]} is stuck and out")

            running = sorted((v for v in vids if v not in finished and v not in out),
                             key=progress.get, reverse=True)
            if replay_file:
                spotter.tick(rt, running, positions, {})
            lead = running[0] if running else None
            if lead and crest_cam == "waiting" and progress[lead] > crest_s - CREST_CAM_BEFORE:
                crest_cam = "on"
                bng.control.queue_lua_command(
                    f"raceCam.setSpot({crest_spot[0]:.2f}, {crest_spot[1]:.2f}, "
                    f"{crest_spot[2]:.2f}, '{lead}')")
            if crest_cam == "on":
                if lead and progress[lead] >= crest_s and not any(
                        m.kind == "crest" for m in spotter.moments):
                    spotter.crest(rt, lead, crest_spot)
                if not lead or progress[lead] > crest_s + CREST_CAM_AFTER:
                    crest_cam = "done"
                director.hold(now, f"crest cam on {lead}")
            else:
                director.tick(now, finished + running or vids)
            if now - last_print > 3.0:
                last_print = now
                lead = running[0] if running else (finished[0] if finished else vids[0])
                print(f"{now:6.1f}s  leader {LABELS[lead]}  "
                      f"{progress.get(lead, 0):.0f}/{finish_s:.0f} m   [cam: {director.describe()}]")
            if finished and now - finish_time[finished[0]] > FINISH_GRACE:
                break
            if now > TIME_LIMIT:
                print("Time limit reached.")
                break
            time.sleep(0.05)

        running = sorted((v for v in vids if v not in finished and v not in out),
                         key=progress.get, reverse=True)
        places = [f"P{i + 1} {LABELS[v]}" for i, v in enumerate(finished)]
        places += [f"P{len(finished) + i + 1} {LABELS[v]} ({progress[v]:.0f} m)"
                   for i, v in enumerate(running)]
        result = "  ".join(places)
        if out:
            result += "   Out: " + ", ".join(LABELS[v] for v in out)
        print("Result:", result)
        bng.ui.display_message(f"Checkered flag!  {result}")
        time.sleep(5)
        if replay_file:
            bng.control.queue_lua_command("core_replay.stopRecording()")
        for car, _ in cars:
            park(car)
        if replay_file:
            print(f"Replay saved: {replay_file} (in the game's user folder)")
            play_reel(bng, replay_file, spotter.moments, args.reel_countdown, SHOTS_NO_BLIMP)
        bng.control.queue_lua_command("raceCam.stop()")
    finally:
        bng.disconnect()
