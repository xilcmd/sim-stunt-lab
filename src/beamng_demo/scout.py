"""Find a spot on a map where the big jump course fits the terrain.

Loads the map, measures the ground (terrain, buildings, roads) on a grid, and
scores every position and heading: the course should sit close above the
hillside (short posts), clear everything under it, and face water if the map
has a sea. The best spot is checked again in the game every 4 m and saved to
sites.json, so `beamng-bigjump --type <name>` can use it.

Trees are not measured: rays pass through them, and the big jump removes the
trees along its course anyway.
"""

import argparse
import json
import math
import os
import time

import numpy as np
from beamngpy import BeamNGpy, Scenario, Vehicle

from beamng_demo import bigjump as bj
from beamng_demo import default_host

GRID_CELLS = 160        # grid points along each side of the map
POSITION_STEP = 64.0    # metres between candidate start points
HEADING_STEP = 15       # degrees between candidate headings
GAP_CLEAR = 3.0         # the ground under the gap must be this far below the lip
WATER_MARGIN = 1.5      # outrun at least this far above the sea
INRUN_HALF, LANDING_HALF = 7.0, 26.0  # half widths of the footprint checked
SEA_AHEAD = (100.0, 1500.0)           # range past the pool where water scores
CHECK_STEP = 4.0        # metres between points in the close check in the game


def footprint(inrun: bj.Profile, landing: bj.Profile, step: float):
    """Points (u, v, top z) covering the course, in course coordinates. The
    top is the course surface, or for the gap the lip less GAP_CLEAR."""
    lip_u, lip_z = inrun.end()
    pool_end = landing.end()[0] + bj.POOL_WALL + 1
    us, vs, tops, parts = [], [], [], []
    u = inrun.x[0]
    while u <= pool_end:
        if u <= lip_u:
            top, half, part = inrun.at_x(u)[0], INRUN_HALF, "inrun"
        elif u < landing.x[0]:
            top, half, part = lip_z - GAP_CLEAR, INRUN_HALF, "gap"
        else:
            top, half, part = landing.at_x(min(u, landing.end()[0]))[0], LANDING_HALF, "landing"
        for v in np.linspace(-half, half, 5):
            us.append(u)
            vs.append(v)
            tops.append(top)
            parts.append(part)
        u += step
    return np.array(us), np.array(vs), np.array(tops), np.array(parts)


def load_map(bng: BeamNGpy, level: str) -> dict:
    sc = Scenario(level, "bigjump_scout")
    sc.add_vehicle(Vehicle("scout", model="pickup"), pos=(0, 0, 2000), rot_quat=(0, 0, 0, 1))
    sc.make(bng)
    bng.scenario.load(sc)
    bng.scenario.start()
    bng.control.resume()
    raw = bng.control.queue_lua_command("""
local w = nil
for _, n in ipairs(scenetree.findClassObjects('WaterPlane')) do
  local z = scenetree.findObject(n):getPosition().z
  if not w or z > w then w = z end
end
local t = scenetree.findObject(scenetree.findClassObjects('TerrainBlock')[1] or '')
if not t then return 'none' end
local b = t:getWorldBox()
return string.format('%.1f %.1f %.1f %.1f %s', b.minExtents.x, b.maxExtents.x,
  b.minExtents.y, b.maxExtents.y, w and string.format('%.2f', w) or 'none')""", response=True)
    if str(raw) == "none":
        raise SystemExit(f"{level} has no terrain: nothing to place the course on")
    x0, x1, y0, y1, sea = str(raw).split()
    return {"x0": float(x0), "x1": float(x1), "y0": float(y0), "y1": float(y1),
            "sea": None if sea == "none" else float(sea)}


def sample_grid(bng: BeamNGpy, box: dict) -> tuple[np.ndarray, np.ndarray, float, float, float]:
    """Surface heights (rays: terrain, buildings, roads) and terrain heights."""
    step = max(box["x1"] - box["x0"], box["y1"] - box["y0"]) / GRID_CELLS
    nx = int((box["x1"] - box["x0"]) / step)
    ny = int((box["y1"] - box["y0"]) / step)
    rows_s, rows_t = [], []
    for iy in range(ny):
        y = box["y0"] + step * (iy + 0.5)
        raw = str(bng.control.queue_lua_command(f"""
local o = {{}}
for ix = 0, {nx - 1} do
  local x = {box['x0']:.2f} + {step:.4f} * (ix + 0.5)
  local d = castRayStatic(vec3(x, {y:.2f}, 3000), vec3(0, 0, -1), 4000)
  local t = core_terrain.getTerrainHeight(vec3(x, {y:.2f}, 0)) or -9999
  table.insert(o, string.format('%.1f,%.1f', (d and d < 4000) and (3000 - d) or -9999, t))
end
return table.concat(o, ' ')""", response=True))
        cells = [c.split(",") for c in raw.split()]
        rows_s.append([float(a) for a, _ in cells])
        rows_t.append([float(b) for _, b in cells])
    return np.array(rows_s), np.array(rows_t), box["x0"] + step / 2, box["y0"] + step / 2, step


def lookup(field, ox, oy, step, x, y):
    """Bilinear lookup; off the map reads as very high (never allowed)."""
    ny, nx = field.shape
    fx, fy = (x - ox) / step, (y - oy) / step
    inside = (fx >= 0) & (fy >= 0) & (fx <= nx - 1.001) & (fy <= ny - 1.001)
    fx, fy = np.clip(fx, 0, nx - 1.001), np.clip(fy, 0, ny - 1.001)
    ix, iy = fx.astype(int), fy.astype(int)
    tx, ty = fx - ix, fy - iy
    v = (field[iy, ix] * (1 - tx) * (1 - ty) + field[iy, ix + 1] * tx * (1 - ty)
         + field[iy + 1, ix] * (1 - tx) * ty + field[iy + 1, ix + 1] * tx * ty)
    return np.where(inside & (v > -9000), v, 1e6)


def score_all(surface, terrain, ox, oy, step, box, sea):
    inrun, landing = bj.build_course()
    us, vs, tops, parts = footprint(inrun, landing, 8.0)
    pool_end = landing.end()[0]
    ahead = np.arange(pool_end + SEA_AHEAD[0], pool_end + SEA_AHEAD[1], 50.0)
    results = []
    for px in np.arange(box["x0"] + 100, box["x1"] - 100, POSITION_STEP):
        for py in np.arange(box["y0"] + 100, box["y1"] - 100, POSITION_STEP):
            for deg in range(0, 360, HEADING_STEP):
                a = math.radians(deg)
                c, s = math.cos(a), math.sin(a)
                g = lookup(surface, ox, oy, step, px + us * c - vs * s, py + us * s + vs * c)
                if g.max() > 1e5:
                    continue
                base = float((g - tops).max()) + 0.3
                if sea is not None:
                    base = max(base, sea + WATER_MARGIN)
                post = tops + base - g
                land = post[parts == "landing"].mean()
                score = land + 0.5 * post[parts == "inrun"].mean()
                water = 0.0
                if sea is not None:
                    t = lookup(terrain, ox, oy, step, px + ahead * c, py + ahead * s)
                    water = float((t < sea).mean())
                    score += 0.03 * (base - sea) - 25 * water
                results.append((score, float(px), float(py), deg, base, land, water))
    results.sort()
    picked = []
    for r in results:
        if all(math.hypot(r[1] - p[1], r[2] - p[2]) > 400 for p in picked):
            picked.append(r)
        if len(picked) == 5:
            break
    return picked


def close_check(bng: BeamNGpy, level: str, x: float, y: float, heading: float,
                ray_top: float) -> float:
    """Exact base height for a spot, from rays every CHECK_STEP metres. Rays
    start at ray_top, just above the highest ground: ground_heights' rays reach
    2000 m, and from far higher they stopped in mid-air."""
    inrun, landing = bj.build_course()
    us, vs, tops, _ = footprint(inrun, landing, CHECK_STEP)
    site = bj.Site(level, x, y, heading, 0.0, flat_floor=False)
    pts = [(*site.world(u, v)[:2], ray_top) for u, v in zip(us, vs)]
    ground = []
    for i in range(0, len(pts), 400):  # keep each Lua chunk a sensible size
        ground += bj.ground_heights(bng, pts[i:i + 400])
    return max(g - t for g, t in zip(ground, tops)) + 0.3


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("map", help="Level name, e.g. jungle_rock_island, west_coast_usa, utah")
    p.add_argument("--name", help="Name to save the spot under (default: the map name)")
    p.add_argument("--pick", type=int, default=1, help="Which of the top 5 spots to save (default 1)")
    p.add_argument("--host", default=default_host())
    p.add_argument("--port", type=int, default=int(os.environ.get("BNG_PORT", "25252")))
    return p.parse_args()


def main() -> None:
    args = parse_args()
    bng = BeamNGpy(args.host, args.port)
    bng.open(launch=False)
    try:
        t0 = time.monotonic()
        box = load_map(bng, args.map)
        sea = box["sea"]
        print(f"{args.map}: map x {box['x0']:.0f}..{box['x1']:.0f}, y {box['y0']:.0f}..{box['y1']:.0f}; "
              f"sea level {sea if sea is not None else 'none'}")
        surface, terrain, ox, oy, step = sample_grid(bng, box)
        picked = score_all(surface, terrain, ox, oy, step, box, sea)
        if not picked:
            raise SystemExit("no spot fits: the map is too small or too steep")
        print(f"Best spots ({time.monotonic() - t0:.0f} s):")
        for i, (score, x, y, deg, base, land, water) in enumerate(picked, 1):
            print(f"  {i}. start ({x:6.0f},{y:6.0f}) heading {deg:3d}  outrun at z {base:6.1f}  "
                  f"posts under landing avg {land:4.1f} m  water ahead {water * 100:3.0f}%")
        _, x, y, deg, _, _, _ = picked[min(max(args.pick, 1), len(picked)) - 1]
        base = close_check(bng, args.map, x, y, deg, float(surface[surface > -9000].max()) + 50)
        if sea is not None:
            base = max(base, sea + WATER_MARGIN)
        name = args.name or args.map
        saved = json.loads(bj.SITES_FILE.read_text()) if bj.SITES_FILE.exists() else {}
        saved[name] = {"level": args.map, "x": x, "y": y, "heading": float(deg), "base_z": round(base, 1)}
        bj.SITES_FILE.write_text(json.dumps(saved, indent=2) + "\n")
        print(f"Saved spot {args.pick} as '{name}' (outrun at z {base:.1f} after the close check).")
        print(f"Run it with: uv run beamng-bigjump --no-launch --type {name}")
        site = bj.Site(args.map, x, y, deg, base, flat_floor=False)
        pos, aim = bj.OVERVIEW_CAMERA
        bng.camera.set_free(site.world(*pos), site.direction(*aim))
    finally:
        bng.disconnect()
