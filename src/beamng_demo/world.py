"""Pieces shared by the races: camera, car control, and building boxes in the world."""

import json
import math
import platform
import random
import struct
import subprocess
import zlib
from importlib import resources
from pathlib import Path

from beamngpy import BeamNGpy, Vehicle, angle_to_quat

# Box = (name, size (x, y, z), material, centre pos, rotation quat (x, y, z, w)).
Box = tuple[str, tuple[float, float, float], str, tuple[float, float, float],
            tuple[float, float, float, float]]
# Material = (name, colour "r g b a", texture path or None).
Material = tuple[str, str, str | None]


def quat_yaw_pitch(yaw: float, pitch: float) -> tuple[float, float, float, float]:
    """Rotation that tips local +x down by `pitch`, then turns it to `yaw` (radians).

    Usual maths order (yaw quaternion times pitch quaternion). build_boxes
    sends the inverse, because the game applies the inverse of the rotation
    it is given.
    """
    qz = (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))
    qy = (0.0, math.sin(pitch / 2), 0.0, math.cos(pitch / 2))
    x1, y1, z1, w1 = qz
    x2, y2, z2, w2 = qy
    return (w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2)


def spawn_rotation(heading: float, downhill: float = 0.0) -> tuple[float, float, float, float]:
    """Rotation for a vehicle to face `heading` (radians, 0 = +x), on a slope
    `downhill` radians steep.

    A yaw of heading - 90 degrees faces the car along the heading. The races
    first used heading + 90, which spawned every car facing exactly backward:
    the AI had to turn each one round at the start, and the chase camera sat in
    front of it. Checked by driving and with getDirectionVector.
    """
    # The yaw is mirrored like box rotations: heading - 90 faced -heading (a
    # car on the Italy course, heading 30 degrees, stood at -30). The two only
    # agree at heading 0, where the oval and soapbox grids sit.
    return angle_to_quat((0, -math.degrees(downhill), -math.degrees(heading) - 90))


def png_bytes(width: int, height: int, pixel) -> bytes:
    """An RGB PNG; pixel(x, y) returns (r, g, b) as 0..255 ints."""
    rows = b"".join(b"\x00" + bytes(c for x in range(width) for c in pixel(x, y))
                    for y in range(height))

    def chunk(kind, data):
        return (struct.pack(">I", len(data)) + kind + data
                + struct.pack(">I", zlib.crc32(kind + data) & 0xFFFFFFFF))
    return (b"\x89PNG\r\n\x1a\n"
            + chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0))
            + chunk(b"IDAT", zlib.compress(rows, 9)) + chunk(b"IEND", b""))


def user_folder(bng: BeamNGpy) -> Path:
    """The game's user folder as a path this machine can write to."""
    win = str(bng.control.queue_lua_command("return FS:getUserPath()", response=True))
    if platform.system() == "Linux" and ":" in win:  # WSL: game on Windows
        return Path(subprocess.check_output(["wslpath", "-u", win], text=True).strip())
    return Path(win)


def install_art(bng: BeamNGpy, art: str, textures: dict, materials: dict[str, str],
                size: int = 256) -> None:
    """Write textures and a materials file into a map folder of the user folder.

    `art` is a game path such as /levels/smallgrid/art/soapbox; `textures` maps
    file names to pixel functions, `materials` maps material names to texture
    files. The game loads every *.materials.json under a map's folder when the
    map loads, so mesh roads can use these materials from the start.
    """
    folder = user_folder(bng) / art.lstrip("/")
    folder.mkdir(parents=True, exist_ok=True)
    for name, pixel in textures.items():
        path = folder / name
        if not path.exists():
            path.write_bytes(png_bytes(size, size, pixel))
    data = {
        name: {"name": name, "mapTo": name, "class": "Material",
               "Stages": [{"colorMap": f"{art}/{tex}"}, {}, {}, {}],
               "materialTag0": "beamng"}
        for name, tex in materials.items()}
    (folder / f"{folder.name}.materials.json").write_text(json.dumps(data, indent=2))


def install_camera(bng: BeamNGpy, vids: list[str],
                   towers: list[tuple[float, float, float]]) -> None:
    """Copy raceCam.lua into the game's user folder, load it, and give it the cars.

    Each API call waits about one game frame, so a camera driven from Python
    stutters. The extension moves the camera every frame instead. `towers` are
    the fixed spots the TV shot films from.
    """
    src = (resources.files(__package__) / "raceCam.lua").read_text()
    assert "]==]" not in src
    bng.control.queue_lua_command(
        f"writeFile('/lua/ge/extensions/raceCam.lua', [==[{src}]==])\n"
        "extensions.unload('raceCam')\n"
        "extensions.load('raceCam')")
    lua_towers = ", ".join(f"{{{x}, {y}, {z}}}" for x, y, z in towers)
    lua_vids = ", ".join(f"'{v}'" for v in vids)
    bng.control.queue_lua_command(f"raceCam.setup({{{lua_vids}}}, {{{lua_towers}}})")


class Director:
    """Picks the next shot and target every few seconds; raceCam does the moving."""

    SHOTS = ("tv", "chase", "blimp")

    def __init__(self, bng: BeamNGpy, shot_seconds: float, shots: tuple[str, ...] = SHOTS):
        self.bng = bng
        self.shot_seconds = shot_seconds
        self.SHOTS = shots
        self.held: str | None = None
        self.shot_i = -1
        self.shot_started = -1e9
        self.target = None

    def tick(self, now: float, order: list[str]) -> None:
        if now - self.shot_started < self.shot_seconds:
            return
        self.held = None
        self.shot_i = (self.shot_i + 1) % len(self.SHOTS)
        self.shot_started = now
        # Mostly follow the leader, sometimes someone in the pack.
        self.target = order[0] if random.random() < 0.5 else random.choice(order)
        self.bng.control.queue_lua_command(
            f"raceCam.setShot('{self.SHOTS[self.shot_i]}', '{self.target}')")

    def hold(self, now: float, describe: str) -> None:
        """Someone else has the camera; start a fresh shot when tick() runs again."""
        self.shot_started = now - self.shot_seconds
        self.held = describe

    def describe(self) -> str:
        if self.held:
            return self.held
        return f"{self.SHOTS[self.shot_i]} on {self.target}"


ENGINE_OFF = 'local e = powertrain.getDevice("mainEngine") if e then e:disable() end'


def park(car: Vehicle) -> None:
    """AI off, parking brake on, engine off, so a wrecked car stops revving.

    Ignition-off calls do not hold on a wrecked car (ignition stays at 2 and the
    engine keeps revving), so switch off the engine device itself.
    """
    car.ai.set_mode("disabled")
    car.queue_lua_command(
        f'input.event("throttle", 0, 1) input.event("parkingbrake", 1, 1) {ENGINE_OFF}')


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


# Mesh = (name, material, origin (x, y, z), vertices, normals, uvs, triangles).
# Vertices and normals are (x, y, z), relative to the origin; uvs are (u, v);
# triangles are index triples into all three lists (0-based).
Mesh = tuple[str, str, tuple[float, float, float], list[tuple[float, float, float]],
             list[tuple[float, float, float]], list[tuple[float, float]],
             list[tuple[int, int, int]]]


def triangles_facing(corners: list[tuple[float, float, float]], normal) -> list[tuple[int, int, int]]:
    """Index triples for a quad (corners 0..3 in order around it) that face `normal`.

    BeamNG treats a triangle as facing its normal when (b - a) x (c - a) points
    the other way (checked against the game's own cube).
    """
    a, b, c = corners[0], corners[1], corners[2]
    u = [b[k] - a[k] for k in range(3)]
    v = [c[k] - a[k] for k in range(3)]
    cross = (u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0])
    if sum(cross[k] * normal[k] for k in range(3)) > 0:
        return [(0, 2, 1), (0, 3, 2)]
    return [(0, 1, 2), (0, 2, 3)]


class MeshBuilder:
    """Collects flat-shaded quads in world coordinates into one Mesh."""

    def __init__(self, name: str, material: str, origin: tuple[float, float, float]):
        self.name, self.material, self.origin = name, material, origin
        self.verts: list = []
        self.normals: list = []
        self.uvs: list = []
        self.tris: list = []

    def quad(self, corners, uvs, outward) -> None:
        """Add a quad (4 corners in order around it) facing roughly `outward`."""
        ox, oy, oz = self.origin
        local = [(p[0] - ox, p[1] - oy, p[2] - oz) for p in corners]
        a, b, d = local[0], local[1], local[3]
        u = [b[k] - a[k] for k in range(3)]
        v = [d[k] - a[k] for k in range(3)]
        n = (u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0])
        if sum(n[k] * outward[k] for k in range(3)) < 0:
            n = tuple(-x for x in n)
        size = math.sqrt(sum(x * x for x in n)) or 1.0
        n = tuple(x / size for x in n)
        base = len(self.verts)
        self.verts.extend(local)
        self.normals.extend([n] * 4)
        self.uvs.extend(uvs)
        self.tris.extend((base + i, base + j, base + k)
                         for i, j, k in triangles_facing(local, n))

    def mesh(self) -> Mesh:
        return (self.name, self.material, self.origin, self.verts, self.normals, self.uvs, self.tris)


def build_meshes(bng: BeamNGpy, meshes: list[Mesh]) -> None:
    """Create custom-shaped ProceduralMesh objects (no collision until the next
    be:reloadCollision(), which build_boxes runs)."""
    bng.control.queue_lua_command("""
rawset(_G, 'raceMeshPut', function(name, mat, ox, oy, oz, V, N, U, T)
  local old = scenetree.findObject(name) if old then old:delete() end
  local mesh = {verts = {}, normals = {}, uvs = {}, faces = {}, material = mat}
  for i = 1, #V, 3 do table.insert(mesh.verts, {x = V[i], y = V[i + 1], z = V[i + 2]}) end
  for i = 1, #N, 3 do table.insert(mesh.normals, {x = N[i], y = N[i + 1], z = N[i + 2]}) end
  for i = 1, #U, 2 do table.insert(mesh.uvs, {u = U[i], v = U[i + 1]}) end
  for i = 1, #T do table.insert(mesh.faces, {v = T[i], n = T[i], u = T[i]}) end
  local proc = createObject('ProceduralMesh')
  proc:registerObject(name)
  proc.canSave = false
  scenetree.MissionGroup:add(proc.obj)
  proc:createMesh({{mesh}})
  proc:setPosition(vec3(ox, oy, oz))
  proc.scale = vec3(1, 1, 1)
end)""")

    def flat(rows, fmt):
        return ",".join(fmt % c for row in rows for c in row)

    for name, mat, (ox, oy, oz), verts, normals, uvs, tris in meshes:
        bng.control.queue_lua_command(
            f"raceMeshPut('{name}', '{mat}', {ox:.3f}, {oy:.3f}, {oz:.3f}, "
            f"{{{flat(verts, '%.3f')}}}, {{{flat(normals, '%.4f')}}}, "
            f"{{{flat(uvs, '%.4f')}}}, {{{flat(tris, '%d')}}})")


# Cone = (name, base radius, height, material, base centre position).
Cone = tuple[str, float, float, str, tuple[float, float, float]]


def build_cones(bng: BeamNGpy, cones: list[Cone]) -> None:
    """Visual-only cones, for example far-off mountains (call after build_boxes)."""
    lua = ["extensions.load('util_trackBuilder_proceduralPrimitives')",
           "local P = util_trackBuilder_proceduralPrimitives",
           "local function cone(name, r, h, mat, x, y, z)",
           "  local old = scenetree.findObject(name) if old then old:delete() end",
           "  local proc = createObject('ProceduralMesh')",
           "  proc:registerObject(name)",
           "  proc.canSave = false",
           "  scenetree.MissionGroup:add(proc.obj)",
           "  proc:createMesh({{P.createCone(r, h, mat)}})",
           "  proc:setPosition(vec3(x, y, z))",
           "  proc.scale = vec3(1, 1, 1)",
           "end"]
    for name, r, h, mat, (x, y, z) in cones:
        lua.append(f"cone('{name}', {r:.1f}, {h:.1f}, '{mat}', {x:.1f}, {y:.1f}, {z:.1f})")
    bng.control.queue_lua_command("\n".join(lua))


def add_water(bng: BeamNGpy, name: str, centre: tuple[float, float, float],
              size: tuple[float, float, float], heading: float = 0.0) -> None:
    """A block of water (a WaterBlock) with its box centred on `centre`,
    turned `heading` degrees about the vertical. The rotation field is applied
    inverted, like box rotations, so the angle is sent negated."""
    x, y, z = centre
    sx, sy, sz = size
    bng.control.queue_lua_command(f"""
local old = scenetree.findObject('{name}') if old then old:delete() end
local w = createObject('WaterBlock')
w:setPosition(vec3({x:.2f}, {y:.2f}, {z:.2f}))
w:setField('rotation', 0, '0 0 1 {-heading:.3f}')
w:setField('scale', 0, '{sx:.2f} {sy:.2f} {sz:.2f}')
w:registerObject('{name}')
scenetree.MissionGroup:add(w.obj)""")


def build_boxes(bng: BeamNGpy, solid: list[Box], visual: list[Box],
                materials: list[Material]) -> None:
    """Create materials, then solid boxes, then visual-only boxes.

    BeamNGpy's ProceduralCube fails on BeamNG.drive: techCore grabs the
    proceduralPrimitives extension before it is loaded. So load it here and
    build the boxes with the same steps techCore uses.

    The game builds its collision data only on be:reloadCollision(), so boxes
    made after that call can be seen but not hit. Even a 1 cm solid step stops a
    car dead, so things like painted lines must be visual.
    """
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
    for name, color, texture in materials:
        lua.append(f"material('{name}', '{color}', {repr(texture) if texture else 'nil'})")
    def inverse(q):
        # The game applies the inverse of the rotation it is given (a probe at
        # +30 degrees yaw came out at -30, tipped up instead of down), so send
        # the inverse to get the rotation we mean.
        return (-q[0], -q[1], -q[2], q[3])

    for name, size, mat, pos, q in solid:
        lua.append(f"put('{name}', {{{nums(size)}}}, '{mat}', {{{nums(pos)}}}, {{{nums(inverse(q))}}})")
    lua.append("be:reloadCollision()")
    for name, size, mat, pos, q in visual:
        lua.append(f"put('{name}', {{{nums(size)}}}, '{mat}', {{{nums(pos)}}}, {{{nums(inverse(q))}}})")
    bng.control.queue_lua_command("\n".join(lua))
