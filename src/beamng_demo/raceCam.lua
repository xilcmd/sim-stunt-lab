-- raceCam: TV-style free camera for beamng-race.
-- Runs every frame inside the game, so the camera moves smoothly. Python only
-- picks the shot (raceCam.setShot) and reads car positions (raceCam.states).

local M = {}

local vids = {}
local towers = {}
local mode, target = nil, nil
local snap = true
local camPos, camAim = nil, nil
local orbit = 0

local function carPos(vid)
  local v = scenetree.findObject(vid)
  if v then return v:getPosition(), v:getDirectionVector() end
end

local function ease(rate, dt)
  return 1 - math.exp(-rate * dt)
end

M.setup = function(vidList, towerList)
  vids = vidList
  towers = {}
  for _, t in ipairs(towerList) do
    table.insert(towers, vec3(t[1], t[2], t[3]))
  end
  commands.setFreeCamera()
end

M.setShot = function(shotMode, vid)
  mode, target, snap = shotMode, vid, true
end

-- Film the target car from one fixed spot, for example beside a hill crest.
local spotPos = nil
M.setSpot = function(x, y, z, vid)
  spotPos = vec3(x, y, z)
  mode, target, snap = 'spot', vid, true
end

M.stop = function()
  mode = nil
end

-- "car1 x y z;car2 x y z;...;T secs" in one round trip. T is the replay
-- recording time (0 when not recording), used to mark highlights.
M.states = function()
  local out = {}
  for _, vid in ipairs(vids) do
    local p = carPos(vid)
    if p then table.insert(out, string.format('%s %.2f %.2f %.2f', vid, p.x, p.y, p.z)) end
  end
  local t = 0
  if core_replay and core_replay.getState() == 'recording' then t = core_replay.getTotalSeconds() or 0 end
  table.insert(out, string.format('T %.2f', t))
  return table.concat(out, ';')
end

M.onPreRender = function(dt)
  if not mode or not target then return end
  local p, d = carPos(target)
  if not p then return end
  dt = math.min(dt or 0.016, 0.1)

  local wantPos, wantAim, kPos, kAim
  if mode == 'spot' and spotPos then
    wantPos, wantAim, kPos, kAim = spotPos, p, 10.0, 10.0
  elseif mode == 'tv' then
    local best, bestD = towers[1], math.huge
    for _, t in ipairs(towers) do
      local dd = (t - p):squaredLength()
      if dd < bestD then best, bestD = t, dd end
    end
    wantPos, wantAim, kPos, kAim = best, p, 1.0, 10.0
  elseif mode == 'chase' then
    local f = vec3(d.x, d.y, 0):normalized()
    wantPos = p - f * 14 + vec3(0, 0, 4.5)
    wantAim = p + f * 12 + vec3(0, 0, 1)
    kPos, kAim = 6.0, 8.0
  else -- blimp: high orbit around the middle of the pack
    local c, n = vec3(0, 0, 0), 0
    for _, vid in ipairs(vids) do
      local q = carPos(vid)
      if q then c = c + q; n = n + 1 end
    end
    c = c / math.max(n, 1)
    orbit = orbit + dt * 0.12
    wantPos = c + vec3(140 * math.cos(orbit), 140 * math.sin(orbit), 90)
    wantAim = vec3(c.x, c.y, 0)
    kPos, kAim = 1.5, 3.0
  end

  if snap or not camPos then
    camPos, camAim, snap = wantPos, wantAim, false
  else
    camPos = camPos + (wantPos - camPos) * ease(kPos, dt)
    camAim = camAim + (wantAim - camAim) * ease(kAim, dt)
  end
  local rot = quatFromDir((camAim - camPos):normalized())
  core_camera.setPosRot(0, camPos.x, camPos.y, camPos.z, rot.x, rot.y, rot.z, rot.w)
end

return M
