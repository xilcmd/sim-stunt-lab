"""Highlight reel: mark exciting moments during the race, then replay them.

The race is recorded with the game's replay system (core_replay). Each moment is
stored with its replay time, so after the race the reel can jump straight to it
and film it with raceCam. The game cannot export video, so the viewer records the
screen (Windows Game Bar: Win+Alt+R, or OBS) while the reel plays.
"""

import math
import time
from dataclasses import dataclass

from beamngpy import BeamNGpy

BEFORE = 6.0          # seconds of build-up shown before each moment
AFTER = 5.0           # seconds shown after it
PASS_HOLD = 2.0       # a new order must hold this long to count as a pass
BATTLE_DISTANCE = 3.0  # metres between two cars that counts as wheel to wheel
BATTLE_COOLDOWN = 20.0  # seconds before the same pair can make another battle
START_LENGTH = 10.0   # seconds of the green flag shown at the top of the reel
MIN_CLIP = 4.0        # shortest clip, so a caption stays up long enough to read


@dataclass
class Moment:
    t: float          # replay seconds
    kind: str         # start, pass, battle, crash, crest, finish
    focus: str        # vid the camera follows
    caption: str
    spot: tuple[float, float, float] | None = None  # fixed camera position, if any


class Spotter:
    """Watches the race each tick and notes moments worth replaying."""

    def __init__(self, labels: dict[str, str], show_laps: bool = True):
        self.labels = labels
        self.show_laps = show_laps
        self.moments: list[Moment] = []
        self.stable: tuple[str, ...] = ()
        self.candidate: tuple[str, ...] = ()
        self.candidate_since = 0.0
        self.battle_last: dict[tuple[str, str], float] = {}

    def name(self, vid: str) -> str:
        return self.labels.get(vid, vid.upper())

    def tick(self, rt: float, running: list[str], positions: dict, laps: dict) -> None:
        """rt: replay time; running: cars still racing, leader first."""
        order = tuple(running)
        if not self.stable:
            self.stable = order
            self.moments.append(Moment(rt, "start", order[0] if order else "", "Green flag!"))
        if order != self.candidate:
            self.candidate, self.candidate_since = order, rt
        elif order != self.stable and rt - self.candidate_since >= PASS_HOLD:
            self._note_pass(self.candidate_since, self.stable, order, laps)
            self.stable = order

        vids = [v for v in running if v in positions]
        for i, a in enumerate(vids):
            for b in vids[i + 1:]:
                pa, pb = positions[a], positions[b]
                if math.hypot(pa[0] - pb[0], pa[1] - pb[1]) < BATTLE_DISTANCE:
                    key = tuple(sorted((a, b)))
                    if rt - self.battle_last.get(key, -1e9) > BATTLE_COOLDOWN:
                        self.battle_last[key] = rt
                        self.moments.append(Moment(
                            rt, "battle", a,
                            f"{self.name(a)} and {self.name(b)} side by side!"))

    def _note_pass(self, rt, old, new, laps) -> None:
        old = [v for v in old if v in new]  # ignore cars that dropped out
        for i, vid in enumerate(new):
            if i < len(old) and old[i] != vid and vid in old and old.index(vid) > i:
                passed = old[i]
                lap = f"Lap {int(laps.get(vid, 0)) + 1} · " if self.show_laps else ""
                self.moments.append(Moment(
                    rt, "pass", vid,
                    f"{lap}{self.name(vid)} passes {self.name(passed)} for P{i + 1}"))
                return

    def crash(self, rt: float, vid: str, caption: str = "crashes out!") -> None:
        self.moments.append(Moment(rt, "crash", vid, f"{self.name(vid)} {caption}"))

    def finish(self, rt: float, vid: str, place: int) -> None:
        if place == 1:
            self.moments.append(Moment(rt, "finish", vid, f"{self.name(vid)} wins!"))

    def crest(self, rt: float, vid: str, spot: tuple[float, float, float]) -> None:
        """The leader goes over a hill crest, filmed from `spot`."""
        self.moments.append(Moment(rt, "crest", vid, f"{self.name(vid)} over the top!", spot))


RANK = {"start": 0, "battle": 1, "pass": 2, "crest": 2, "crash": 3, "finish": 4}


def build_reel(moments: list[Moment], total: float) -> list[tuple[float, float, Moment]]:
    """Clip windows (start, end, moment) in time order.

    Where two clips would overlap, the first is cut short and the next starts
    where it ends, so playback runs on without a jump and each moment keeps its
    own caption and camera. Moments under a second apart keep only the bigger one.
    """
    clips: list[tuple[float, float, Moment]] = []
    for m in sorted(moments, key=lambda m: m.t):
        if clips and m.t - clips[-1][2].t < 1.0:
            if RANK[m.kind] > RANK[clips[-1][2].kind]:
                clips[-1] = (clips[-1][0], clips[-1][1], m)
            continue
        if m.kind == "start":
            start, end = 0.0, START_LENGTH
        else:
            start, end = max(0.0, m.t - BEFORE), min(total, m.t + AFTER)
        if clips and start < clips[-1][1]:
            prev_start, prev_end, prev = clips[-1]
            cut = max(prev_start + MIN_CLIP, min(prev_end, m.t - 2.0))
            clips[-1] = (prev_start, cut, prev)
            start, end = cut, max(end, cut + MIN_CLIP)
        clips.append((start, end, m))
    return clips


SHOTS = {"start": ("blimp",), "battle": ("tv", "chase"), "pass": ("chase", "tv"),
         "crash": ("tv",), "crest": ("tv",), "finish": ("tv", "chase")}
# The same without the high orbiting blimp shot.
SHOTS_NO_BLIMP = {**SHOTS, "start": ("tv", "chase")}


def play_reel(bng: BeamNGpy, replay_file: str, moments: list[Moment], countdown: float,
              shots_by_kind: dict[str, tuple[str, ...]] = SHOTS) -> None:
    def q(chunk: str):
        return bng.control.queue_lua_command(chunk, response=True)

    q(f"core_replay.loadFile('{replay_file}', false)")
    for _ in range(50):
        if q("return core_replay.getState()") == "playback":
            break
        time.sleep(0.2)
    total = float(q("return tostring(core_replay.getTotalSeconds())"))
    clips = build_reel(moments, total)
    print(f"Highlight reel: {len(clips)} clips, "
          f"{sum(end - start for start, end, _ in clips):.0f} s from a {total:.0f} s race")

    q("core_replay.pause(true)")
    q("core_replay.seek(0)")
    bng.ui.display_message(f"Highlight reel starts in {countdown:.0f} s. "
                           "Start screen recording now (Win+Alt+R).")
    time.sleep(countdown)

    played_to = None
    for start, end, m in clips:
        print(f"  {start:6.1f}-{end:6.1f} s  {m.caption}")
        if played_to is None or abs(start - played_to) > 0.5:  # back-to-back clips run on
            q(f"core_replay.seek({start / total:.5f})")
        played_to = end
        shots = shots_by_kind[m.kind]
        if m.spot:
            shots = ("spot",)
            q(f"raceCam.setSpot({m.spot[0]:.2f}, {m.spot[1]:.2f}, {m.spot[2]:.2f}, '{m.focus}')")
        else:
            q(f"raceCam.setShot('{shots[0]}', '{m.focus}')")
        bng.ui.display_message(m.caption)
        q("core_replay.pause(false)")
        length = end - start
        for i, shot in enumerate(shots[1:], 1):
            time.sleep(length / len(shots))
            q(f"raceCam.setShot('{shot}', '{m.focus}')")
        time.sleep(length / len(shots))
        q("core_replay.pause(true)")

    bng.ui.display_message("That's the race! Stop screen recording now (Win+Alt+R).")
    time.sleep(3)
    q("core_replay.stopAndUnload()")
