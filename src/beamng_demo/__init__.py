"""Small BeamNGpy demo: spawn a car, let the AI drive, print its speed."""

import argparse
import os
import socket
import struct
import time

from beamngpy import BeamNGpy, Scenario, Vehicle
from beamngpy.sensors import Electrics


def default_host() -> str:
    """BNG_HOST if set; else localhost, or under WSL the Windows host if that answers.

    With WSL's default NAT networking the game on Windows is reachable only at the
    WSL default gateway, not at localhost; with mirrored networking localhost works.
    """
    if os.environ.get("BNG_HOST"):
        return os.environ["BNG_HOST"]
    port = int(os.environ.get("BNG_PORT", "25252"))
    candidates = ["localhost"]
    try:
        with open("/proc/net/route") as f:
            for line in f.readlines()[1:]:
                fields = line.split()
                if fields[1] == "00000000":  # default route; gateway is little-endian hex
                    candidates.append(socket.inet_ntoa(struct.pack("<L", int(fields[2], 16))))
                    break
    except OSError:
        pass
    for host in candidates:
        try:
            with socket.create_connection((host, port), timeout=0.5):
                return host
        except OSError:
            continue
    return "localhost"


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
    p.add_argument("--seconds", type=int, default=20, help="How long to watch the car")
    p.add_argument("--speed", type=float, default=15,
                   help="AI speed limit in m/s (15 is about 54 km/h)")
    p.add_argument("--aggression", type=float, default=None,
                   help="AI driving style, 0.3 (careful) to 1.0 (racing)")
    p.add_argument("--model", default="etk800",
                   help="Vehicle model id, for example etk800, van, pickup, semi")
    p.add_argument("--config", default=None,
                   help="Part config, for example vehicles/van/<name>.pc")
    p.add_argument("--list-vehicles", action="store_true",
                   help="Print the models and configs the game has, then exit")
    return p.parse_args()


def list_vehicles(bng: BeamNGpy) -> None:
    available = bng.vehicles.get_available().get("vehicles", {})
    for model, info in sorted(available.items()):
        print(f"{model}  ({info.get('name', '')})")
        configs = info.get("configurations", {})
        for cfg in sorted(configs.values(), key=lambda c: c.get("pc_file_path", "")):
            path = cfg.get("pc_file_path", "").lstrip("/")
            print(f"    {path:<50} {cfg.get('name', '')}")


def main() -> None:
    args = parse_args()

    bng = BeamNGpy(args.host, args.port, home=args.home)
    bng.open(launch=not args.no_launch)

    try:
        if args.list_vehicles:
            list_vehicles(bng)
            return

        scenario = Scenario("west_coast_usa", "beamng_demo")
        car = Vehicle("ego", model=args.model, license="DEMO", part_config=args.config)
        car.sensors.attach("electrics", Electrics())
        scenario.add_vehicle(car, pos=(-717, 101, 118), rot_quat=(0, 0, 0.3826834, 0.9238795))
        scenario.make(bng)

        bng.scenario.load(scenario)
        bng.scenario.start()

        car.ai.set_mode("span")  # the AI drives around the map
        car.ai.set_speed(args.speed, mode="limit")
        if args.aggression is not None:
            car.ai.set_aggression(args.aggression)

        for t in range(args.seconds):
            car.sensors.poll()
            e = car.sensors["electrics"]
            print(f"t={t:3d}s  speed={e.get('wheelspeed', 0) * 3.6:6.1f} km/h  "
                  f"rpm={e.get('rpm', 0):6.0f}  gear={e.get('gear')}")
            time.sleep(1)
    finally:
        bng.disconnect()  # leave the game running so you can look at it
