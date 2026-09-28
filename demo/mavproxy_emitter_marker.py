"""MAVProxy module that plots a finalized DF estimate as a red antenna."""
from __future__ import annotations

import json
import os
from pathlib import Path
import time

from MAVProxy.modules.lib import mp_module


class EmitterMarkerModule(mp_module.MPModule):
    def __init__(self, mpstate):
        super().__init__(mpstate, "emitter_marker", "plots the final DF emitter estimate")
        self.marker_file = Path(os.environ.get("KRAKEN_DF_MARKER_FILE", "emitter_marker.json"))
        self.last_check = 0.0
        self.plotted = False
        self.detected_announced = False

    def idle_task(self):
        if self.plotted or time.monotonic()-self.last_check < 0.25:
            return
        self.last_check = time.monotonic()
        if not self.marker_file.is_file():
            return
        try:
            payload = json.loads(self.marker_file.read_text(encoding="utf-8"))
            if payload.get("phase") == "EMITTER_DETECTED" and not self.detected_announced:
                print("RF EMITTER DETECTED - STARTING DF SEARCH")
                self.detected_announced = True
                return
            latitude, longitude = float(payload["latitude"]), float(payload["longitude"])
        except (OSError, ValueError, KeyError, json.JSONDecodeError):
            return
        map_module = self.module("map")
        if map_module is None:
            return
        map_module.cmd_map(["icon", str(latitude), str(longitude), "redantenna"])
        print(f"DF EMITTER FOUND: red antenna at {latitude:.7f}, {longitude:.7f}")
        self.plotted = True


def init(mpstate):
    return EmitterMarkerModule(mpstate)
