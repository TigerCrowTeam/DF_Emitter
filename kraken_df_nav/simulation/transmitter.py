from dataclasses import dataclass


@dataclass(frozen=True)
class SimulatedTransmitter:
    latitude: float
    longitude: float
    power_dbm: float = 20.0
