from kraken_df_nav.config import AppConfig
from kraken_df_nav.df.estimator import WeightedLeastSquaresEstimator
from kraken_df_nav.df.geometry import GeoPoint, distance_m
from kraken_df_nav.logging_utils.recorder import NullRecorder
from kraken_df_nav.mission.observation_planner import ObservationPlanner
from kraken_df_nav.mission.state_machine import Mission, MissionState
from kraken_df_nav.simulation.kraken_simulator import SimulatedBearingSource, SimulatedVehicleController
from kraken_df_nav.simulation.transmitter import SimulatedTransmitter


def test_end_to_end_simulation_converges():
    config = AppConfig()
    config.navigation.stabilization_seconds = 0
    tx = SimulatedTransmitter(37.0008, -121.9992)
    vehicle = SimulatedVehicleController(GeoPoint(37, -122, 60))
    source = SimulatedBearingSource(tx, vehicle, noise_std_deg=2, seed=7)
    mission = Mission(config, source, vehicle, WeightedLeastSquaresEstimator(),
                      ObservationPlanner(60, 100, 500, 200), NullRecorder())
    result = mission.run(max_ticks=100, tick_seconds=0)
    assert mission.state == MissionState.COMPLETE
    assert len(mission.observations) == 7
    assert len([event for event in mission.recorder.events if event["type"] == "waypoint"]) == 6
    assert mission.recorder.final_report is not None
    assert mission.recorder.final_report["accepted_observations"] == 7
    assert result is not None and result.uncertainty_radius_m <= 10
    assert distance_m(GeoPoint(tx.latitude, tx.longitude), GeoPoint(result.latitude, result.longitude)) <= 10
