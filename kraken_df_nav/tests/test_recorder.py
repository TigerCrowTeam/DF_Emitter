from kraken_df_nav.df.observation import Observation
from kraken_df_nav.logging_utils.recorder import MissionRecorder
from kraken_df_nav.logging_utils.replay import load_observations


def test_concurrent_recorders_receive_distinct_paths(tmp_path):
    first = MissionRecorder(str(tmp_path))
    second = MissionRecorder(str(tmp_path))
    assert first.prefix != second.prefix


def test_recorder_json_and_csv_replay_round_trip(tmp_path):
    recorder = MissionRecorder(str(tmp_path), save_json=True, save_csv=True)
    observation = Observation.now(
        latitude=37.0, longitude=-122.0, altitude_m=10,
        vehicle_heading_deg=15, relative_bearing_deg=30,
        absolute_bearing_deg=45, rssi_dbm=-62, confidence=.91,
        accepted=True, position_accuracy_m=1.2, heading_accuracy_deg=.8,
    )
    recorder.observation(observation)
    recorder.set_final_report(status="COMPLETE")
    recorder.flush()

    json_rows = load_observations(str(recorder.prefix.with_suffix(".json")))
    csv_rows = load_observations(str(recorder.prefix.with_suffix(".csv")))
    assert json_rows == [observation]
    assert csv_rows == [observation]
