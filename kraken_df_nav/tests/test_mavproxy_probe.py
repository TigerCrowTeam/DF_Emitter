from kraken_df_nav.mavproxy_telemetry_probe import REQUIRED


def test_m1_probe_requires_complete_vehicle_telemetry():
    assert REQUIRED == ("HEARTBEAT", "GLOBAL_POSITION_INT", "GPS_RAW_INT", "ATTITUDE")
