from dataclasses import asdict

import pytest

from kraken_df_nav.config import DigitalTwinConfig, load_config
from kraken_df_nav.df.geometry import GeoPoint
from kraken_df_nav.simulation.kraken_simulator import SimulatedBearingSource, SimulatedVehicleController
from kraken_df_nav.simulation.transmitter import SimulatedTransmitter


def make_source(effects=None, seed=11, noise=0.0):
    vehicle = SimulatedVehicleController(GeoPoint(37.0, -122.0, 20), heading_deg=0)
    transmitter = SimulatedTransmitter(37.001, -122.0)
    return SimulatedBearingSource(transmitter, vehicle, noise_std_deg=noise, seed=seed,
                                  effects=effects, clock=lambda: 1_000.0), vehicle


def test_same_seed_and_config_reproduce_measurement_values():
    effects = DigitalTwinConfig(correlated_noise_std_deg=2, correlated_noise_coefficient=.7,
                                range_noise_deg_per_km=3)
    first, _ = make_source(effects, noise=1.5)
    second, _ = make_source(DigitalTwinConfig(**asdict(effects)), noise=1.5)
    assert [first.get_measurement() for _ in range(4)] == [second.get_measurement() for _ in range(4)]


def test_bias_drift_and_truth_audit_are_separate_from_measurement():
    source, _ = make_source(DigitalTwinConfig(constant_bias_deg=5, drift_deg_per_sample=2))
    first, second = source.get_measurement(), source.get_measurement()
    assert first.relative_bearing_deg == pytest.approx(5)
    assert second.relative_bearing_deg == pytest.approx(7)
    assert source.records[0]["clean_absolute_bearing_deg"] == pytest.approx(0)
    assert "transmitter_latitude" in source.records[0]
    assert not hasattr(first, "transmitter_latitude")


def test_intermediate_and_burst_dropouts_are_recorded():
    effects = DigitalTwinConfig(dropout_probability=1)
    source, _ = make_source(effects)
    assert source.get_measurement() is None
    assert source.records[-1]["reason"] == "intermittent dropout"

    effects = DigitalTwinConfig(burst_dropout_probability=1, burst_dropout_samples=3)
    source, _ = make_source(effects)
    assert [source.get_measurement() for _ in range(3)] == [None, None, None]
    assert all(record["outcome"] == "dropout" for record in source.records)


def test_delay_stale_frequency_and_confidence_effects_combine():
    effects = DigitalTwinConfig(constant_bias_deg=10, delay_seconds=2, stale_probability=1,
        stale_age_seconds=20, wrong_frequency_probability=1, wrong_frequency_offset_hz=5000,
        confidence_error_scale_deg=20)
    source, _ = make_source(effects)
    measurement = source.get_measurement()
    assert measurement.timestamp == 978
    assert measurement.frequency_hz == 915_005_000
    assert measurement.confidence < .5


def test_duplicate_frozen_and_malformed_outputs():
    source, _ = make_source(DigitalTwinConfig())
    first = source.get_measurement()
    source.effects.duplicate_probability = 1
    assert source.get_measurement() is first
    assert source.records[-1]["outcome"] == "duplicate"

    frozen, vehicle = make_source(DigitalTwinConfig(frozen_probability=1))
    initial = frozen.get_measurement()
    vehicle.heading = 90
    repeated = frozen.get_measurement()
    assert repeated.relative_bearing_deg == initial.relative_bearing_deg

    malformed, _ = make_source(DigitalTwinConfig(malformed_probability=1))
    with pytest.raises(ValueError, match="malformed Kraken"):
        malformed.get_measurement()
    assert malformed.records[-1]["outcome"] == "malformed"


def test_outlier_burst_and_persistent_multipath_are_audited():
    effects = DigitalTwinConfig(outlier_burst_probability=1, outlier_burst_samples=2,
        outlier_magnitude_deg=80, multipath_probability=1, multipath_bias_deg=25,
        multipath_persistence_samples=2)
    source, _ = make_source(effects)
    source.get_measurement()
    effects.outlier_burst_probability = 0
    effects.multipath_probability = 0
    source.get_measurement()
    for record in source.records:
        assert record["effects"]["outlier_deg"] == 80
        assert record["effects"]["multipath_deg"] == 25


def test_movement_and_attitude_reduce_confidence_and_add_error():
    effects = DigitalTwinConfig(moving_bearing_penalty_deg=6, moving_confidence_penalty=.2,
        attitude_bearing_penalty_deg=9, attitude_confidence_penalty=.3)
    source, vehicle = make_source(effects)
    baseline = source.get_measurement()
    vehicle.point = GeoPoint(37.0001, -122.0, 20)
    vehicle.target = GeoPoint(37.001, -122.0, 20)
    original_get_state = vehicle.get_vehicle_state
    vehicle.get_vehicle_state = lambda: type(original_get_state())(
        **{**asdict(original_get_state()), "roll_deg": 45})
    affected = source.get_measurement()
    assert source.records[-1]["moving"] is True
    assert source.records[-1]["effects"]["moving_penalty_deg"] == 6
    assert source.records[-1]["effects"]["attitude_penalty_deg"] == 9
    assert affected.confidence < baseline.confidence


def test_nested_digital_twin_config_loads_and_validates(tmp_path):
    config_path = tmp_path / "config.yaml"
    config_path.write_text("simulation:\n  digital_twin:\n    constant_bias_deg: 4\n    dropout_probability: 0.2\n",
                           encoding="utf-8")
    config = load_config(config_path)
    assert config.simulation.digital_twin.constant_bias_deg == 4
    assert config.simulation.digital_twin.dropout_probability == .2

    config_path.write_text("simulation:\n  digital_twin:\n    dropout_probability: 1.2\n", encoding="utf-8")
    with pytest.raises(ValueError, match="probabilities"):
        load_config(config_path)


def test_bias_plus_noise_combination_is_reproducible_and_audited():
    effects = DigitalTwinConfig(constant_bias_deg=6)
    source, _ = make_source(effects, seed=4, noise=2)
    source.get_measurement()
    components = source.records[-1]["effects"]
    assert components["constant_bias_deg"] == 6
    assert components["gaussian_noise_deg"] != 0
    assert source.records[-1]["total_bearing_error_deg"] == pytest.approx(sum(components.values()))


def test_dropout_plus_delay_preserves_both_configured_behaviors():
    effects = DigitalTwinConfig(dropout_probability=1, delay_seconds=7)
    source, _ = make_source(effects)
    assert source.get_measurement() is None
    effects.dropout_probability = 0
    assert source.get_measurement().timestamp == 993


def test_multipath_plus_heading_error_combination():
    effects = DigitalTwinConfig(heading_error_deg=4, multipath_probability=1,
                                multipath_bias_deg=30, multipath_persistence_samples=2)
    source, _ = make_source(effects)
    measurement = source.get_measurement()
    assert measurement.relative_bearing_deg == pytest.approx(34)
    assert source.records[-1]["effects"]["heading_error_deg"] == 4
    assert source.records[-1]["effects"]["multipath_deg"] == 30
