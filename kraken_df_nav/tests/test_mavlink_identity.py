import pytest

from kraken_df_nav.config import AppConfig
from kraken_df_nav.mavproxy_command_probe import _ack_accepted, _validate_target


def test_mavlink_identity_defaults_are_explicit():
    config = AppConfig()
    assert config.mavlink.source_system == 252
    assert config.mavlink.source_component == 191
    assert config.mavlink.expected_target_system is None


@pytest.mark.parametrize("field", ["source_system", "source_component",
                                    "expected_target_system", "expected_target_component"])
def test_mavlink_identity_rejects_zero(field):
    config = AppConfig()
    setattr(config.mavlink, field, 0)
    with pytest.raises(ValueError, match=field):
        config.validate()


def test_m2_target_mismatch_is_rejected():
    with pytest.raises(RuntimeError, match="wrong target 2:1"):
        _validate_target(2, 1)


def test_m2_command_ack_rejection_is_not_accepted():
    assert _ack_accepted({"result": 0})
    assert not _ack_accepted({"result": 4})
