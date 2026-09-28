import socket

import pytest

from kraken_df_nav.config import AppConfig, load_config
from kraken_df_nav.mavproxy_preflight import check_ports


def routed_config() -> AppConfig:
    config = AppConfig()
    config.simulation.use_sitl = True
    config.mavproxy.enabled = True
    config.mavproxy.master_connection = "tcp:127.0.0.1:5760"
    config.mavlink.connection = "udpin:127.0.0.1:14560"
    return config


def test_complete_mavproxy_profiles_load():
    sitl = load_config("kraken_df_nav/config.mavproxy_sitl.yaml")
    live = load_config("kraken_df_nav/config.mavproxy_live.yaml")
    assert sitl.mavproxy.enabled and sitl.simulation.use_sitl
    assert live.mavproxy.enabled and live.mode == "LIVE"
    assert live.mavlink.connection.startswith("udpin:127.0.0.1:")
    assert live.mavproxy.master_connection.startswith("/dev/serial/by-id/")


def test_routed_mode_rejects_direct_autopilot_connection():
    config = routed_config()
    config.mavlink.connection = "/dev/serial/by-id/a-cube"
    with pytest.raises(ValueError, match="must not open the autopilot master"):
        config.validate()


def test_routed_sitl_rejects_non_loopback_master():
    config = routed_config()
    config.mavproxy.master_connection = "tcp:192.0.2.10:5760"
    with pytest.raises(ValueError, match="SITL master must be a loopback"):
        config.validate()


def test_routed_mode_rejects_non_loopback_and_duplicate_ports():
    config = routed_config()
    config.mavproxy.loopback_host = "0.0.0.0"
    with pytest.raises(ValueError, match="must be 127.0.0.1"):
        config.validate()
    config = routed_config()
    config.mavproxy.preparation_output_port = config.mavproxy.poc_output_port
    with pytest.raises(ValueError, match="must be unique"):
        config.validate()


def test_port_preflight_detects_occupied_udp_port(monkeypatch):
    class FakeSocket:
        def bind(self, address):
            if address[1] == 14560:
                raise OSError("already in use")
        def close(self):
            pass
    monkeypatch.setattr(socket, "socket", lambda *args: FakeSocket())
    result = check_ports("127.0.0.1", [14560, 14561])
    assert result[14560].startswith("BUSY:")
    assert result[14561] == "AVAILABLE"
