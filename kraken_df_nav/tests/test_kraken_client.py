import pytest

from kraken_df_nav.kraken.client import KrakenBearingSource


def source():
    return KrakenBearingSource("http://127.0.0.1:8081/DOA_value.html")


def test_parse_normalized_json_response():
    result = source().parse_response(
        '{"bearing_deg": 42.5, "confidence": 0.91, "rssi_dbm": -61.2, '
        '"frequency_hz": 915000000, "timestamp": 123.5}'
    )
    assert result.relative_bearing_deg == 42.5
    assert result.frequency_hz == 915_000_000


def test_parse_kraken_v2_legacy_doa_response():
    result = source().parse_response(
        "1787930000123, 358, 0.87, 31.5, 915000000, UCA, 0, station, 0, 0\n"
    )
    assert result.relative_bearing_deg == 358
    assert result.confidence == .87
    assert result.rssi_dbm == 31.5
    assert result.timestamp == pytest.approx(1787930000.123)


def test_multiple_decimators_are_refused_as_ambiguous():
    row = "1787930000123, 10, .9, 30, 915000000\n"
    with pytest.raises(ValueError, match="multiple decimators"):
        source().parse_response(row + row)
