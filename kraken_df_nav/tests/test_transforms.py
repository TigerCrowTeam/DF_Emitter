import pytest

from kraken_df_nav.df.transforms import angular_difference_deg, relative_to_absolute, wrap_heading


@pytest.mark.parametrize("value, expected", [(0, 0), (360, 0), (-10, 350), (725, 5)])
def test_wrap_heading(value, expected):
    assert wrap_heading(value) == expected


def test_mount_and_heading_transform():
    assert relative_to_absolute(30, 350, 10) == 30
    assert angular_difference_deg(359, 1) == 2
