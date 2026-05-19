"""Search-pattern safety tests."""

from control.search_patterns import (
    ExpandingSquareSearch,
    InitialScanSearch,
    LissajousSearch,
    SectorScanSearch,
)


def test_initial_scan_pretilt_searches_down():
    scan = InitialScanSearch()
    scan.start()

    _, pitch = scan.get_command()

    assert pitch < 0


def test_initial_scan_return_is_only_upward_move():
    scan = InitialScanSearch()
    scan.start()
    scan._state = "pitch_up"

    _, pitch = scan.get_command()

    assert pitch > 0


def test_sector_pitch_scan_searches_down():
    search = SectorScanSearch()
    search.start()
    search.pitch_scanning = True
    search.pitch_direction = -1

    _, pitch = search.get_command()

    assert pitch < 0


def test_expanding_square_pitch_arm_searches_down():
    search = ExpandingSquareSearch()
    search.start()
    search._arm = 1

    _, pitch = search.get_command()

    assert pitch < 0


def test_lissajous_pitch_never_commands_up_during_ground_search():
    search = LissajousSearch()
    search.start()

    for offset_s in (0.0, 5.0, 12.0, 25.0, 48.0):
        search._t0 = search._t0 - offset_s
        _, pitch = search.get_command()
        assert pitch <= 0
