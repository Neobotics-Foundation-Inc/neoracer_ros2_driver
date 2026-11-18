"""Tests for scripts/udev/99-racecar.rules, read as text."""

from pathlib import Path
import re

import pytest

RULES = Path(__file__).resolve().parent.parent / 'scripts' / 'udev' / '99-racecar.rules'


def _rules(link):
    """Return the rule lines that link a device to /dev/`link`."""
    return [
        line
        for line in RULES.read_text().splitlines()
        if not line.lstrip().startswith('#') and f'SYMLINK+="{link}"' in line
    ]


def _ids(link):
    """Return the vendor:product pairs the rules link to /dev/`link`."""
    ids = set()
    for line in _rules(link):
        vid = re.search(r'ATTRS\{idVendor\}=="(\w{4})"', line)
        pid = re.search(r'ATTRS\{idProduct\}=="(\w{4})"', line)
        assert vid and pid, line
        ids.add(f'{vid.group(1)}:{pid.group(1)}')
    return ids


# lsusb on the dev car, 2026-11-17: "303a:4001 Seeed Studio & OSRBOT Neoracer
# V1" and "1209:4001 Generic OSRDISP LED Matrix". Without these lines, their
# links came only from another rules file on the car.
@pytest.mark.parametrize(
    'link, vid_pid', [('osrbot_base', '303a:4001'), ('osrbot_led_matrix', '1209:4001')]
)
def test_the_neoracer_v1_devices_get_their_links(link, vid_pid):
    assert vid_pid in _ids(link)


def test_modemmanager_leaves_every_base_board_alone():
    # A ModemManager probe can open the port mid-line and drop data.
    for line in _rules('osrbot_base'):
        assert 'ENV{ID_MM_DEVICE_IGNORE}="1"' in line, line
