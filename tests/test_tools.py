"""Тесты чистых функций офлайн-инструментов (tools/), без записей."""
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / 'tools'))

from bags import EMPTY_BAGS, safe_messages          # noqa: E402


def test_safe_messages_stops_on_read_error():
    def broken():
        yield 1
        yield 2
        raise OSError('zstd checksum mismatch')

    errors = []
    assert list(safe_messages(broken(), errors)) == [1, 2]
    assert errors == ['OSError: zstd checksum mismatch']


def test_empty_bags_are_five_recordings_without_obstacle():
    assert len(EMPTY_BAGS) == 5 and 'doubleT_obstacle' not in EMPTY_BAGS and 'new_data' not in EMPTY_BAGS
