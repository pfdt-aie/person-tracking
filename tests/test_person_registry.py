"""Tests for persistent person re-identification."""

import numpy as np

from tracking.person_registry import PersonRegistry


def _solid_crop(bgr):
    crop = np.zeros((32, 32, 3), dtype=np.uint8)
    crop[:, :] = bgr
    return crop


def test_known_bytetrack_id_keeps_matching_appearance():
    reg = PersonRegistry()

    pid1 = reg.update(7, _solid_crop((0, 0, 255)), 10.0, 20.0, 100.0)
    pid2 = reg.update(7, _solid_crop((0, 0, 255)), 11.0, 21.0, 101.0)

    assert pid2 == pid1


def test_recycled_bytetrack_id_gets_new_person_when_appearance_changes():
    reg = PersonRegistry()

    pid1 = reg.update(7, _solid_crop((0, 0, 255)), 10.0, 20.0, 100.0)
    pid2 = reg.update(7, _solid_crop((255, 0, 0)), 11.0, 21.0, 101.0)

    assert pid2 != pid1


def test_known_bytetrack_id_with_invalid_crop_does_not_validate_identity():
    reg = PersonRegistry()

    pid1 = reg.update(7, _solid_crop((0, 0, 255)), 10.0, 20.0, 100.0)
    pid2 = reg.update(7, np.zeros((0, 0, 3), dtype=np.uint8), 11.0, 21.0, 101.0)

    assert pid2 != pid1
