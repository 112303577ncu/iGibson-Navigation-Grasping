import importlib.util
import json
import math
from pathlib import Path

import pytest

spec = importlib.util.spec_from_file_location('scan_filter', Path(__file__).resolve().parents[1] / 'deploy/ros/g2_self_return_filter.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def test_matched_ray_is_unknown_and_other_ranges_retained():
    result, count = module.filter_ranges([.3, .2, .3], 0, math.radians(1), {0: [.3]}, .025)
    assert math.isnan(result[0]) and result[1:] == [.2, .3] and count == 1


def test_metadata_geometry_not_cropped_and_invalid_rays_preserved():
    values = [float('inf'), float('nan'), .5]
    result, count = module.filter_ranges(values, -math.pi, .01, {}, .025)
    assert len(result) == len(values) and math.isinf(result[0]) and math.isnan(result[1])
    assert count == 0


def test_rounding_matches_profile_builder_at_half_degree():
    result, count = module.filter_ranges([.3], math.radians(.5), .01, {0: [.3]}, .025)
    assert count == 1


def test_profile_json_keys_loaded_as_integer(tmp_path):
    path = tmp_path / 'profile.json'
    path.write_text(json.dumps({'schema': 1, 'frame_id': 'laser', 'tolerance_m': .025,
                               'ranges_by_angle_deg': {'90': [.3]}}))
    profile, tolerance = module.load_profile(path)
    assert profile == {90: [.3]} and tolerance == .025


def test_invalid_profile_rejected(tmp_path):
    path = tmp_path / 'profile.json'
    path.write_text(json.dumps({'schema': 1, 'frame_id': 'laser', 'tolerance_m': .1,
                               'ranges_by_angle_deg': {'90': [.3]}}))
    with pytest.raises(ValueError):
        module.load_profile(path)


def test_fixed_profile_does_not_learn_new_obstacles():
    profile = {90: [.3]}
    result, count = module.filter_ranges([.2], math.pi/2, .01, profile, .025)
    assert result == [.2] and count == 0 and profile == {90: [.3]}


def test_body_profile_masks_observed_structure_but_preserves_same_ray_wall():
    path = Path(__file__).resolve().parents[1] / 'deploy/ros/lidar_self_returns_2026-10-18.json'
    profile, tolerance = module.load_profile(path)
    # Raw laser 81deg maps into the physically confirmed front-right cable patch.
    a = math.radians(81)
    result, count = module.filter_ranges([.23], a, .01, profile, tolerance)
    assert count == 1 and math.isnan(result[0])
    result, count = module.filter_ranges([.6], a, .01, profile, tolerance)
    assert count == 0 and result == [.6]
    # A new close return outside known body geometry remains visible.
    result, count = module.filter_ranges([.25], math.pi, .01, profile, tolerance)
    assert count == 0 and result == [.25]


def test_body_profile_retains_every_return_outside_32cm_base_radius():
    path = Path(__file__).resolve().parents[1] / 'deploy/ros/lidar_self_returns_2026-10-18.json'
    profile, tolerance = module.load_profile(path)
    for degrees in range(360):
        a = math.radians(degrees)
        # Construct a ray endpoint at base-centre radius 0.33, 0.40 and 0.60m.
        for radius in (.33,.4,.6):
            theta = a + math.pi
            r = -.1*math.cos(theta) + math.sqrt(radius**2 - .01*math.sin(theta)**2)
            result, count = module.filter_ranges([r], a, .01, profile, tolerance)
            assert count == 0 and result == [r]


@pytest.mark.parametrize('change', ['geometry', 'wide', 'reversed', 'no_source'])
def test_invalid_body_masks_are_rejected(tmp_path, change):
    source = Path(__file__).resolve().parents[1] / 'deploy/ros/lidar_self_returns_2026-10-18.json'
    data = json.loads(source.read_text())
    if change == 'geometry':data['laser_geometry']['forward_m'] = .13
    elif change == 'wide':data['body_rectangles_base_m'][0]['bounds_xyxy'] = [-.5,-.1,.1,.1]
    elif change == 'reversed':data['body_rectangles_base_m'][0]['bounds_xyxy'] = [.1,-.1,-.1,.1]
    else:data['body_rectangles_base_m'][0]['source'] = ''
    path = tmp_path / 'profile.json';path.write_text(json.dumps(data))
    with pytest.raises(ValueError):module.load_profile(path)
