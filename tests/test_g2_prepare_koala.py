"""Runtime pinning and headless/motion patches; no network or robot access."""
import hashlib
import json
from types import SimpleNamespace

import pytest

from integration import g2_prepare_koala as prepare


def source_tree(root):
    (root / 'integration').mkdir()
    (root / 'tests').mkdir()
    for name in prepare.SOURCES:
        (root / 'integration' / name).write_text('# fixture\n', encoding='utf-8')
    (root / 'integration/mission_loop.py').write_text('''def frame_pose(span, pose, moving, moved, turned):
        uncovered = (0.
                     if moving else 0.0)
        return pose, moved + PATROL_MAX_VX * uncovered, turned + PATROL_MAX_WZ * uncovered
''', encoding='utf-8')
    (root / 'integration/sugarbox_rl_approach_final2.py').write_text('''def finish():
    try:
        return "stage-result"
    finally:
        cv2.destroyAllWindows()
''', encoding='utf-8')
    (root / 'tests/test_flow_contracts.py').write_text(
        '\n'.join(old for old, new in prepare.FLOW_TEST_EDITS), encoding='utf-8')


def manifest(root):
    data = dict(schema=1, upstream_commit=prepare.COMMIT,
                patches=['frame_sample_motion_bound', 'headless_cleanup'],
                source_sha256={name: hashlib.sha256((root / 'integration' / name).read_bytes()).hexdigest()
                               for name in prepare.SOURCES})
    (root / 'g2_runtime_manifest.json').write_text(json.dumps(data), encoding='utf-8')


def test_equal_endpoints_still_bound_unsampled_motion(tmp_path):
    source_tree(tmp_path)
    prepare.patch_sources(tmp_path)
    env = dict(PATROL_MAX_VX=.1, PATROL_MAX_WZ=.5)
    exec((tmp_path / 'integration/mission_loop.py').read_text(), env)
    span, pose = [(0., [0., 0., 0.]), (.5, [0., 0., 0.])], [0., 0., 0.]
    _, moved, turned = env['frame_pose'](span, pose, True, 0., 0.)
    assert moved == pytest.approx(.025) and turned == pytest.approx(.125)
    assert env['frame_pose'](span, pose, False, 0., 0.) == (pose, 0., 0.)


def test_headless_cleanup_preserves_stage_result_but_not_other_errors(tmp_path):
    source_tree(tmp_path)
    prepare.patch_sources(tmp_path)
    class HeadlessError(Exception): pass
    def fail(): raise HeadlessError('no GUI')
    cv2 = SimpleNamespace(error=HeadlessError, destroyAllWindows=fail)
    env = dict(cv2=cv2)
    exec((tmp_path / 'integration/sugarbox_rl_approach_final2.py').read_text(), env)
    assert env['finish']() == 'stage-result'
    def unexpected(): raise RuntimeError('unrelated fault')
    cv2.destroyAllWindows = unexpected
    with pytest.raises(RuntimeError, match='unrelated'):
        env['finish']()


def test_manifest_rejects_tampered_source(tmp_path):
    source_tree(tmp_path)
    prepare.patch_sources(tmp_path)
    manifest(tmp_path)
    assert prepare.verify_manifest(tmp_path)['upstream_commit'] == prepare.COMMIT
    (tmp_path / 'integration/robot_rpc.py').write_text('# modified')
    with pytest.raises(ValueError, match='source changed: robot_rpc'):
        prepare.verify_manifest(tmp_path)


def test_changed_patch_context_is_rejected(tmp_path):
    source_tree(tmp_path)
    (tmp_path / 'integration/mission_loop.py').write_text('# newer unreviewed version')
    with pytest.raises(ValueError, match='patch context changed'):
        prepare.patch_sources(tmp_path)


def test_existing_runtime_is_never_overwritten(tmp_path):
    with pytest.raises(FileExistsError):
        prepare.prepare(tmp_path)
