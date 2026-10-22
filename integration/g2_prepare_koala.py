"""Prepare a pinned, coherent PC runtime without connecting to the robot."""
import argparse
import hashlib
import json
from pathlib import Path, PurePosixPath
import shutil
import subprocess
import tarfile
import tempfile

UPSTREAM = 'https://github.com/koala915/iGibson-Navigation-Grasping.git'
COMMIT = 'e8766f03d0de3e48419a488f330b62e3412afab3'
SOURCES = ('mission_loop.py', 'mission_runtime.py', 'robot_rpc.py',
           'sugarbox_approach_grasp.py', 'sugarbox_rl_approach_final2.py')
FLOW_TEST_EDITS = (
    ('self.link(samples).frame_pose(10.5), (pose, 0.0, 0.0)',
     'self.link(samples).frame_pose(10.5, moving=False), (pose, 0.0, 0.0)'),
    ('self.assertAlmostEqual(moved, 0.02, places=6)',
     'self.assertAlmostEqual(moved, 0.02 + mission.PATROL_MAX_VX * 0.05, places=6)'),
    ('self.assertAlmostEqual(turned, 0.3)',
     'self.assertAlmostEqual(turned, 0.3 + mission.PATROL_MAX_WZ * 0.05)'),
    ('mission.PATROL_MAX_VX * 0.25)', 'mission.PATROL_MAX_VX * 0.275)'),
    ('mission.PATROL_MAX_WZ * 0.25)', 'mission.PATROL_MAX_WZ * 0.275)'),
    ('self.link(poses).frame_pose(10.5)[2], 0.01)',
     'self.link(poses).frame_pose(10.5)[2], 0.06)'),
)


def patch_sources(root):
    mission = root / 'integration/mission_loop.py'
    source = mission.read_text(encoding='utf-8')
    old = '                     if moving else 0.0)\n        return pose, moved + PATROL_MAX_VX * uncovered'
    new = ('                     if moving else 0.0)\n'
           '        if moving:\n'
           '            uncovered += max((b[0] - a[0] for a, b in zip(span, span[1:])), default=0.) / 2.\n'
           '        return pose, moved + PATROL_MAX_VX * uncovered')
    if source.count(old) != 1:
        raise ValueError('pinned frame-motion patch context changed')
    mission.write_text(source.replace(old, new), encoding='utf-8')
    approach = root / 'integration/sugarbox_rl_approach_final2.py'
    source = approach.read_text(encoding='utf-8')
    old = '        cv2.destroyAllWindows()'
    new = ('        try:\n'
           '            cv2.destroyAllWindows()\n'
           '        except cv2.error:\n'
           '            pass  # Headless GUI cleanup must not replace the stage result.\n')
    if source.count(old) != 1:
        raise ValueError('pinned headless-cleanup patch context changed')
    approach.write_text(source.replace(old, new), encoding='utf-8')
    tests = root / 'tests/test_flow_contracts.py'
    source = tests.read_text(encoding='utf-8')
    for old, new in FLOW_TEST_EDITS:
        if source.count(old) != 1:
            raise ValueError('pinned frame-motion test context changed')
        source = source.replace(old, new)
    tests.write_text(source, encoding='utf-8')


def verify_manifest(root):
    path = root / 'g2_runtime_manifest.json'
    if not path.is_file():
        raise ValueError('prepare the pinned runtime with integration/g2_prepare_koala.py first')
    manifest = json.loads(path.read_text(encoding='utf-8'))
    if manifest.get('upstream_commit') != COMMIT or manifest.get('schema') != 1:
        raise ValueError('incompatible koala runtime manifest')
    if manifest.get('patches') != ['frame_sample_motion_bound', 'headless_cleanup']:
        raise ValueError('required runtime fixes are missing')
    for name in SOURCES:
        actual = hashlib.sha256((root / 'integration' / name).read_bytes()).hexdigest()
        if manifest.get('source_sha256', {}).get(name) != actual:
            raise ValueError('prepared runtime source changed: ' + name)
    return manifest


def prepare(output, git_cache=None):
    output = Path(output).resolve()
    output.mkdir(parents=True, exist_ok=False)
    with tempfile.TemporaryDirectory(prefix='g2-koala-') as temp:
        temp = Path(temp)
        if git_cache is None:
            cache = temp / 'cache.git'
            subprocess.run(['git', 'init', '--bare', str(cache)], check=True, capture_output=True)
            subprocess.run(['git', '-C', str(cache), 'fetch', '--depth=1', UPSTREAM, COMMIT],
                           check=True, capture_output=True, timeout=180)
        else:
            cache = Path(git_cache).resolve()
            subprocess.run(['git', '-C', str(cache), 'cat-file', '-e', COMMIT + '^{commit}'],
                           check=True, capture_output=True, timeout=15)
        archive = temp / 'runtime.tar'
        with archive.open('wb') as target:
            subprocess.run(['git', '-C', str(cache), 'archive', '--format=tar', COMMIT],
                           stdout=target, stderr=subprocess.PIPE, check=True, timeout=120)
        with tarfile.open(archive) as bundle:
            for member in bundle:
                relative = PurePosixPath(member.name)
                if relative.is_absolute() or '..' in relative.parts or member.issym() or member.islnk():
                    raise ValueError('unsafe runtime archive path')
                target = output.joinpath(*relative.parts)
                if member.isdir():
                    target.mkdir(parents=True, exist_ok=True)
                elif member.isfile():
                    target.parent.mkdir(parents=True, exist_ok=True)
                    with target.open('xb') as stream:
                        shutil.copyfileobj(bundle.extractfile(member), stream)
                else:
                    raise ValueError('unsupported runtime archive entry')
    patch_sources(output)
    manifest = {'schema': 1, 'upstream_repository': UPSTREAM, 'upstream_commit': COMMIT,
                'patches': ['frame_sample_motion_bound', 'headless_cleanup'],
                'hardware_validated': False,
                'source_sha256': {name: hashlib.sha256((output / 'integration' / name).read_bytes()).hexdigest()
                                  for name in SOURCES}}
    (output / 'g2_runtime_manifest.json').write_text(json.dumps(manifest, indent=2), encoding='utf-8')
    return verify_manifest(output)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--git-cache', type=Path, help='existing Git object cache; never modified')
    args = parser.parse_args()
    result = prepare(args.output, args.git_cache)
    print(json.dumps({'prepared': str(args.output.resolve()), 'upstream_commit': result['upstream_commit']}))


if __name__ == '__main__':
    main()
