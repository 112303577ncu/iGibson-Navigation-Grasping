"""Prepare a pinned, coherent PC runtime without connecting to the robot."""
import argparse
import hashlib
import json
from pathlib import Path, PurePosixPath
import shutil
import subprocess
import tarfile
import tempfile
import importlib.util

# The entry verifies the manifest BEFORE selecting the coherent integration
# package. Importing this builder must not load the local legacy package.
_patch_spec = importlib.util.spec_from_file_location('g2_recovery_build_patches',
    Path(__file__).with_name('g2_recovery_patches.py'))
g2_recovery_patches = importlib.util.module_from_spec(_patch_spec)
_patch_spec.loader.exec_module(g2_recovery_patches)

UPSTREAM = 'https://github.com/koala915/iGibson-Navigation-Grasping.git'
COMMIT = 'e8766f03d0de3e48419a488f330b62e3412afab3'
SOURCES = ('mission_loop.py', 'mission_runtime.py', 'robot_rpc.py',
           'sugarbox_approach_grasp.py', 'sugarbox_rl_approach_final2.py',
           'placement_dock.py', 'g2_placement_corridor.py', 'g2_task_recovery.py',
           'g2_task_coordinator.py', 'patrol_vision_permit_jetson.py')
SCHEMA = 3
PATCHES = ['frame_sample_motion_bound', 'headless_cleanup', 'reversible_placement_gate',
           'bounded_task_recovery_v1']
ENGINE_SEND = '        self._send_json(\n            msg\n        )'
ENGINE_GATE = '''        if G2_GUARD_PATH and TARGET_KIND == 'placement' and (vx or wz):
            if self.sock is None:
                self.connect()
            if not hasattr(self, '_g2_guard'):
                self._g2_guard = G2Permit(G2_GUARD_PATH)
            safe_vx, safe_wz = self._g2_guard.limit(vx, wz)
            if not safe_vx and not safe_wz:
                msg = {'action': 'stop'}
            else:
                msg['vx'], msg['wz'] = safe_vx, safe_wz
        self._send_json(
            msg
        )'''
CONNECT_ASSIGN = '        self.sock = sock'
CONNECT_GATE = '''        self.sock = sock
        if G2_GUARD_PATH and TARGET_KIND == 'placement':
            if getattr(self, '_g2_connected_once', False):
                self.sock.sendall(json.dumps({'action': 'stop'}).encode('utf-8') + bytes([10]))
                raise RuntimeError('placement motor connection changed; velocity was not replayed')
            self._g2_connected_once = True'''
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


def recipe_sha256():
    digest = hashlib.sha256()
    for path in (Path(__file__), Path(g2_recovery_patches.__file__)):
        digest.update(path.read_bytes())
    return digest.hexdigest()


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
    source = source.replace(old, new)
    if (source.count('import json\n') != 1 or source.count(ENGINE_SEND) != 1
            or source.count(CONNECT_ASSIGN) != 1):
        raise ValueError('pinned placement motor gate context changed')
    source = source.replace('import json\n', 'import json\nfrom integration.g2_placement_corridor import Permit as G2Permit\n')
    source = source.replace(ENGINE_SEND, ENGINE_GATE)
    source = source.replace(CONNECT_ASSIGN, CONNECT_GATE)
    # Capture the run's environment during module loading; resident workers
    # restore os.environ immediately after importing each stage.
    marker = 'import os\n'
    source = source.replace(marker, marker + "G2_GUARD_PATH = os.environ.get('G2_PLACEMENT_GUARD_FILE', '')\n")
    approach.write_text(source, encoding='utf-8')
    shutil.copyfile(Path(__file__).with_name('g2_placement_corridor.py'),
                    root / 'integration/g2_placement_corridor.py')
    shutil.copyfile(Path(__file__).with_name('g2_task_recovery.py'),
                    root / 'integration/g2_task_recovery.py')
    shutil.copyfile(Path(__file__).with_name('g2_task_coordinator.py'),
                    root / 'integration/g2_task_coordinator.py')
    g2_recovery_patches.apply(root)
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
    if manifest.get('upstream_commit') != COMMIT or manifest.get('schema') != SCHEMA:
        raise ValueError('incompatible koala runtime manifest')
    if manifest.get('patches') != PATCHES:
        raise ValueError('required runtime fixes are missing')
    if manifest.get('patch_recipe_sha256') != recipe_sha256():
        raise ValueError('runtime patch recipe changed; prepare a new runtime')
    for name in SOURCES:
        actual = hashlib.sha256((root / 'integration' / name).read_bytes()).hexdigest()
        if manifest.get('source_sha256', {}).get(name) != actual:
            raise ValueError('prepared runtime source changed: ' + name)
    helper = 'g2_placement_corridor.py'
    if manifest['source_sha256'][helper] != hashlib.sha256(Path(__file__).with_name(helper).read_bytes()).hexdigest():
        raise ValueError('placement guard differs from PC planner; prepare a new runtime')
    helper = 'g2_task_recovery.py'
    if manifest['source_sha256'][helper] != hashlib.sha256(Path(__file__).with_name(helper).read_bytes()).hexdigest():
        raise ValueError('recovery helper differs from PC coordinator; prepare a new runtime')
    helper = 'g2_task_coordinator.py'
    if manifest['source_sha256'][helper] != hashlib.sha256(Path(__file__).with_name(helper).read_bytes()).hexdigest():
        raise ValueError('recovery coordinator differs from PC entry; prepare a new runtime')
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
    manifest = {'schema': SCHEMA, 'upstream_repository': UPSTREAM, 'upstream_commit': COMMIT,
                'patches': PATCHES,
                'patch_recipe_sha256': recipe_sha256(),
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
