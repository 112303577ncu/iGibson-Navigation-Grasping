"""Exact source patches for the pinned e8766f0 runtime, never for legacy code."""

ENGINE_INIT = '''    progress = ProgressWatchdog()
    g2_recovery = None
    if G2_RECOVERY_CONFIG.get('G2_RECOVERY_STATE_FILE'):
        g2_recovery = G2EngineGate(
            G2_RECOVERY_CONFIG['G2_RECOVERY_STATE_FILE'],
            json.loads(G2_RECOVERY_CONFIG['G2_RECOVERY_CONTEXT']),
            G2_RECOVERY_CONFIG['G2_RECOVERY_CHECKPOINT_FILE'],
            json.loads(G2_RECOVERY_CONFIG['G2_RECOVERY_FOOTPRINT']),
            json.loads(G2_RECOVERY_CONFIG['G2_RECOVERY_HOLDING']),
            frame_bound_verified=json.loads(G2_RECOVERY_CONFIG.get('G2_RECOVERY_FRAME_BOUND_VERIFIED') or 'false'),
            emit=lambda value: print('[RECOVERY] ' + json.dumps(value), flush=True))
        g2_recovery.episode.used = float(G2_RECOVERY_CONFIG.get('G2_RECOVERY_WAIT_USED') or 0.)
'''

ENGINE_POLL = '''
                if g2_recovery is not None and not (arrived_latched or blocked_latched):
                    problem = ('SCAN_STALE' if scan is None or get_age(now, scan.stamp) > SCAN_STALE_TIMEOUT
                               else 'ODOM_STALE' if get_age(now, odom.stamp) > ODOM_STALE_TIMEOUT else None)
                    decision = g2_recovery.poll(vision_sequence, vision_frame_time,
                        vision_result if vision_reason == 'VALID' else None, problem,
                        observation_valid=vision_reason in ('VALID', 'NO_YOLO'))
                    if decision['reset']:
                        target.clear()
                        progress.reset()
                        target_search_start = 0.0
                        prev_raw_action = np.zeros(2, dtype=np.float32)
                        action_delay = [np.zeros(2, dtype=np.float32) for _ in range(MOTOR_DELAY_STEPS)]
                        contract_action.reset()
                        anti_jitter.reset_all()
                        center_controller.reset()
                        final_approach.reset()
                        pursuit.reset()
                        blind_finish.rearm()
                        seed_facing = SeedFacing()
                        seed_look = SeedLook()
                        far_sighting = FarSighting()
                    if decision.get('target_lost'):
                        motor.stop(repeat=1)
                        blocked_latched = True
                        blocked_reason = 'RECOVERY_TARGET_NOT_RECONFIRMED'
                        break
                    if not decision['allow']:
                        motor.stop(repeat=1)
                        progress.reset()
                        target.clear()
                        last_safety_reason = 'WAIT_LOCALIZATION_OR_OBSERVATION'
                        if not headless:
                            recovery_key = cv2.waitKey(1) & 0xff
                            if recovery_key == 27:
                                break
                            if recovery_key == 32:
                                e_stop_latched = True
                        time.sleep(.02)
                        continue
'''

PATCHES = {
    'sugarbox_rl_approach_final2.py': [
        ('import os\n', "import os\nfrom integration.g2_task_recovery import EngineGate as G2EngineGate\n"
         "G2_RECOVERY_CONFIG = {k: v for k, v in os.environ.items() if k.startswith('G2_RECOVERY_')}\n"),
        ('    progress = ProgressWatchdog()\n', ENGINE_INIT),
        ('                ) = ros_bridge.snapshot()\n\n\n                # ------------------------------------------------\n                # YOLO/SAM',
         '                ) = ros_bridge.snapshot()\n' + ENGINE_POLL + '\n                # ------------------------------------------------\n                # YOLO/SAM'),
        ('        if blocked_latched:\n            print("[STOPPED] " + blocked_reason)\n            return 4',
         '        if blocked_latched:\n            print("[STOPPED] " + blocked_reason)\n'
         '            if blocked_reason == "RECOVERY_TARGET_NOT_RECONFIRMED":\n                return 7\n            return 4'),
    ],
    'mission_runtime.py': [
        ('import time\n', 'import time\nfrom integration.g2_task_recovery import effective_pause\n'),
        ('        self.stage_log.write_text(\'\', encoding=\'utf-8\')',
         "        self.recovery_file = env.get('G2_RECOVERY_CHECKPOINT_FILE')\n        self.stage_log.write_text('', encoding='utf-8')"),
        ('        deadline = None if timeout is None else time.monotonic() + timeout\n        while self.poll() is None:\n            if deadline is not None and time.monotonic() >= deadline:',
         "        deadline = None if timeout is None else time.monotonic() + timeout\n"
         "        while self.poll() is None:\n"
         "            paused = effective_pause(getattr(self.runtime, 'recovery_file', None))\n"
         "            if deadline is not None and time.monotonic() >= deadline + paused:"),
    ],
    'mission_loop.py': [
        ('    def __init__(self, host, popen=subprocess.Popen, clock=time.monotonic):',
         '    def __init__(self, host, popen=subprocess.Popen, clock=time.monotonic, report_path=None):'),
        ('        self.clock, self.allowed = clock, False\n',
         "        command += ' ' + shlex.quote(str(report_path or PATROL_OUTPUT)) + ' ' + ('1' if report_path else '0')\n"
         "        self.recovery_record, self.recovery_at = None, None\n        self.clock, self.allowed = clock, False\n"),
        ('                record = json.loads(line)\n                patrol = record.get(\'patrol\')',
         "                record = json.loads(line)\n                with self.lock:\n"
         "                    self.recovery_record, self.recovery_at = record.get('recovery'), self.clock()\n"
         "                patrol = record.get('patrol')"),
        ('    args = parser.parse_args(argv)\n',
         "    args = parser.parse_args(argv)\n    globals()['_g2_host'] = args.host\n    globals()['_g2_homography'] = args.homography\n"),
        ('    def pose_now(self):\n',
         "    def recovery_now(self):\n        with self.lock:\n"
         "            if self.recovery_at is None or self.clock()-self.recovery_at > .5:\n                return None\n"
         "            return self.recovery_record\n\n    def pause_recovery(self):\n"
         "        self.proc.stdin.write(json.dumps({'allow': False, 'age': 0., 'pause': True}) + '\\n')\n"
         "        self.proc.stdin.flush()\n        self.allowed = False\n\n    def pose_now(self):\n"),
        ("                                              'age': round(max(0.0, age), 3)}) + '\\n')",
         "                                              'age': round(max(0.0, age), 3),\n"
         "                                              'frame': getattr(self, '_g2_last_frame', None),\n"
         "                                              'recovery_ack': getattr(self, '_g2_recovery_ack', 0)}) + '\\n')"),
        ("    deadline = now() + POSE_LOST_S\n    while link.pose_now() is None:",
         "    coordinator = globals().get('_g2_task_coordinator')\n"
         "    deadline = now() + POSE_LOST_S\n    while link.pose_now() is None:"),
        ('        time.sleep(0.1)\n    try:\n        while True:\n            if not link.alive():',
         "        time.sleep(0.1)\n    if coordinator is not None:\n        coordinator.begin_patrol(host, scene, reader, watcher, link)\n"
         "    try:\n        while True:\n            if not link.alive():"),
        ('            sighting = watcher.look(frame)\n            point, far = sighting if sighting is not None else (None, False)\n            pose = link.pose_now()',
         "            sighting = watcher.look(frame)\n"
         "            if coordinator is not None:\n"
         "                if not coordinator.patrol_frame(host, frame, last_stamp, sighting, link):\n"
         "                    clear = 0\n                    continue\n"
         "                if coordinator.patrol_reset(host):\n"
         "                    clips, unseen, far_cleared = [], [], []\n"
         "                    unplaced_since, last_deferred = None, None\n"
         "            point, far = sighting if sighting is not None else (None, False)\n            pose = link.pose_now()"),
        ('        if pid is not None and patrol_still_running(host, pid, link):\n            stop_patrol(host, pid, link=link)\n',
         "        try:\n            if pid is not None and patrol_still_running(host, pid, link):\n                stop_patrol(host, pid, link=link)\n"
         "        finally:\n            if coordinator is not None:\n                coordinator.end_patrol(host)\n"),
        ('    while now() < deadline:\n        frame, last_stamp = fresh_frame(reader, last_stamp, now)',
         "    coordinator = globals().get('_g2_task_coordinator')\n    recovery_host = getattr(link, '_g2_host', None)\n"
         "    while now() < deadline + (coordinator.paused_duration(recovery_host) if coordinator else 0.):\n"
         "        frame, last_stamp = fresh_frame(reader, last_stamp, now)"),
        ('        sighting = watcher.look(frame)\n        point, far = sighting if sighting is not None else (None, False)\n        if confirm.update',
         "        sighting = watcher.look(frame)\n"
         "        if coordinator is not None and recovery_host is not None:\n"
         "            if not coordinator.patrol_frame(recovery_host, frame, last_stamp, sighting, link):\n"
         "                confirm = Confirm()\n                continue\n"
         "        point, far = sighting if sighting is not None else (None, False)\n        if confirm.update"),
        ("    if state == 'unknown':\n        raise MissionError('grasp runner exit {}; the arm state is unknown'.format(code))",
         "    if state == 'unknown':\n        raise MissionError('grasp runner exit {}; the arm state is unknown'.format(code))\n"
         "    if code == 7:\n        to_travel(args.host)\n        return 'reobserve'"),
        ("            if outcome == 'missed_at_box':",
         "            if outcome == 'reobserve':\n"
         "                stage = 'REOBSERVE {}'.format(n)\n                room.leave(0.)\n                continue\n"
         "            if outcome == 'missed_at_box':"),
    ],
    'sugarbox_approach_grasp.py': [
        ('        if returncode == 4:\n', '        if returncode == 7:\n            return 7  # fresh views no longer confirm the old target; no arm action\n        if returncode == 4:\n'),
    ],
    'placement_dock.py': [
        ('def read_localization(host):', 'def read_localization(host, timeout=30):'),
        ("        capture_output=True, text=True, timeout=30)\n    lines = result.stdout.strip().splitlines()",
         "        capture_output=True, text=True, timeout=timeout)\n    lines = result.stdout.strip().splitlines()"),
    ],
    'patrol_vision_permit_jetson.py': [
        ('def write_permit(path, owner, boot, allow, stamp):',
         'def write_permit(path, owner, boot, allow, stamp, pause=False, frame=None, recovery_ack=0):'),
        ("                   'allow': allow is True}, f)",
         "                   'allow': allow is True, 'pause': pause is True and allow is not True, 'frame': frame, 'recovery_ack': recovery_ack}, f)"),
        ('            write_permit(permit_path, owner, boot, allow, stamp)\n',
         "            try:\n                pause = json.loads(line).get('pause') is True\n"
         "            except (ValueError, TypeError, AttributeError):\n                pause = False\n"
         "            message = json.loads(line) if pause or allow else {}\n"
         "            write_permit(permit_path, owner, boot, allow, stamp, pause, message.get('frame'), message.get('recovery_ack', 0))\n"),
        ('    permit_path, health_path, socket_path = argv[1:4]\n',
         "    permit_path, health_path, socket_path = argv[1:4]\n"
         "    report_path = argv[4] if len(argv) > 4 else None\n"
         "    independent_report = len(argv) > 5 and argv[5] == '1'\n"),
        ("            sys.stdout.write(json.dumps(line) + '\\n')\n",
         "            if report_path and (pid is not None or independent_report):\n"
         "                try:\n                    with open(report_path) as report_file:\n                        report = json.load(report_file)\n"
         "                    line['recovery'] = report.get('task_checkpoint')\n"
         "                except (IOError, ValueError):\n                    pass\n"
         "            sys.stdout.write(json.dumps(line) + '\\n')\n"),
    ],
}

TEST_PATCHES = {
    'test_mission_loop.py': [
        ("                self.assertEqual(self.fetch(code, 'empty'), 'missed')",
         "                self.assertEqual(self.fetch(code, 'empty'), 'reobserve' if code == 7 else 'missed')"),
        ("                         [{'allow': True, 'age': 0.4}, {'allow': False, 'age': 0.0}])",
         "                         [{'allow': True, 'age': 0.4, 'frame': None, 'recovery_ack': 0},\n"
         "                          {'allow': False, 'age': 0.0, 'frame': None, 'recovery_ack': 0}])"),
    ],
    'test_sugarbox_blind_finish.py': [
        ('        self.assertEqual(returns, [4])', '        self.assertEqual(returns, [4, 7])'),
    ],
}


def apply(root):
    for name, changes in PATCHES.items():
        path = root / 'integration' / name
        source = path.read_text(encoding='utf-8')
        for old, new in changes:
            if source.count(old) != 1:
                raise ValueError('pinned recovery patch context changed: ' + name + ': ' + old[:65])
            source = source.replace(old, new)
        path.write_text(source, encoding='utf-8')
    for name, changes in TEST_PATCHES.items():
        path = root / 'tests' / name
        source = path.read_text(encoding='utf-8')
        for old, new in changes:
            if source.count(old) != 1:
                raise ValueError('pinned recovery test context changed: ' + name)
            source = source.replace(old, new)
        path.write_text(source, encoding='utf-8')
