import json
import math
from pathlib import Path
import shutil
from unittest.mock import patch

import pytest

from test_g2_patrol_entry import patrol, runner_at
from g2_body_geometry import wall_distance
from g2_patrol_entry import BODY_RADIUS, TURN_MARGIN, EntryGeometry
from g2_navigation_goals import patrol_goals, resolve_destination
from g2_boundary_recovery import RecoveryPolicy
from g2_build_compact_patrol import build

ROOT=Path(__file__).resolve().parents[1]
SCENE=ROOT/'maps/demo_box_20261018_patrol'
SOURCE=ROOT/'tests/fixtures/patrol_entry_20261029'


def turning_cells(policy, legacy=False):
    result=[]
    for row in range(policy.height):
        for col in range(policy.width):
            x=policy.origin[0]+(col+.5)*policy.resolution
            y=policy.origin[1]+(policy.height-row-.5)*policy.resolution
            if not policy.allowed(x,y):continue
            if legacy:
                good=all(policy.allowed(x+.06*math.cos(i*math.pi/8),y+.06*math.sin(i*math.pi/8)) for i in range(16))
            else:good=policy.compact_turn_allowed([x,y])
            if good:result.append((x,y))
    return result


def test_active_mask_and_effective_turn_area_are_smaller_keepouts():
    old,new=patrol.Policy(str(SOURCE)),patrol.Policy(str(SCENE))
    assert new.compact
    assert sum(v>250 for v in new.cells)>sum(v>250 for v in old.cells)
    before,after=turning_cells(old,True),turning_cells(new)
    assert len(after)>1.2*len(before)
    assert new.layout['center_wall_margin_m']==.34
    assert new.layout['bin']['center_keepout_buffer_m']==old.layout['bin']['center_keepout_buffer_m']
    g=EntryGeometry(new)
    # Check actual body polygons at multiple headings throughout the expanded
    # region, independently of the centre's physical-circle test.
    for xy in after[::31]:
        for i in range(16):assert g.translation_problem(list(xy)+[i*math.pi/8],0.) is None


@pytest.mark.parametrize('yaw',[0,45,90,135,180,225,270,315])
def test_close_box_start_works_for_every_heading(yaw):
    r,plant,clock,sleep=runner_at([1.06,.4,math.radians(yaw)])
    r.policy=patrol.Policy(str(SCENE))
    with patch.object(patrol.time,'time',side_effect=lambda:clock[0]), patch.object(patrol.time,'sleep',side_effect=sleep):
        r.enter_turn_reserve()
        r.pulse(0.,1.,.08)
    assert plant['commands'][-1]['action']=='stop'
    assert resolve_destination(r.policy,plant['pose'][:2],'patrol-cycle')


@pytest.mark.parametrize('pose',[(1.04,.37,87),(1.03,.45,114),(1.06,.4,75)])
def test_all_recorded_starts_with_active_compact_scene(pose):
    x,y,yaw=pose
    r,plant,clock,sleep=runner_at([x,y,math.radians(yaw)])
    r.policy=patrol.Policy(str(SCENE))
    with patch.object(patrol.time,'time',side_effect=lambda:clock[0]), patch.object(patrol.time,'sleep',side_effect=sleep):
        r.enter_turn_reserve()
        assert r.rotation_allowed(plant['pose'])
        r.pulse(0.,1.,.08)
    assert r._entry_reverse_used<=.1


def test_actual_route_and_recovery_share_physical_clearance():
    p=patrol.Policy(str(SCENE));g=EntryGeometry(p);recovery=RecoveryPolicy(p)
    assert recovery.radius==BODY_RADIUS+TURN_MARGIN
    route=patrol_goals(p)
    for a,b in zip(route,route[1:]+route[:1]):
        assert g.segment_allowed(a[1],b[1])
        assert p.compact_turn_allowed(a[1])
        assert recovery.recovered(a[1])
        assert p.plan(a[1],b[1])
    assert p.plan([.5,.5],p.layout['bin']['navigation_preparation']['point_map'])
    assert not p.allowed(*p.layout['bin']['footprint_map'][0])
    assert not p.compact_turn_allowed(p.layout['room_polygon'][0])


@pytest.mark.parametrize('kind',['mask','raw','grid','box','budget'])
def test_compact_approval_cannot_be_bypassed_by_editing_files(tmp_path,kind):
    target=tmp_path/'scene';target.mkdir()
    for name in ('scene_layout.json','navigation_center_mask.yaml','navigation_center_mask.pgm','demo_box.pgm'):
        shutil.copyfile(SCENE/name,target/name)
    layout=json.loads((target/'scene_layout.json').read_text(encoding='utf-8'))
    if kind in ('mask','raw'):
        path=target/('navigation_center_mask.pgm' if kind=='mask' else 'demo_box.pgm')
        data=bytearray(path.read_bytes());data[-1]^=255;path.write_bytes(data)
    elif kind=='box':layout['navigation_mask_validation']['envelope_room_uv'][0]-=.1
    elif kind=='budget':layout['compact_patrol']['physical_margin_m']=0.
    else:
        path=target/'navigation_center_mask.yaml'
        path.write_text(path.read_text().replace('- -2.0','- -2.1'))
    (target/'scene_layout.json').write_text(json.dumps(layout),encoding='utf-8')
    with pytest.raises(ValueError,match='compact mask approval'):patrol.Policy(str(target))


def test_rebuilding_profile_is_deterministic(tmp_path):
    build(SOURCE,SCENE,tmp_path)
    for name in ('scene_layout.json','navigation_center_mask.pgm','navigation_center_mask.yaml','outer_patrol_goals.json'):
        assert (tmp_path/name).read_bytes()==(SCENE/name).read_bytes()


def test_placement_return_uses_the_same_compact_turn_rule():
    from integration.g2_placement_corridor import Grid,encode_mask,turn_ready
    from integration.g2_patrol_reentry import turn_ready as reentry_ready
    p=patrol.Policy(str(SCENE))
    grid=Grid(encode_mask(p))
    # Centre is safe for a physical turn although its old 6/7cm ring fails.
    pose=[1.06,.4,math.radians(75)]
    assert grid.compact and p.compact_turn_allowed(pose)
    assert turn_ready(grid,pose) and reentry_ready(p,pose)
    assert not turn_ready(grid,[1.25,.6,0.])


def test_old_experimental_workspace_circuit_is_not_reused_with_moved_box():
    from integration.g2_offline_workspace_plan import Workspace,Camera,build as old_proposal
    with pytest.raises(ValueError,match='patrol corridor is not safe'):
        old_proposal(Workspace(),Camera())
