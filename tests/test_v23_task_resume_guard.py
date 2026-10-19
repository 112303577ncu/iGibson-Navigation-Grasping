import copy
import math
from pathlib import Path
import sys

import pytest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from integration import v23_task_resume_guard as guard


def snapshot(now=100):
    h=dict(ok=True,service_pid=7,holding=False,
           chassis=dict(motors=[0,0,0,0],moving=False,stopped_for_s=1,arm_busy=False,
                        arm_pose='travel',client=None,board_rx_age_s=0),
           odom=dict(valid=True,pose=[0,0,0]))
    l=dict(ready=True,stamp=now,boot_id='boot',owner_pid=7,map_hash='map',
           state=dict(pose=[1,2,0],odom=[0,0,0],odom_stamp=now,scan_age=.01,tf_age=0,cov=[.001,.001,.001]))
    return h,l,'boot',1500


def checkpoint(phase='idle'):
    h,l,b,_=snapshot()
    return guard.begin_checkpoint(phase,h,l,b,100,'operation-1' if phase in ('grasp','release') else None)


def reference_config():
    return dict(source='aruco',calibration_verified=True,calibration_id='camera-1',
                registration_id='anchors-1',map_hash='map',marker_ids=[0,2])


def packet(now=100,seq=1):
    return dict(valid=True,source='aruco',frame='map',stamp=now,seq=seq,
                boot_id='boot',owner_pid=7,map_hash='map',calibration_id='camera-1',
                registration_id='anchors-1',marker_ids=[2],pose_map=[1.4,2,0],covariance_diag=[.001,.001,.001])


def test_idle_can_last_hours_without_artificial_expiry():
    g=guard.ResumeGate(checkpoint())
    for i in range(5):
        now=3700+i*.05;h,l,b,_=snapshot(now);result=g.poll(h,l,b,now)
    assert result['ok'] and result['status']=='READY_TO_REPLAN'
    assert not result['arm_or_motion_executed'] and result['valid_until']==pytest.approx(now+.4)


@pytest.mark.parametrize('phase,holding',[('grasp',True),('release',False)])
def test_completed_arm_task_requires_exact_result_id_and_holding_state(phase,holding):
    g=guard.ResumeGate(checkpoint(phase))
    h,l,b,_=snapshot(240);h['holding']=holding
    assert g.poll(h,l,b,240)['status']=='WAIT_ACTION_RESULT'
    assert g.poll(h,l,b,240,'success','other-operation')['status']=='WAIT_ACTION_RESULT'
    assert g.poll(h,l,b,240,'failure','operation-1')['status']=='WAIT_ACTION_RESULT'
    for i in range(5):
        h,l,b,_=snapshot(240+i*.05);h['holding']=holding
        result=g.poll(h,l,b,240+i*.05,'success','operation-1')
    assert result['ok']


def test_arm_busy_is_task_wait_not_localization_failure_or_retry():
    cp=checkpoint('grasp');h,l,b,_=snapshot(500);h['holding']=True;h['chassis']['arm_busy']=True
    state=guard.observe_wait(cp,h,l,b,500,1500)
    assert state['localization_ready'] and state['task_wait_elapsed_s']==400
    assert not state['task_ready'] and not state['task_completed']
    assert guard.ResumeGate(cp).poll(h,l,b,500,'success','operation-1')['status']=='WAIT_TASK_READY'


def test_five_distinct_frames_and_stability_are_required():
    g=guard.ResumeGate(checkpoint());h,l,b,_=snapshot()
    for _ in range(10):result=g.poll(h,l,b,100)
    assert not result['ok'] and result['stable_frames']==1
    # A slow cumulative correction must not look stable just because each step is small.
    for i in range(1,6):
        h,l,b,_=snapshot(100+i*.05);l['state']['pose'][0]+=i*.015
        result=g.poll(h,l,b,100+i*.05)
    assert not result['ok']


@pytest.mark.parametrize('change',["health_stamp","odom_stamp","scan_age","tf_age","covariance","odom_mismatch"])
def test_stale_or_uncertain_inputs_never_resume(change):
    g=guard.ResumeGate(checkpoint());h,l,b,_=snapshot()
    if change=='health_stamp':l['stamp']=99
    elif change=='odom_stamp':l['state']['odom_stamp']=99
    elif change=='scan_age':l['state']['scan_age']=1
    elif change=='tf_age':l['state']['tf_age']=2
    elif change=='covariance':l['state']['cov'][0]=.1
    else:h['odom']['pose']=[.05,0,0]
    result=g.poll(h,l,b,100)
    assert not result['ok'] and result['status']=='WAIT_LOCALIZATION'


@pytest.mark.parametrize('change',['boot','owner','map'])
def test_task_binding_change_needs_new_context(change):
    g=guard.ResumeGate(checkpoint());h,l,b,_=snapshot()
    if change=='boot':b='new-boot'
    elif change=='owner':h['service_pid']=8
    else:l['map_hash']='new-map'
    assert g.poll(h,l,b,100)['status']=='NEW_TASK_CONTEXT_REQUIRED'


def test_relative_approach_motion_is_propagated_but_stationary_task_motion_is_not():
    h,l,b,_=snapshot();h['odom']['pose'][0]=.5;l['state']['odom'][0]=.5;l['state']['pose'][0]=1.5
    assert guard.ResumeGate(checkpoint()).poll(h,l,b,100)['status']=='NEED_REFERENCE'
    g=guard.ResumeGate(checkpoint('visual-approach'))
    for i in range(5):
        now=100+i*.05;l['stamp']=now;l['state']['odom_stamp']=now
        result=g.poll(h,l,b,now)
    assert result['ok']


def test_observed_jump_and_reported_relocation_require_reference():
    h,l,b,_=snapshot();l['state']['pose'][0]=1.4
    assert guard.ResumeGate(checkpoint()).poll(h,l,b,100)['status']=='NEED_REFERENCE'
    h,l,b,_=snapshot()
    assert guard.ResumeGate(checkpoint()).poll(h,l,b,100,relocated=True)['status']=='NEED_REFERENCE'


def test_disabled_calibration_template_cannot_authorize_seed():
    cfg=reference_config();cfg['calibration_verified']=False
    with pytest.raises(ValueError):guard.ReferenceCollector(cfg,checkpoint())


@pytest.mark.parametrize('change',['id','stale','future','frame','cov','calibration','registration','owner','boot','map','id_only'])
def test_bad_aruco_observation_never_becomes_a_seed(change):
    c=guard.ReferenceCollector(reference_config(),checkpoint());h,_,b,_=snapshot();p=packet()
    if change=='id':p['marker_ids']=[7]
    elif change=='stale':p['stamp']=98
    elif change=='future':p['stamp']=101
    elif change=='frame':p['frame']='camera'
    elif change=='cov':p['covariance_diag'][0]=.1
    elif change=='calibration':p['calibration_id']='unmeasured'
    elif change=='registration':p['registration_id']='unregistered'
    elif change=='owner':p['owner_pid']=8
    elif change=='boot':p['boot_id']='other-boot'
    elif change=='map':p['map_hash']='other-map'
    else:del p['pose_map']
    for _ in range(5):assert c.feed(p,h,b,100) is None


def test_reference_needs_three_new_frames_and_preserves_source_time():
    c=guard.ReferenceCollector(reference_config(),checkpoint());h,_,b,_=snapshot()
    for _ in range(10):assert c.feed(packet(),h,b,100) is None
    assert c.count==1
    assert c.feed(packet(100.1,2),h,b,100.15) is None
    seed=c.feed(packet(100.2,3),h,b,100.25)
    assert seed['stamp']==100.2 and seed['pose']==[1.4,2,0]


def run_with_clock(cp,mutate=None,real=False,with_reference=False,ack=True,after_submit=None,**kwargs):
    elapsed=[0.];writes=[]
    def now():return 100+elapsed[0]
    def sleep(dt):elapsed[0]+=dt
    def snap():
        h,l,b,ram=snapshot(now())
        if mutate:mutate(h,l,elapsed[0])
        if writes:
            l['ready']=True;l['state']['pose']=list(writes[-1]['pose'])
            if ack:l['accepted_reference_stamp']=writes[-1]['stamp']
            if after_submit:after_submit(h,l)
        return h,l,b,ram
    c=guard.ReferenceCollector(reference_config(),cp) if with_reference else None
    result=guard.run_resume(cp,snap,wait_seconds=1,collector=c,
        reference_read=lambda:packet(now(),round(elapsed[0]/.05)+1),seed_write=writes.append,
        real=real,wall_clock=now,clock=lambda:elapsed[0],sleep=sleep,**kwargs)
    return result,writes,elapsed[0]


def test_transient_problem_recovers_without_repeating_task():
    def mutate(h,l,t):
        if t<.2:l['ready']=False;l['reason_code']='AMCL_STALE'
    result,writes,_=run_with_clock(checkpoint(),mutate)
    assert result['ok'] and not writes and not result['arm_or_motion_executed']


def test_no_healthy_frames_has_bounded_timeout():
    result,writes,elapsed=run_with_clock(checkpoint(),lambda h,l,t:l.update(ready=False))
    assert result['status']=='RESUME_TIMEOUT' and elapsed<=1.1 and not writes


def test_dry_run_can_propose_but_never_submit_reference():
    result,writes,_=run_with_clock(checkpoint(),with_reference=True,relocated=True)
    assert result['status']=='REFERENCE_READY_DRY_RUN' and not writes


def test_reference_bridge_waits_for_ack_and_new_stable_pose():
    result,writes,_=run_with_clock(checkpoint(),with_reference=True,real=True,relocated=True)
    assert result['ok'] and len(writes)==1 and result['stable_frames']==5
    assert result['pose']==[1.4,2,0] and not result['arm_or_motion_executed']


def test_no_supervisor_ack_cannot_resume_or_submit_seed_repeatedly():
    result,writes,_=run_with_clock(checkpoint(),with_reference=True,real=True,relocated=True,ack=False)
    assert result['status']=='RESUME_TIMEOUT' and len(writes)==1
    assert result['last_status']=='WAIT_REFERENCE_APPLIED'


def test_acknowledged_but_inconsistent_pose_does_not_resume():
    def wrong_pose(h,l):l['state']['pose'][0]+=0.2
    result,writes,_=run_with_clock(checkpoint(),with_reference=True,real=True,relocated=True,after_submit=wrong_pose)
    assert result['status']=='RESUME_TIMEOUT' and len(writes)==1


def test_owner_change_during_reference_application_cancels_old_task_context():
    def new_owner(h,l):h['service_pid']=8;l['owner_pid']=8
    result,writes,_=run_with_clock(checkpoint(),with_reference=True,real=True,relocated=True,after_submit=new_owner)
    assert result['status']=='NEW_TASK_CONTEXT_REQUIRED' and len(writes)==1


def test_unrelated_aruco_packet_gap_cannot_satisfy_three_frame_confirmation():
    c=guard.ReferenceCollector(reference_config(),checkpoint());h,_,b,_=snapshot()
    assert c.feed(packet(100,1),h,b,100) is None
    assert c.feed(packet(101.1,2),h,b,101.1) is None
    assert c.count==1


def test_unknown_arm_result_returns_immediately_without_seed_write():
    result,writes,elapsed=run_with_clock(checkpoint('grasp'),with_reference=True,real=True)
    assert result['status']=='WAIT_ACTION_RESULT' and not writes and elapsed==0


def test_wait_monitor_does_not_mistake_stale_health_for_ready():
    cp=checkpoint();h,l,b,ram=snapshot();l['stamp']=98
    result=guard.observe_wait(cp,h,l,b,100,ram)
    assert not result['localization_ready'] and not result['task_completed']
