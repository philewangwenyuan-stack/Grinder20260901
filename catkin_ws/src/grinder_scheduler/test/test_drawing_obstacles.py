"""Offline drawing contracts. No hardware, ROS node or map-server changes."""
import hashlib
import sys
import tempfile
from copy import deepcopy
from pathlib import Path

import numpy as np
import pytest
from google.protobuf.json_format import ParseDict

ROOT=Path(__file__).resolve().parents[4]
sys.path.insert(0,str(ROOT/'catkin_ws/src/grinder_scheduler/src'))
sys.path.insert(0,str(ROOT/'third_party/sl_linka/sdk/python'))
from sl_link.message_gen import sl_link_pb2 as pb
from grinder_scheduler.drawing_obstacles import DrawingStore, validate_payload, foundation_mask, path_avoids_foundations


def binding():
    return dict(width=100,height=100,resolution=.1,origin_x=0.,origin_y=0.,frame_id='map',map_revision='raw-one')


def payload():
    return dict(info=dict(layer_id='drawing-one',source_file='基础.dxf',source_sha256='a'*64,binding=binding(),
        transform=dict(scale=1.,measured_scale=1.,rms_m=.03,scale_locked=True,
        control_points=[dict(drawing_x_mm=x*1000,drawing_y_mm=y*1000,map_x_m=x,map_y_m=y) for x,y in [(0,0),(9,0),(0,9)]])),
        foundations=[dict(region_id='drawing:one',name='J-1',source='drawing',subtype='foundation',enabled=True,closed=True,
            inflation_m=.3,points=[dict(x=x,y=y) for x,y in [(4,4),(5,4),(5,5),(4,5)]])])


def upload(store, value=None, draft_id='draft-one', expected=0, split=300):
    message=pb.DrawingLayerPayload();ParseDict(value or payload(),message)
    data=message.SerializeToString();chunks=[data[i:i+split] for i in range(0,len(data),split)]
    result=None
    for i in reversed(range(len(chunks))):
        r=pb.DrawingDraftRequest(request_id='upload',map_id='map-one',operation=pb.DRAWING_UPLOAD,draft_id=draft_id,
            data=chunks[i],chunk_index=i,total_chunks=len(chunks),payload_sha256=hashlib.sha256(data).hexdigest(),expected_layer_version=expected)
        result=store.request(r,pb,binding())
    return result


def commit(store, request_id='save-one'):
    assert store.prepare_save('map-one','map-one','draft-one',1,binding(),request_id) is None
    response=pb.MapSaveResponse(result=pb.RESULT_SUCCESS,map_id='map-one')
    store.finish_save('map-one',binding(),response)
    return response


def test_vectors_survive_restart_without_creating_rasters():
    with tempfile.TemporaryDirectory() as root:
        store=DrawingStore(root);r=upload(store)
        assert r.complete and r.result==pb.RESULT_SUCCESS
        response=commit(store)
        assert response.drawing_layer_version==1
        restored=DrawingStore(root).layer('map-one',binding())
        region=restored['foundations'][0]
        assert region['source']=='drawing' and region['inflation_m']==pytest.approx(.3)
        assert restored['info']['source_file']=='基础.dxf'
        assert all(p.suffix=='.json' for p in Path(root).rglob('*') if p.is_file())


def test_cancel_draft_preserves_published_layer():
    store=DrawingStore();upload(store);commit(store)
    original=store.layer('map-one')
    value=payload();value['foundations'][0]['inflation_m']=1.0
    assert upload(store,value,draft_id='edit',expected=1).complete
    store.request(pb.DrawingDraftRequest(map_id='map-one',draft_id='edit',operation=pb.DRAWING_DISCARD),pb,binding())
    assert store.layer('map-one')==original


def test_stale_requires_version_bound_explicit_task_bypass():
    store=DrawingStore();upload(store);commit(store)
    changed=dict(binding(),map_revision='raw-two')
    assert store.layer('map-one',changed)['info']['status']==pb.DRAWING_STALE
    with pytest.raises(ValueError):store.effective('map-one','task-one',changed)
    store.authorize('map-one','task-one',True,1,changed)
    assert store.effective('map-one','task-one',changed)[0]==[]
    with pytest.raises(ValueError):store.effective('map-one','task-two',changed)
    with pytest.raises(ValueError):store.authorize('map-one','task-one',True,2,changed)


def test_display_angle_does_not_stale_world_coordinates():
    store=DrawingStore();upload(store);commit(store)
    assert store.layer('map-one',dict(binding(),app_rotation_deg=90,map_version=55))['info']['status']==pb.DRAWING_PUBLISHED


def test_pending_commit_blocks_start_and_commit_retries_are_idempotent():
    store=DrawingStore();upload(store)
    store.prepare_save('map-one','map-one','draft-one',1,binding(),'save-one')
    with pytest.raises(ValueError):store.authorize('map-one','task',False,0,binding())
    result=pb.MapSaveResponse(result=pb.RESULT_SUCCESS,map_id='map-one')
    store.finish_save('map-one',binding(),result)
    cached=store.prepare_save('map-one','map-one','draft-one',1,binding(),'save-one')
    assert pb.MapSaveResponse.FromString(cached).drawing_layer_version==1


def test_draft_is_rejected_on_wrong_map_or_changed_geometry():
    store=DrawingStore();r=upload(store);assert r.complete
    with pytest.raises(ValueError):store.draft('another-map','draft-one',1,binding())
    with pytest.raises(ValueError):store.draft('map-one','draft-one',1,dict(binding(),resolution=.2))


@pytest.mark.parametrize('mutate,code',[
    (lambda p:p['foundations'][0].update(points=[dict(x=1,y=1)]),'INVALID_POLYGON'),
    (lambda p:p['foundations'][0]['points'][0].update(x=float('nan')),'INVALID_POLYGON'),
    (lambda p:p['foundations'][0].update(points=[dict(x=x,y=y) for x,y in [(1,1),(3,3),(1,3),(3,1)]]),'SELF_INTERSECTION'),
    (lambda p:p['info']['transform'].update(rms_m=.21),'REGISTRATION'),
    (lambda p:p['info']['transform'].update(measured_scale=1.02),'REGISTRATION'),
    (lambda p:p['info']['transform'].update(control_points=[]),'CONTROL_POINTS'),
    (lambda p:p['foundations'][0].update(inflation_m=-.1),'INVALID_POLYGON'),
    (lambda p:p['foundations'][0].update(points=[dict(x=x,y=y) for x,y in [(20,20),(21,20),(21,21),(20,21)]]),'OUTSIDE_MAP')])
def test_invalid_drawing_is_blocked(mutate,code):
    value=payload();mutate(value)
    assert code in [c['code'] for c in validate_payload(value,binding())]


def test_empty_selection_does_not_delete_but_explicit_removal_can():
    value=payload();value['foundations']=[]
    assert any(c['code']=='EMPTY_LAYER' for c in validate_payload(value,binding()))
    value['remove_layer']=True
    assert validate_payload(value,binding())==[]


def test_inflated_mask_is_transient_and_ignore_is_skipped():
    value=payload();base=np.zeros((100,100),dtype=np.int16);copy=base.copy()
    mask=foundation_mask(base.shape,binding(),value['foundations'])
    assert np.array_equal(base,copy) and mask[45,45] and mask[38,45]
    value['foundations'][0]['drawing_action']=1
    assert not np.any(foundation_mask(base.shape,binding(),value['foundations']))


def test_collision_checks_entire_segments_and_rotated_footprint():
    mask=foundation_mask((100,100),binding(),payload()['foundations'])
    assert not path_avoids_foundations([dict(x=2,y=4.5),dict(x=7,y=4.5)],mask,binding())
    assert path_avoids_foundations([dict(x=2,y=2),dict(x=7,y=2)],mask,binding())
    assert not path_avoids_foundations([dict(x=3,y=3.5),dict(x=6,y=3.5)],mask,binding(),footprint=[(-.2,-.2),(1.2,-.2),(1.2,.6),(-.2,.6)])


def test_partial_outside_polygon_is_not_vertex_clamped_to_map_edge():
    value=payload();value['foundations'][0]['points']=[dict(x=x,y=y) for x,y in [(-5,4),(2,4),(2,5),(-5,5)]]
    assert any(c['code']=='PARTIAL_OUTSIDE' for c in validate_payload(value,binding()))
    mask=foundation_mask((100,100),binding(),value['foundations'])
    assert mask[45,0] and not mask[0,0]


def test_corrupt_persistent_layer_blocks_instead_of_disappearing(tmp_path):
    store=DrawingStore(str(tmp_path))
    Path(store.root).mkdir(parents=True)
    Path(store._path('map-one')).write_text('{broken',encoding='utf-8')
    assert store.layer('map-one')['info']['status']==pb.DRAWING_STALE
    with pytest.raises(ValueError):store.effective('map-one','task',binding())


def test_stop_epoch_revokes_even_an_in_flight_authorization():
    store=DrawingStore();upload(store);commit(store)
    changed=dict(binding(),map_revision='replacement')
    store.authorize('map-one','task',True,1,changed)
    old=store.tasks[('map-one','task')]
    store.forget_tasks()
    # Simulate a concurrent write completing after lock-free cancellation.
    store.tasks[('map-one','task')]=old
    with pytest.raises(ValueError):store.effective('map-one','task',changed)


def test_pending_commit_can_retry_after_process_restart(tmp_path):
    store=DrawingStore(str(tmp_path));upload(store)
    store.prepare_save('map-one','map-one','draft-one',1,binding(),'restart-save')
    recovered=DrawingStore(str(tmp_path))
    with pytest.raises(ValueError):recovered.effective('map-one','task',binding())
    assert recovered.prepare_save('map-one','map-one','draft-one',1,binding(),'restart-save') is None
    recovered.finish_save('map-one',binding(),pb.MapSaveResponse(result=pb.RESULT_SUCCESS,map_id='map-one'))
    assert len(recovered.effective('map-one','task',binding())[0])==1


def test_exact_registration_default_zero_rms_is_valid_on_wire():
    value=payload();value['info']['transform']['rms_m']=0
    store=DrawingStore()
    assert upload(store,value).complete


def test_board_recomputes_control_residuals_instead_of_trusting_metadata():
    value=payload();value['info']['transform']['tx_m']=1
    assert any(c['code']=='REGISTRATION' and c['level']=='error' for c in validate_payload(value,binding()))


def test_selecting_another_task_revokes_previous_task_bypass():
    store=DrawingStore();upload(store);commit(store)
    changed=dict(binding(),map_revision='replacement')
    store.authorize('map-one','task-a',True,1,changed)
    store.authorize('map-one','task-b',True,1,changed)
    with pytest.raises(ValueError):store.effective('map-one','task-a',changed)
    assert store.effective('map-one','task-b',changed)[0]==[]
