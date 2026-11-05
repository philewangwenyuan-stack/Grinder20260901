import sys
import tempfile
from pathlib import Path
from types import SimpleNamespace

import numpy as np

ROOT=Path(__file__).resolve().parents[4]
sys.path.insert(0,str(ROOT/'catkin_ws/src/grinder_scheduler/src'))
sys.path.insert(0,str(ROOT/'catkin_ws/src/grinder_chassis_driver/src'))
sys.path.insert(0,str(ROOT/'third_party/sl_linka/sdk/python'))
from grinder_scheduler.map_service import MapService
from grinder_scheduler.models import TaskConfigModel
from grinder_scheduler.drawing_scheduler import DrawingObstacleMixin
from grinder_scheduler.drawing_scheduler import region_response_frames
from test_drawing_obstacles import pb,binding,payload,upload,commit


def service():
    value=MapService()
    stamp=SimpleNamespace(secs=0,nsecs=0)
    info=SimpleNamespace(width=100,height=100,resolution=.1,origin=SimpleNamespace(position=SimpleNamespace(x=0.,y=0.)))
    value.set_raw_map(SimpleNamespace(info=info,header=SimpleNamespace(frame_id='map',seq=0,stamp=stamp),data=np.zeros(10000,dtype=np.int8)))
    return value


class Node(DrawingObstacleMixin):
    def __init__(self):
        self.sl_link_server=SimpleNamespace(pb=pb)
        self.map_service=service();self.task_config=TaskConfigModel()
        self.state=SimpleNamespace(value='READY');self._persist_state_enabled=False
        self.map_id='map-one';self.revision='raw-one';self.original_save_calls=0
        self._robot_footprint=[[-.4,.47],[1.2,.47],[1.2,-.47],[-.4,-.47]]
    def _current_map_id(self):return self.map_id
    def _find_recorded_map_by_id(self,map_id):return dict(map_revision=self.revision)
    def _is_live_map_id(self,map_id):return map_id in ('','LIVE_MAP')
    def _validate_requested_map_id(self,map_id,*args,**kwargs):return map_id==self.map_id,self.map_id
    def _localization_motion_status(self):return False,'',{}
    def _set_error(self,message):self.last_error=message
    def _handle_map_save_without_drawing(self,data):
        request=pb.MapSaveRequest.FromString(data)
        assert not request.drawing_draft_id
        self.original_save_calls+=1
        response=pb.MapSaveResponse(result=pb.RESULT_SUCCESS,map_id=request.map_id)
        return response.SerializeToString(),pb.MSG_ID_MAP_SAVE_RESPONSE,pb.COMP_SCHEDULER
    def _handle_map_edit_without_drawing(self,data):
        request=pb.MapEditCommand.FromString(data)
        self.map_service.apply_edit(dict(operation='UPSERT_OBSTACLE_REGION',region=dict(region_id=request.region.region_id,
            enabled=True,points=[dict(x=p.x,y=p.y) for p in request.region.points])))
        return pb.MapEditResponse().SerializeToString(),pb.MSG_ID_MAP_EDIT_RESPONSE,pb.COMP_SCHEDULER
    def _handle_path_plan_without_drawing(self,data,**kwargs):
        request=pb.PathPlanRequest.FromString(data)
        self.planning_input=self._drawing_regions(strict=True)
        result=pb.PathPlanResponse(request_id=request.request_id,planned=True)
        return result.SerializeToString(),pb.MSG_ID_PATH_PLAN_RESPONSE,pb.COMP_SCHEDULER


def test_large_region_snapshot_is_chunked_without_losing_foundations():
    import hashlib
    response=pb.MapRegionPointResponse(map_id='map-one',map_version=7,message='x'*140000)
    response.obstacle_regions.add(region_id='drawing:one',source='drawing')
    chunks=region_response_frames(response,pb)
    assert isinstance(chunks,list) and len(chunks)==3
    envelopes=[pb.MapRegionPointResponse.FromString(data) for data,_,_ in chunks]
    assert all(len(data)<65535 for data,_,_ in chunks)
    assert len(set(r.transfer_id for r in envelopes))==1
    joined=b''.join(r.chunk_data for r in sorted(envelopes,key=lambda r:r.chunk_index))
    assert hashlib.sha256(joined).hexdigest()==envelopes[0].payload_sha256
    assert pb.MapRegionPointResponse.FromString(joined)==response


def test_restart_uses_saved_binding_before_map_service_is_ready():
    node=Node();upload(node._drawings());commit(node._drawings())
    saved=SimpleNamespace(image=np.zeros((100,100),dtype=np.uint8),resolution=.1,origin_x=0.,origin_y=0.)
    node._load_saved_raw_grid=lambda map_id:saved
    # Restored overlay/selected map, but the first saved raw grid is not loaded.
    node.map_service=MapService()
    assert node._drawing_binding()==dict(binding(),heading_deg=0.)
    assert node._drawing_regions(strict=True)
    assert node._drawings().layer('map-one')['info']['status']==pb.DRAWING_PUBLISHED
    # A different service geometry must not replace this saved map's binding.
    node.map_service=SimpleNamespace(get_map_info=lambda:dict(binding(),width=800,origin_x=30.))
    assert node._drawing_binding()==dict(binding(),heading_deg=0.)


def test_unavailable_saved_map_does_not_permanently_invalidate_layer():
    node=Node();upload(node._drawings());commit(node._drawings())
    def unavailable(map_id):
        raise ValueError('saved raster unavailable')
    node._load_saved_raw_grid=unavailable
    assert node._drawing_regions()==[]
    assert node._drawings().layer('map-one')['info']['status']==pb.DRAWING_PUBLISHED


def test_small_region_response_retains_legacy_wire_shape():
    response=pb.MapRegionPointResponse(map_id='map-one')
    result=region_response_frames(response,pb)
    assert isinstance(result,tuple) and pb.MapRegionPointResponse.FromString(result[0])==response


def test_scheduler_draft_preview_does_not_paint_navigation_map():
    node=Node();before=node.map_service.compose_map().copy()
    assert upload(node._drawings()).complete
    request=pb.PathPlanRequest(map_id='map-one',request_id='preview',drawing_draft_id='draft-one',drawing_draft_version=1)
    data,_,_=node._handle_path_plan_request_impl_core(request.SerializeToString(),task_scoped=False)
    assert pb.PathPlanResponse.FromString(data).drawing_draft_version==1
    assert len(node.planning_input)==1
    assert np.array_equal(before,node.map_service.compose_map())
    assert not node.map_service.get_overlay_regions()['obstacle_regions']


def test_map_save_commits_drawing_once_and_binds_real_map_id():
    node=Node();assert upload(node._drawings()).complete
    request=pb.MapSaveRequest(map_id='map-one',drawing_draft_id='draft-one',drawing_draft_version=1,request_id='save-once')
    data,_,_=node.handle_map_save_request(request.SerializeToString())
    result=pb.MapSaveResponse.FromString(data)
    assert result.result==pb.RESULT_SUCCESS and result.map_id=='map-one' and result.drawing_layer_version==1
    data,_,_=node.handle_map_save_request(request.SerializeToString())
    assert pb.MapSaveResponse.FromString(data).drawing_layer_version==1
    assert node.original_save_calls==1


def test_pending_failure_blocks_start_without_touching_rasters():
    node=Node();assert upload(node._drawings()).complete
    before=node.map_service.compose_map().copy()
    def failed(data):return pb.MapSaveResponse(result=pb.RESULT_FAILED,message='disk').SerializeToString(),pb.MSG_ID_MAP_SAVE_RESPONSE,pb.COMP_SCHEDULER
    node._handle_map_save_without_drawing=failed
    request=pb.MapSaveRequest(map_id='map-one',drawing_draft_id='draft-one',drawing_draft_version=1,request_id='save-failed')
    data,_,_=node.handle_map_save_request(request.SerializeToString())
    assert pb.MapSaveResponse.FromString(data).result==pb.RESULT_FAILED
    assert node._drawings().layer('map-one')['pending']
    assert np.array_equal(before,node.map_service.compose_map())


def test_legacy_generic_obstacle_edit_cannot_rasterize_drawing():
    node=Node();before=node.map_service.compose_map().copy()
    request=pb.MapEditCommand(operation=pb.MAP_EDIT_OP_UPSERT_OBSTACLE_REGION)
    request.region.region_id='drawing:one';request.region.points.extend([pb.PolygonPoint(x=4,y=4),pb.PolygonPoint(x=5,y=4),pb.PolygonPoint(x=5,y=5)])
    data,_,_=node.handle_map_edit_command(request.SerializeToString())
    assert pb.MapEditResponse.FromString(data).result==pb.RESULT_INVALID_PARAM
    assert np.array_equal(before,node.map_service.compose_map())


def test_ordinary_obstacle_keeps_original_raster_semantics():
    node=Node();request=pb.MapEditCommand(operation=pb.MAP_EDIT_OP_UPSERT_OBSTACLE_REGION)
    request.region.region_id='ordinary';request.region.points.extend([pb.PolygonPoint(x=4,y=4),pb.PolygonPoint(x=5,y=4),pb.PolygonPoint(x=5,y=5)])
    node.handle_map_edit_command(request.SerializeToString())
    assert node.map_service.compose_map()[44,44]==100


def test_draft_confirmation_does_not_rasterize_or_check_clearance(monkeypatch):
    node=Node();assert upload(node._drawings()).complete
    def forbidden(*args,**kwargs):
        raise AssertionError('draft validation must not allocate a map raster')
    monkeypatch.setattr(node.map_service,'compose_map',forbidden)
    monkeypatch.setattr('grinder_scheduler.drawing_obstacles.foundation_mask',forbidden)
    request=pb.DrawingDraftRequest(map_id='map-one',operation=pb.DRAWING_VALIDATE,
        draft_id='draft-one',draft_version=1,request_id='validate')
    data,_,_=node.handle_drawing_draft_request(request.SerializeToString())
    response=pb.DrawingDraftResponse.FromString(data)
    assert response.result==pb.RESULT_SUCCESS, response.message
    assert not any(check.code=='NARROW_GAP' for check in response.checks)


def test_live_map_cannot_be_used_as_final_drawing_binding():
    node=Node();upload(node._drawings())
    request=pb.MapSaveRequest(map_id='LIVE_MAP',drawing_draft_id='draft-one',drawing_draft_version=1,request_id='save')
    data,_,_=node.handle_map_save_request(request.SerializeToString())
    assert pb.MapSaveResponse.FromString(data).result==pb.RESULT_FAILED
    assert node.original_save_calls==0


def test_task_cannot_execute_an_unpublished_drawing_draft():
    node=Node();upload(node._drawings())
    request=pb.PathPlanRequest(map_id='map-one',task_id='task',drawing_draft_id='draft-one',drawing_draft_version=1)
    data,_,_=node._handle_path_plan_request_impl_core(request.SerializeToString())
    assert pb.PathPlanResponse.FromString(data).result==pb.RESULT_INVALID_PARAM


def test_drawings_are_injected_in_real_scheduler_sync_without_painting_map():
    # Existing offline harness supplies only ROS imports. Use the real sync method.
    from test_live_map_save import SchedulerNode
    node=SchedulerNode.__new__(SchedulerNode)
    node.map_service=service();node.task_config=TaskConfigModel();node._persist_state_enabled=False
    node._current_map_id=lambda:'map-one';node._find_recorded_map_by_id=lambda _:dict(map_revision='raw-one')
    node._live_map_id='LIVE_MAP'
    node._load_saved_raw_grid=lambda _:SimpleNamespace(image=np.zeros((100,100),dtype=np.uint8),resolution=.1,origin_x=0.,origin_y=0.)
    node._plan_use_all_work_regions=False
    upload(node._drawings());commit(node._drawings())
    before=node.map_service.compose_map().copy()
    node._sync_task_regions_from_overlay(update_task_binding=False)
    assert any(r.get('source')=='drawing' for r in node.task_config.obstacle_regions)
    assert not node.map_service.get_overlay_regions()['obstacle_regions']
    assert np.array_equal(before,node.map_service.compose_map())


def test_path_point_adapter_forwards_draft_instead_of_dropping_fields():
    from test_live_map_save import SchedulerNode
    node=SchedulerNode.__new__(SchedulerNode);node.sl_link_server=SimpleNamespace(pb=pb)
    captured=[]
    def plan(data,**kwargs):
        captured.append(pb.PathPlanRequest.FromString(data))
        return [(pb.PathPointPlanResponse(drawing_draft_version=7).SerializeToString(),pb.MSG_ID_PATH_POINT_PLAN_RESPONSE,pb.COMP_SCHEDULER)]
    node.handle_path_plan_request=plan
    request=pb.PathPointPlanRequest(map_id='map-one',request_id='req',drawing_draft_id='draft',drawing_draft_version=7)
    node.build_path_point_plan_chunks(request.SerializeToString())
    assert captured[0].drawing_draft_id=='draft' and captured[0].drawing_draft_version==7


def test_failed_point_preview_preserves_reason_and_validated_drawing_versions():
    from test_live_map_save import SchedulerNode
    node=SchedulerNode.__new__(SchedulerNode);node.sl_link_server=SimpleNamespace(pb=pb)
    reason='全局路径或车体包络与图纸地基相交，无法生成安全路径'
    response=pb.PathPlanResponse(result=pb.RESULT_FAILED,planned=False,message=reason,
        drawing_draft_version=7,drawing_layer_version=2)
    node.handle_path_plan_request=lambda *args,**kwargs: (response.SerializeToString(),pb.MSG_ID_PATH_PLAN_RESPONSE,pb.COMP_SCHEDULER)
    def serialize(**kwargs):
        chunk=pb.PathPointPlanResponse(request_id=kwargs['request_id'],planned=kwargs['planned'],
            result=pb.RESULT_FAILED,message=kwargs['message'])
        return [(chunk.SerializeToString(),pb.MSG_ID_PATH_POINT_PLAN_RESPONSE,pb.COMP_SCHEDULER)]
    node._serialize_path_point_plan_chunks=serialize
    request=pb.PathPointPlanRequest(map_id='map-one',request_id='failed-preview',drawing_draft_id='draft',drawing_draft_version=7)
    chunks=node.build_path_point_plan_chunks(request.SerializeToString())
    actual=pb.PathPointPlanResponse.FromString(chunks[0][0])
    assert not actual.planned and actual.message==reason and actual.request_id=='failed-preview'
    assert actual.drawing_draft_version==7 and actual.drawing_layer_version==2
