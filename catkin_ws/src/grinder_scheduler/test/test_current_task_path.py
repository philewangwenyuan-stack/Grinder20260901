"""Read-only current-path requests never plan or touch task execution."""
import json
import threading
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

ROOT=Path(__file__).resolve().parents[4]
sys.path.insert(0,str(ROOT/'catkin_ws/src/grinder_scheduler/src'))
sys.path.insert(0,str(ROOT/'catkin_ws/src/grinder_chassis_driver/src'))
from test_drawing_integration import TaskConfigModel
from test_live_map_save import SchedulerNode, pb


def node_with_path(state):
    node=SchedulerNode.__new__(SchedulerNode)
    node.sl_link_server=SimpleNamespace(pb=pb)
    node._active_map_id='map-one';node._live_map_id='LIVE_MAP';node._live_map_source_frame='map'
    node.task_config=TaskConfigModel(task_id='task-one',map_id='map-one')
    node.state=state;node._exec_active=True;node.current_path_index=1
    node._motion=SimpleNamespace(lock=threading.RLock())
    node._chassis_settings={'run_speed':.1}
    node.map_service=SimpleNamespace(get_map_info=lambda:dict(map_version=4))
    node.current_path=SimpleNamespace(task_id='task-one',path_version=7,length_m=2.,
        points=[dict(x=1.,y=2.,path_type='work'),dict(x=3.,y=2.,path_type='work')])
    def forbidden(*args,**kwargs):
        raise AssertionError('read-only query reached planner')
    node.handle_path_plan_request=forbidden
    return node


@pytest.mark.parametrize('state',['RUNNING','PAUSED','READY'])
def test_existing_path_is_returned_without_changing_execution(state):
    node=node_with_path(state);original=node.current_path;config=node.task_config
    request=pb.PathPointPlanRequest(task_id='task-one',map_id='map-one',request_id='query',
        read_only=True,force_replan=True,max_chunk_size=256)
    frames=node.build_path_point_plan_chunks(request.SerializeToString())
    chunks=[pb.PathPointPlanResponse.FromString(data) for data,_,_ in frames]
    result=json.loads(b''.join(c.data for c in chunks))
    assert all(c.request_id=='query' and c.planned and c.result==pb.RESULT_SUCCESS for c in chunks)
    assert result['task_id']=='task-one' and result['map_id']=='map-one'
    assert result['path_version']==7 and len(result['points'])==2
    assert node.current_path is original and node.task_config is config
    assert node.state==state and node._exec_active and node.current_path_index==1


@pytest.mark.parametrize('task,map_id',[('another-task','map-one'),('task-one','another-map')])
def test_other_task_or_map_cannot_receive_current_path(task,map_id):
    node=node_with_path('RUNNING')
    request=pb.PathPointPlanRequest(task_id=task,map_id=map_id,request_id='wrong',read_only=True)
    frames=node.build_path_point_plan_chunks(request.SerializeToString())
    chunks=[pb.PathPointPlanResponse.FromString(data) for data,_,_ in frames]
    assert all(not c.planned and c.result==pb.RESULT_FAILED for c in chunks)
    assert json.loads(b''.join(c.data for c in chunks))['points']==[]
    assert node.state=='RUNNING' and node._exec_active


def test_missing_path_reports_failure_without_planning():
    node=node_with_path('RUNNING');node.current_path=None
    request=pb.PathPointPlanRequest(task_id='task-one',request_id='missing',read_only=True)
    frames=node.build_path_point_plan_chunks(request.SerializeToString())
    chunk=pb.PathPointPlanResponse.FromString(frames[0][0])
    assert not chunk.planned and chunk.message=='Current task has no stored path'
    assert node.current_path is None and node._exec_active
