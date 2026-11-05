import types

import numpy as np
from test_planner_preprocessing import adapter,adapter_module,mst,TaskConfigModel
from grinder_scheduler.drawing_obstacles import path_avoids_foundations


def inputs():
    grid=np.zeros((200,200),dtype=np.int16)
    grid[0,:]=grid[-1,:]=grid[:,0]=grid[:,-1]=100
    info=dict(width=200,height=200,resolution=.1,origin_x=0.,origin_y=0.,frame_id='map')
    foundation=dict(region_id='drawing:one',source='drawing',enabled=True,inflation_m=.2,
        points=[dict(x=x,y=y) for x,y in [(9,9),(11,9),(11,11),(9,11)]])
    work=dict(region_id='work',name='work',points=[dict(x=x,y=y) for x,y in [(2,2),(18,2),(18,18),(2,18)]],enabled=True,
        start_pose=dict(x=3.,y=3.),end_pose=dict(x=17.,y=17.))
    config=TaskConfigModel(work_regions=[work],obstacle_regions=[foundation],vehicle_width=.4,vehicle_length=.6,
        default_path_spacing=.8,inflation_radius=.3,endpoint_margin=.7,overlap_ratio=0.,turn_radius=.2)
    return grid,info,config


def test_real_planner_avoids_foundation_without_mutating_navigation_grid():
    grid,info,config=inputs();before=grid.copy()
    result=adapter().plan('drawing-task',grid,info,config)
    assert result.points
    assert np.array_equal(before,grid)
    mask=adapter_module.foundation_mask(grid.shape,info,config.obstacle_regions)
    assert path_avoids_foundations(result.points,mask,info,footprint=[[-.3,-.2],[.3,-.2],[.3,.2],[-.3,.2]])


def test_ignored_foundation_does_not_change_legacy_fallback_behavior():
    grid,info,config=inputs();config.obstacle_regions[0]['drawing_action']=1
    planner=adapter();planner._planner_module=None;planner._planner_import_error=None
    assert planner.plan('ignored',grid,info,config).points


def test_preview_and_task_return_planner_result_with_drawing_raster_without_extra_guard():
    grid,info,config=inputs();before=grid.copy();planner=adapter()
    candidate=types.SimpleNamespace(points=[dict(x=10.,y=10.)])
    captured=[]
    def plan(task_id,planning_grid,map_info,task_config):
        captured.append(planning_grid.copy())
        return candidate
    planner._plan_without_drawing=plan
    for task_id in ('', 'task'):
        config.task_id=task_id
        assert planner.plan(task_id or 'preview',grid,info,config) is candidate
        assert captured[-1][100,100]==100
        assert np.array_equal(before,grid)
