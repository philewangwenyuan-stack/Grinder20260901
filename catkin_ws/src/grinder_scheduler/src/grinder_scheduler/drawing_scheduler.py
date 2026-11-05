"""Scheduler integration for vector-only drawing obstacles. No ROS map writes."""
import threading
import hashlib
import secrets
from copy import deepcopy

from grinder_scheduler.drawing_obstacles import DrawingStore, as_dict

_STORE_INIT_LOCK=threading.Lock()


def region_response_frames(response,pb):
    data=response.SerializeToString()
    if len(data)<=65000:
        return data,pb.MSG_ID_MAP_REGION_POINT_RESPONSE,pb.COMP_SCHEDULER
    if len(data)>16*1024*1024:
        error=pb.MapRegionPointResponse(result=pb.RESULT_FAILED,map_id=response.map_id,message="区域快照超过16 MiB，请精简边界")
        return error.SerializeToString(),pb.MSG_ID_MAP_REGION_POINT_RESPONSE,pb.COMP_SCHEDULER
    transfer=secrets.token_hex(16);sha=hashlib.sha256(data).hexdigest()
    total=(len(data)+49151)//49152
    result=[]
    for index in range(total):
        envelope=pb.MapRegionPointResponse(map_id=response.map_id,map_version=response.map_version,
            transfer_id=transfer,chunk_index=index,total_chunks=total,payload_sha256=sha,chunk_data=data[index*49152:(index+1)*49152])
        result.append((envelope.SerializeToString(),pb.MSG_ID_MAP_REGION_POINT_RESPONSE,pb.COMP_SCHEDULER))
    return result


class DrawingObstacleMixin:
    def _drawing_ensure_map(self,force=False):
        layer=self._drawings().layer(self._current_map_id())
        if force or layer.get("info") or layer.get("pending"):
            ensure=getattr(self,"_ensure_offline_map_service_for_current_map",None)
            if ensure:
                ensure("drawing_geometry_preflight")
    def _drawing_catalog_status(self,response):
        for item in response.items:
            if hasattr(item,"drawing_layer_status"):
                status,version=self._drawings().summary(item.map_id,getattr(item,"map_revision",""))
                item.drawing_layer_status=status
                item.drawing_layer_version=version

    def _drawing_forget_task(self):
        store=getattr(self,"_drawing_obstacle_store",None)
        # Never wait on CAD validation/disk writes on a motion-stop path.
        if store is not None:
            store.forget_tasks()

    def handle_map_edit_command(self,payload):
        pb=self.sl_link_server.pb;request=pb.MapEditCommand();request.ParseFromString(payload)
        if str(request.region.region_id).startswith("drawing:") or str(request.target_region_id).startswith("drawing:") or getattr(request.region,"source","")=="drawing":
            response=pb.MapEditResponse(result=pb.RESULT_INVALID_PARAM,message="图纸地基请通过图纸导入步骤编辑，不能写为普通栅格禁区")
            return response.SerializeToString(),pb.MSG_ID_MAP_EDIT_RESPONSE,pb.COMP_SCHEDULER
        return self._handle_map_edit_without_drawing(payload)

    def _drawings(self):
        store=getattr(self,"_drawing_obstacle_store",None)
        if store is None:
            with _STORE_INIT_LOCK:
                store=getattr(self,"_drawing_obstacle_store",None)
                if store is None:
                    root=getattr(self,"_persist_state_dir",None) if getattr(self,"_persist_state_enabled",False) else None
                    store=DrawingStore(root)
                    self._drawing_obstacle_store=store
        return store

    def _drawing_binding(self, map_id=None):
        map_id=map_id or self._current_map_id()
        service=getattr(self,"map_service",None)
        getter=getattr(service,"get_map_info",None)
        info=(getter() or {}) if getter else {}
        # Startup restores the selected map before its MapService raw grid.
        # Bind saved drawings to the saved raster, never to an empty/live service
        # or to the service belonging to a different map.
        is_live=getattr(self,"_is_live_map_id",lambda value:True)
        load_saved=getattr(self,"_load_saved_raw_grid",None)
        if not is_live(map_id) and load_saved:
            raster=load_saved(map_id)
            info=dict(width=raster.image.shape[1],height=raster.image.shape[0],
                resolution=raster.resolution,origin_x=raster.origin_x,
                origin_y=raster.origin_y,frame_id="map",heading_deg=0.0)
        if int(info.get("width",0))<=0 or int(info.get("height",0))<=0:
            raise ValueError("地图尚未加载完成")
        binding={k:info.get(k,0) for k in ("width","height","resolution","origin_x","origin_y")}
        binding["frame_id"]=info.get("frame_id","map")
        binding["heading_deg"]=float(info.get("heading_deg",0))
        lookup=getattr(self,"_find_recorded_map_by_id",None)
        record=lookup(map_id) if lookup else None
        binding["map_revision"]=str((record or {}).get("map_revision","") or "")
        return binding

    def _drawing_regions(self, strict=False):
        store=self._drawings();map_id=self._current_map_id()
        context=getattr(store.local,"planning",None) or ("",0)
        try:
            return store.effective(map_id,str(self.task_config.task_id or ""),self._drawing_binding(map_id),*context)[0]
        except ValueError:
            if strict:
                raise
            return []

    def _drawing_preflight(self):
        try:
            self._drawing_regions(strict=True)
            return True
        except ValueError as exc:
            self._set_error(str(exc))
            return False

    def handle_drawing_draft_request(self,payload):
        pb=self.sl_link_server.pb;request=pb.DrawingDraftRequest();request.ParseFromString(payload)
        if not request.map_id:
            request.map_id=self._current_map_id()
        try:
            ok,_=self._validate_requested_map_id(request.map_id,"DrawingDraftRequest",keep_current_on_empty=False)
            if not ok:
                raise ValueError("图纸所属地图与当前地图不一致")
            self._drawing_ensure_map(force=True)
            binding=self._drawing_binding(request.map_id)
            if binding["width"]<=0 or binding["height"]<=0:
                raise ValueError("地图尚未加载完成")
            # Pose checks require valid localization on the current map; otherwise
            # start_execution performs the definitive check after localization.
            pose=None
            ready=getattr(self,"_localization_motion_status",None)
            readiness=ready() if ready else (False,"",{})
            if readiness[0] and readiness[2].get("map_id")==request.map_id:
                p=self.ros_data_bridge.get_pose() or {}
                if "x" in p and "y" in p:
                    pose=(float(p["x"]),float(p["y"]))
            response=self._drawings().request(request,pb,binding,pose)
        except Exception as exc:
            response=pb.DrawingDraftResponse(result=pb.RESULT_INVALID_PARAM,message=str(exc),request_id=request.request_id,feature_version=1)
        return response.SerializeToString(),pb.MSG_ID_DRAWING_DRAFT_RESPONSE,pb.COMP_SCHEDULER

    def _handle_path_plan_request_impl_core(self,payload,task_scoped=True,include_preview=True):
        pb=self.sl_link_server.pb;request=pb.PathPlanRequest();request.ParseFromString(payload)
        store=self._drawings();previous=getattr(store.local,"planning",None)
        draft_id=str(getattr(request,"drawing_draft_id","") or "")
        draft_version=int(getattr(request,"drawing_draft_version",0))
        try:
            if draft_id and request.task_id:
                raise ValueError("任务不能使用未发布图纸草稿，请先保存地图")
            store.local.planning=(draft_id,draft_version)
            result=self._handle_path_plan_without_drawing(payload,task_scoped=task_scoped,include_preview=include_preview)
            values=[result] if isinstance(result,tuple) else result
            info=store.layer(self._current_map_id()).get("info",{})
            updated=[]
            for data,msg_id,component in values:
                cls=pb.PathPlanResponse if int(msg_id)==int(pb.MSG_ID_PATH_PLAN_RESPONSE) else pb.PathPointPlanResponse if int(msg_id)==int(pb.MSG_ID_PATH_POINT_PLAN_RESPONSE) else None
                if cls:
                    response=cls();response.ParseFromString(data)
                    response.drawing_layer_version=0 if draft_id else int(info.get("version",0))
                    response.drawing_draft_version=draft_version
                    data=response.SerializeToString()
                updated.append((data,msg_id,component))
            return updated[0] if isinstance(result,tuple) else updated
        except ValueError as exc:
            response=pb.PathPlanResponse(result=pb.RESULT_INVALID_PARAM,message=str(exc),request_id=request.request_id,task_id=request.task_id)
            return response.SerializeToString(),pb.MSG_ID_PATH_PLAN_RESPONSE,pb.COMP_SCHEDULER
        finally:
            store.local.planning=previous

    def handle_map_save_request(self,payload):
        pb=self.sl_link_server.pb;request=pb.MapSaveRequest();request.ParseFromString(payload)
        if not getattr(request,"drawing_draft_id",""):
            return self._handle_map_save_without_drawing(payload)
        response=pb.MapSaveResponse(request_id=request.request_id)
        try:
            # Existing motion gates remain authoritative; do not swap obstacle
            # data underneath an executing task.
            state=str(getattr(getattr(self,"state",None),"value",""))
            if state.upper() in ("RUNNING","EXECUTING","PAUSED"):
                raise ValueError("任务未结束，无法更新图纸障碍")
            store=self._drawings()
            source=store.drafts.get(request.drawing_draft_id,{}).get("map_id",request.map_id)
            target=str(request.map_id or "")
            if self._is_live_map_id(target):
                raise ValueError("新地图提交图纸时须提供稳定的目标地图编号")
            if not request.request_id:
                raise ValueError("保存图纸地图需要请求编号")
            cached=store.prepare_save(source,target,request.drawing_draft_id,request.drawing_draft_version,self._drawing_binding(source),request.request_id)
            if cached is not None:
                return cached,pb.MSG_ID_MAP_SAVE_RESPONSE,pb.COMP_SCHEDULER
            original=pb.MapSaveRequest();original.CopyFrom(request)
            original.ClearField("drawing_draft_id");original.ClearField("drawing_draft_version")
            data,msg_id,component=self._handle_map_save_without_drawing(original.SerializeToString())
            response.ParseFromString(data);response.request_id=request.request_id
            if response.result==pb.RESULT_SUCCESS:
                if str(response.map_id)!=target:
                    raise ValueError("地图保存返回了不同的地图编号，图纸提交已阻止")
                store.finish_save(target,self._drawing_binding(target),response)
                self._path_plan_request_cache_key=None
                self.current_path=None
            return response.SerializeToString(),msg_id,component
        except Exception as exc:
            response.result=pb.RESULT_FAILED;response.message=str(exc)
            return response.SerializeToString(),pb.MSG_ID_MAP_SAVE_RESPONSE,pb.COMP_SCHEDULER
