"""Map-scoped vector obstacles for GLOBAL planning only. Never paints MapService.

Python 3.8 compatible. Persistent files contain vectors and a commit journal, not
occupancy grids. Navigation exports must continue using MapService.compose_map.
"""
import base64
import hashlib
import json
import math
import os
import struct
import threading
import time
from collections import OrderedDict
from copy import deepcopy

import cv2
import numpy as np
from google.protobuf.json_format import MessageToDict, ParseDict

MAX_CHUNK = 48 * 1024
MAX_PAYLOAD = 8 * 1024 * 1024


def as_dict(message):
    return MessageToDict(message, preserving_proto_field_name=True, use_integers_for_enums=True)


def same_geometry(a, b):
    for key in ("width", "height", "frame_id"):
        if a.get(key, "" if key == "frame_id" else 0) != b.get(key, "" if key == "frame_id" else 0):
            return False
    for key in ("resolution", "origin_x", "origin_y", "heading_deg"):
        left=float(a.get(key,0));right=float(b.get(key,0))
        if not math.isfinite(left) or not math.isfinite(right):
            return False
        if abs(left-right)>1e-6:
            # MapChunk.resolution and Pose2D origin/heading are float32. APP
            # stores that rounded value in DrawingMapBinding's double fields;
            # compare at wire precision rather than declaring a new base map.
            # A coarse float32 ULP at very large coordinates must not hide
            # a physical map shift. Keep rounding allowances below 1 mm
            # (or 0.001 degree); resolution retains the existing tolerance.
            allowance=1e-6 if key=="resolution" else 1e-3
            if abs(left-right)>allowance:
                return False
            try:
                if struct.pack("<f",left)!=struct.pack("<f",right):
                    return False
            except (OverflowError,struct.error):
                return False
    return not (a.get("map_revision") and b.get("map_revision") and a["map_revision"] != b["map_revision"])


def _cross(a, b, c):
    return (b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0])


def _intersects(a, b, c, d):
    ab_c, ab_d = _cross(a, b, c), _cross(a, b, d)
    cd_a, cd_b = _cross(c, d, a), _cross(c, d, b)
    if ab_c * ab_d < 0 and cd_a * cd_b < 0:
        return True
    def on(p, q, r):
        return abs(_cross(p, q, r)) < 1e-9 and min(p[0], q[0])-1e-9 <= r[0] <= max(p[0], q[0])+1e-9 and min(p[1], q[1])-1e-9 <= r[1] <= max(p[1], q[1])+1e-9
    return on(a, b, c) or on(a, b, d) or on(c, d, a) or on(c, d, b)


def validate_payload(payload, binding, pose=None):
    checks = []
    def add(level, code, message, region_id="", position=None):
        item = dict(level=level, code=code, message=message, region_id=region_id)
        if position is not None:
            item["position"] = dict(x=float(position[0]), y=float(position[1]))
        checks.append(item)
    info = payload.get("info", {})
    if payload.get("remove_layer", False):
        if payload.get("foundations"):
            add("error", "REMOVE_WITH_GEOMETRY", "移除图纸层不能同时提交地基")
        if not same_geometry(info.get("binding", {}), binding):
            add("error", "MAP_CHANGED", "地图已变化，请刷新后移除图纸层")
        return checks
    transform = info.get("transform", {})
    rms = float(transform.get("rms_m", 0))
    scale = float(transform.get("scale", 0))
    measured = float(transform.get("measured_scale", scale))
    if not all(math.isfinite(v) for v in (rms, scale, measured)) or rms < 0 or rms > 0.20 or not 0.995 <= measured <= 1.005 or not 0.995 <= scale <= 1.005:
        add("error", "REGISTRATION", "配准误差或比例超出允许范围")
    elif rms > 0.10:
        add("warning", "REGISTRATION", "配准RMS超过10 cm，请核对位置")
    controls = transform.get("control_points", [])
    if len(controls) < 3 or len(controls)>128:
        add("error", "CONTROL_POINTS", "至少需要三对控制点")
    values = [transform.get(key, 0) for key in ("theta_deg", "tx_m", "ty_m")]
    values += [p.get(key, 0) for p in controls for key in ("drawing_x_mm", "drawing_y_mm", "map_x_m", "map_y_m", "residual_m")]
    if not all(math.isfinite(float(v)) for v in values):
        add("error", "CONTROL_POINTS", "控制点或变换包含非有限数值")
    elif 3 <= len(controls) <= 128 and math.isfinite(scale):
        source=np.asarray([(float(p.get("drawing_x_mm",0))/1000,float(p.get("drawing_y_mm",0))/1000) for p in controls],dtype=np.float64)
        target=np.asarray([(float(p.get("map_x_m",0)),float(p.get("map_y_m",0))) for p in controls],dtype=np.float64)
        a=source-source.mean(axis=0);b=target-target.mean(axis=0)
        variance=float(np.sum(a*a))
        dot=float(np.sum(a*b));cross=float(np.sum(a[:,0]*b[:,1]-a[:,1]*b[:,0]))
        if variance<=1e-6 or math.hypot(dot,cross)<=1e-12:
            add("error","CONTROL_POINTS","控制点重复或有效跨度不足")
        else:
            measured_actual=math.hypot(dot,cross)/variance
            angle=math.radians(float(transform.get("theta_deg",0)))
            rotation=np.asarray([[math.cos(angle),-math.sin(angle)],[math.sin(angle),math.cos(angle)]])
            predicted=source.dot(rotation.T)*scale+np.asarray([float(transform.get("tx_m",0)),float(transform.get("ty_m",0))])
            rms_actual=math.sqrt(float(np.mean(np.sum((predicted-target)**2,axis=1))))
            if not .995<=measured_actual<=1.005 or rms_actual>.20 or rms_actual>rms+1e-4:
                add("error","REGISTRATION","控制点复核与所提交配准结果不一致")
    if not same_geometry(info.get("binding", {}), binding):
        add("error", "MAP_CHANGED", "地图底图或坐标几何已变化，请重新对齐")
    foundations = payload.get("foundations", [])
    if not foundations and not payload.get("remove_layer", False):
        add("error", "EMPTY_LAYER", "未选择地基；移除现有层必须明确确认")
    elif foundations and all(int(r.get("drawing_action",0))==1 for r in foundations):
        add("warning","ALL_IGNORED","全部地基均已忽略，本图纸层不会参与全局避障")
    if payload.get("remove_layer", False):
        if foundations:
            add("error", "REMOVE_WITH_GEOMETRY", "移除图纸层不能同时提交地基")
        return checks
    if len(foundations) > 4096 or sum(len(r.get("points", [])) for r in foundations) > 100000:
        add("error", "TOO_MANY_POINTS", "请精简地基边界后重试")
        return checks
    ids = set()
    x0 = float(binding.get("origin_x", 0)); y0 = float(binding.get("origin_y", 0))
    x1 = x0 + int(binding.get("width", 0))*float(binding.get("resolution", 0))
    y1 = y0 + int(binding.get("height", 0))*float(binding.get("resolution", 0))
    for region in foundations:
        rid = str(region.get("region_id", ""))
        points = [(float(p.get("x", 0)), float(p.get("y", 0))) for p in region.get("points", [])]
        margin = float(region.get("inflation_m", 0))
        geometry_error=float(region.get("geometry_error_m",0))
        if not rid or rid in ids:
            add("error", "REGION_ID", "地基编号缺失或重复", rid)
        ids.add(rid)
        if len(points) < 3 or len(points) > 2048 or not all(math.isfinite(v) for p in points for v in p) or not math.isfinite(margin) or not 0 <= margin <= 2 or not math.isfinite(geometry_error) or not 0<=geometry_error<=.02:
            add("error", "INVALID_POLYGON", "边界点或额外安全间距无效", rid)
            continue
        if points[0] == points[-1]:
            points = points[:-1]
        n = len(points)
        area = abs(sum(points[i][0]*points[(i+1)%n][1]-points[(i+1)%n][0]*points[i][1] for i in range(n)))*0.5
        if area < 0.01:
            add("error", "AREA", "地基面积小于0.01平方米", rid)
        crossing = False
        for i in range(n):
            for j in range(i+1, n):
                if j == i+1 or i == 0 and j == n-1:
                    continue
                if _intersects(points[i], points[(i+1)%n], points[j], points[(j+1)%n]):
                    crossing = True
                    break
            if crossing:
                break
        if crossing:
            add("error", "SELF_INTERSECTION", "地基边界自相交", rid)
        poly = np.asarray(points, dtype=np.float32)
        rect = np.asarray([(x0,y0),(x1,y0),(x1,y1),(x0,y1)], dtype=np.float32)
        # Bounding boxes alone cannot establish intersection with a concave polygon.
        inside = [x0 <= x <= x1 and y0 <= y <= y1 for x,y in points]
        overlaps = any(inside) or any(cv2.pointPolygonTest(poly, tuple(p), False) >= 0 for p in rect.tolist())
        if not overlaps:
            overlaps = any(_intersects(points[i], points[(i+1)%n], tuple(rect[j]), tuple(rect[(j+1)%4])) for i in range(n) for j in range(4))
        if not overlaps:
            add("error", "OUTSIDE_MAP", "地基完全位于地图范围外", rid, points[0])
        elif not all(inside):
            add("warning", "PARTIAL_OUTSIDE", "部分边界超出地图，规划时仅作用于范围内区域", rid, points[0])
        if pose is not None and int(region.get("drawing_action", 0)) == 0 and cv2.pointPolygonTest(poly, tuple(pose), True) >= -margin:
            add("error", "ROBOT_INSIDE", "机器人位于地基或额外安全间距内", rid, pose)
    return checks


def foundation_mask(shape, map_info, regions):
    """Conservative rasterization on a new request-owned array, no vertex clamping."""
    mask = np.zeros(shape, dtype=np.uint8)
    resolution = float(map_info["resolution"])
    if not math.isfinite(resolution) or resolution <= 0:
        raise ValueError("invalid map resolution")
    origin = np.asarray([map_info["origin_x"], map_info["origin_y"]], dtype=np.float64)
    for r in regions:
        if not r.get("enabled", True) or int(r.get("drawing_action", 0)) == 1:
            continue
        world = np.asarray([(p["x"],p["y"]) for p in r.get("points", [])], dtype=np.float64)
        if len(world) < 3 or not np.isfinite(world).all():
            raise ValueError("invalid foundation polygon")
        pixels = np.rint((world-origin)/resolution)
        if np.abs(pixels).max() > 2**29:
            raise ValueError("foundation coordinates outside raster range")
        region_mask = np.zeros(shape, dtype=np.uint8)
        cv2.fillPoly(region_mask, [pixels.astype(np.int32)], 1)
        # Include curve approximation and a cell's diagonal for raster rounding.
        margin = max(0, float(r.get("inflation_m", 0))) + max(0, float(r.get("geometry_error_m", 0)))
        radius = int(math.ceil(margin/resolution)) + 1
        if not np.any(region_mask):
            continue
        # Linear memory even with a very fine map resolution or large margin.
        distances=cv2.distanceTransform(1-region_mask,cv2.DIST_L2,cv2.DIST_MASK_PRECISE)
        region_mask=(distances<=radius).astype(np.uint8)
        np.maximum(mask, region_mask, out=mask)
    return mask


def path_avoids_foundations(points, mask, map_info, safety_m=0, footprint=None):
    if mask is None or not np.any(mask):
        return True
    resolution = float(map_info["resolution"])
    radius = int(math.ceil(max(0, safety_m)/resolution))
    occupied=(cv2.distanceTransform(1-mask,cv2.DIST_L2,cv2.DIST_MASK_PRECISE)<=radius).astype(np.uint8) if radius else mask
    if not points:
        return False
    h,w = mask.shape
    body=np.asarray(footprint or [],dtype=np.float64).reshape((-1,2))
    body_radius=float(np.linalg.norm(body,axis=1).max()) if len(body) else 0
    def cell(p):
        return ((float(p["x"])-map_info["origin_x"])/resolution,(float(p["y"])-map_info["origin_y"])/resolution)
    def yaw(p,fallback):
        q=p.get("orientation") or {}
        if not q:
            return math.radians(float(p["heading_deg"])) if "heading_deg" in p else fallback
        x=float(q.get("x",0));y=float(q.get("y",0));z=float(q.get("z",0));wq=float(q.get("w",1))
        return math.atan2(2*(wq*z+x*y),1-2*(y*y+z*z))
    for a,b in zip(points, points[1:] or points):
        ax,ay=cell(a); bx,by=cell(b)
        if not all(math.isfinite(v) for v in (ax,ay,bx,by)):
            return False
        fallback=math.atan2(by-ay,bx-ax)
        ya=yaw(a,fallback);yb=yaw(b,fallback)
        angle=(yb-ya+math.pi)%(2*math.pi)-math.pi
        count=max(1,int(math.ceil(max(math.hypot(bx-ax,by-ay)*2,abs(angle)*body_radius/resolution*2))))
        if count>2000000:
            return False
        for t in np.linspace(0,1,count+1):
            x=int(round(ax+(bx-ax)*t)); y=int(round(ay+(by-ay)*t))
            if x<0 or y<0 or x>=w or y>=h or occupied[y,x]:
                return False
            if len(body):
                angle_t=ya+angle*t;c=math.cos(angle_t);sn=math.sin(angle_t)
                polygon=np.rint(body.dot(np.asarray([[c,sn],[-sn,c]]))/resolution+[ax+(bx-ax)*t,ay+(by-ay)*t]).astype(np.int32)
                lo=polygon.min(axis=0);hi=polygon.max(axis=0)
                if lo[0]<0 or lo[1]<0 or hi[0]>=w or hi[1]>=h:
                    return False
                roi=occupied[lo[1]:hi[1]+1,lo[0]:hi[0]+1]
                if np.any(roi):
                    shape=np.zeros(roi.shape,dtype=np.uint8)
                    cv2.fillPoly(shape,[polygon-lo],1)
                    if np.any(shape & roi):
                        return False
    return True


def clearance_checks(grid,map_info,regions,vehicle_width):
    enabled=[r for r in regions if r.get("enabled",True) and int(r.get("drawing_action",0))==0]
    if not enabled or grid is None:
        return []
    masks=[foundation_mask(grid.shape[:2],map_info,[r]) for r in enabled]
    all_foundations=np.maximum.reduce(masks)
    occupied=(np.asarray(grid)!=0).astype(np.uint8)
    checks=[]
    for region,own in zip(enabled,masks):
        other=np.maximum(occupied,all_foundations)
        other[own>0]=0
        if not np.any(other) or not np.any(own):
            continue
        distance=cv2.distanceTransform(1-other,cv2.DIST_L2,cv2.DIST_MASK_PRECISE)
        gap=float(distance[own>0].min())*float(map_info["resolution"])
        if gap<float(vehicle_width):
            points=region["points"]
            checks.append(dict(level="warning",code="NARROW_GAP",region_id=region.get("region_id",""),
                message="与周边障碍的净距可能不足车宽（{:.2f} m），请在最终路径预览核对".format(vehicle_width),
                position=dict(x=sum(p["x"] for p in points)/len(points),y=sum(p["y"] for p in points)/len(points))))
    return checks


class DrawingStore:
    def __init__(self, root=None):
        self.root = os.path.join(root, "drawing_layers") if root else None
        self.lock = threading.RLock()
        self.layers = OrderedDict()
        self.uploads = {}
        self.drafts = {}
        self.tasks = {}
        self.task_epoch = 0
        self.local = threading.local()

    def _path(self, map_id):
        return os.path.join(self.root, hashlib.sha256(str(map_id).encode()).hexdigest()+".json")

    def _write(self, map_id, value):
        if self.root:
            os.makedirs(self.root, exist_ok=True)
            path = self._path(map_id)
            with open(path+".tmp", "w", encoding="utf-8") as handle:
                json.dump(value, handle, ensure_ascii=False, allow_nan=False)
                handle.flush()
                os.fsync(handle.fileno())
            os.replace(path+".tmp", path)
        self.layers[map_id] = deepcopy(value)
        self.layers.move_to_end(map_id)
        while len(self.layers)>8:
            self.layers.popitem(last=False)

    def _get(self, map_id):
        if map_id not in self.layers:
            value = {}
            if self.root and os.path.exists(self._path(map_id)):
                try:
                    with open(self._path(map_id), encoding="utf-8") as handle:
                        value = json.load(handle)
                    if not isinstance(value,dict) or not isinstance(value.get("foundations",[]),list) or not isinstance(value.get("pending",{}),dict):
                        raise ValueError("invalid drawing state")
                    pending_payload=value.get("pending",{}).get("payload",{})
                    if not isinstance(pending_payload,dict) or not isinstance(value.get("info",pending_payload.get("info")),dict):
                        raise ValueError("invalid drawing state")
                except (OSError,ValueError,TypeError):
                    value=dict(info=dict(status=2,version=0,stale_reason="图纸状态文件损坏，请重新导入"),corrupt=True)
            self.layers[map_id] = value
            while len(self.layers)>8:
                self.layers.popitem(last=False)
        self.layers.move_to_end(map_id)
        return self.layers[map_id]

    def summary(self,map_id,revision=""):
        with self.lock:
            value=self._get(map_id);info=value.get("info",{})
            if info and revision and info.get("binding",{}).get("map_revision") and revision!=info["binding"]["map_revision"]:
                info.update(status=2,stale_reason="原始地图已替换，请重新对齐")
                self._write(map_id,value)
            return (2 if value.get("pending") else int(info.get("status",0)),int(info.get("version",0)))

    def layer(self, map_id, binding=None):
        with self.lock:
            value = self._get(map_id)
            info = value.get("info", {})
            if info and binding and not same_geometry(info.get("binding",{}), binding):
                if info.get("status") != 2:
                    info.update(status=2, stale_reason="地图底图或坐标几何发生变化")
                    self._write(map_id,value)
            elif (info and binding and info.get("status")==2
                  and info.get("stale_reason")=="地图底图或坐标几何发生变化"
                  and not value.get("pending") and not value.get("corrupt")
                  and info.get("binding",{}).get("map_revision")
                  and info["binding"]["map_revision"]==binding.get("map_revision")):
                # Recover a startup false invalidation only after both the full
                # geometry and the original asset revision match again.
                info.update(status=1 if value.get("foundations") else 0,stale_reason="")
                self._write(map_id,value)
            return deepcopy(value)

    def invalidate(self, map_id, reason):
        with self.lock:
            value=self._get(map_id)
            if value.get("info"):
                value["info"].update(status=2,stale_reason=reason)
                self._write(map_id,value)

    def authorize(self, map_id, task_id, skip, expected_version, binding):
        epoch = self.task_epoch
        with self.lock:
            layer=self.layer(map_id,binding); info=layer.get("info",{})
            version=int(info.get("version",0))
            if layer.get("pending"):
                raise ValueError("图纸层提交未完成，请重试保存地图")
            if expected_version and expected_version != version:
                raise ValueError("图纸层版本已变化，请重新预览")
            if info.get("status")==2 and not skip:
                raise ValueError("图纸层已失效，请重新对齐或明确确认本次跳过")
            if epoch != self.task_epoch:
                raise ValueError("任务已停止，请重新确认图纸策略")
            self.tasks.clear()
            self.tasks[(map_id,task_id)]=(version,bool(skip),epoch)

    def forget_tasks(self):
        # Lock-free cancellation: even an authorization already in progress
        # cannot revive a grant, because effective() also checks its epoch.
        self.task_epoch += 1
        self.tasks.clear()

    def effective(self, map_id, task_id, binding, draft_id="", draft_version=0):
        with self.lock:
            if draft_id:
                draft=self.draft(map_id,draft_id,draft_version,binding)
                return deepcopy(draft["payload"].get("foundations",[])),draft["version"],0
            layer=self.layer(map_id,binding);info=layer.get("info",{})
            if layer.get("pending"):
                raise ValueError("图纸层提交未完成")
            version=int(info.get("version",0))
            if info.get("status")==2:
                if self.tasks.get((map_id,task_id)) != (version,True,self.task_epoch):
                    raise ValueError("图纸层已失效，请重新对齐或确认本次跳过")
                return [],0,version
            return deepcopy(layer.get("foundations",[])),0,version

    def draft(self, map_id, draft_id, version, binding):
        item=self.drafts.get(draft_id)
        if not item or item["map_id"]!=map_id or item["version"]!=version or item["expires"]<time.monotonic():
            raise ValueError("图纸草稿不存在或已过期，请重新确认")
        if not same_geometry(item["payload"]["info"].get("binding",{}),binding):
            raise ValueError("地图已变化，请重新对齐图纸")
        current=int(self.layer(map_id).get("info",{}).get("version",0))
        if item["expected"]!=current:
            raise ValueError("已发布图纸层版本冲突，请刷新后重试")
        return item

    def request(self, request, pb, binding, pose=None):
        response=pb.DrawingDraftResponse(request_id=request.request_id, feature_version=1,max_chunk_bytes=MAX_CHUNK)
        try:
            with self.lock:
                now=time.monotonic()
                self.uploads={k:v for k,v in self.uploads.items() if v["expires"]>now}
                self.drafts={k:v for k,v in self.drafts.items() if v["expires"]>now}
                map_id=str(request.map_id)
                if request.operation==pb.DRAWING_CAPABILITIES:
                    response.complete=True
                    ParseDict(self.layer(map_id,binding).get("info",{}),response.layer)
                    return response
                if not request.draft_id or len(request.draft_id)>128:
                    raise ValueError("无效草稿编号")
                if request.operation==pb.DRAWING_DISCARD:
                    for collection in (self.uploads,self.drafts):
                        item=collection.get(request.draft_id)
                        if item and item["map_id"]!=map_id:
                            raise ValueError("草稿地图不匹配")
                        collection.pop(request.draft_id,None)
                    response.complete=True
                    return response
                if request.operation==pb.DRAWING_UPLOAD:
                    total=int(request.total_chunks); index=int(request.chunk_index)
                    if not 1<=total<=math.ceil(MAX_PAYLOAD/MAX_CHUNK) or not 0<=index<total or not request.data or len(request.data)>MAX_CHUNK:
                        raise ValueError("无效图纸分片")
                    done=self.drafts.get(request.draft_id)
                    if done and done["sha"]==request.payload_sha256 and done["map_id"]==map_id:
                        response.draft_id=request.draft_id;response.draft_version=done["version"];response.complete=True
                        return response
                    item=self.uploads.get(request.draft_id)
                    if item is None:
                        if len(self.uploads)>=4 or len(self.drafts)>=8:
                            raise ValueError("图纸草稿数量超过上限，请关闭其他编辑后重试")
                        item=dict(map_id=map_id,sha=request.payload_sha256,total=total,chunks={},expires=now+300,expected=int(request.expected_layer_version))
                        self.uploads[request.draft_id]=item
                    if (item["map_id"],item["sha"],item["total"],item["expected"])!=(map_id,request.payload_sha256,total,int(request.expected_layer_version)):
                        raise ValueError("分片所属草稿不一致")
                    if index in item["chunks"] and item["chunks"][index]!=request.data:
                        raise ValueError("重复分片内容不同")
                    item["chunks"][index]=bytes(request.data)
                    if sum(map(len,item["chunks"].values()))>MAX_PAYLOAD:
                        self.uploads.pop(request.draft_id,None)
                        raise ValueError("图纸边界数据超过8 MiB，请精简")
                    response.draft_id=request.draft_id
                    if len(item["chunks"])!=total:
                        return response
                    data=b"".join(item["chunks"][i] for i in range(total))
                    if hashlib.sha256(data).hexdigest()!=item["sha"]:
                        self.uploads.pop(request.draft_id,None)
                        raise ValueError("图纸数据校验失败")
                    payload=pb.DrawingLayerPayload();payload.ParseFromString(data)
                    value=as_dict(payload)
                    checks=validate_payload(value,binding,pose)
                    for check in checks:
                        ParseDict(check,response.checks.add())
                    if any(c["level"]=="error" for c in checks):
                        self.uploads.pop(request.draft_id,None)
                        response.result=pb.RESULT_INVALID_PARAM
                        response.message="图纸边界检查未通过"
                        return response
                    version=(done["version"]+1) if done else 1
                    self.drafts[request.draft_id]=dict(map_id=map_id,payload=value,sha=item["sha"],expected=item["expected"],version=version,expires=now+86400)
                    self.uploads.pop(request.draft_id,None)
                    self.draft(map_id,request.draft_id,version,binding)
                elif request.operation!=pb.DRAWING_VALIDATE:
                    raise ValueError("不支持的图纸操作")
                item=self.draft(map_id,request.draft_id,request.draft_version or self.drafts[request.draft_id]["version"],binding)
                response.draft_id=request.draft_id;response.draft_version=item["version"];response.complete=True
                ParseDict(item["payload"].get("info",{}),response.layer)
                if request.operation==pb.DRAWING_VALIDATE:
                    checks=validate_payload(item["payload"],binding,pose)
                    for check in checks:
                        ParseDict(check,response.checks.add())
                    if any(c["level"]=="error" for c in checks):
                        response.result=pb.RESULT_INVALID_PARAM
            return response
        except Exception as exc:
            response.result=pb.RESULT_INVALID_PARAM;response.message=str(exc);response.complete=False
            return response

    def prepare_save(self, source_map_id, target_map_id, draft_id, version, binding, request_id):
        with self.lock:
            current=self._get(target_map_id)
            if current.get("last_request_id")==request_id and request_id and current.get("last_response"):
                return base64.b64decode(current["last_response"])
            pending=current.get("pending",{})
            if pending.get("request_id")==request_id and request_id and pending.get("draft_id")==draft_id and pending.get("version")==version:
                if not same_geometry(pending["payload"]["info"].get("binding",{}),binding):
                    raise ValueError("待提交图纸的地图几何已变化，请重新对齐")
                return None
            draft=self.draft(source_map_id,draft_id,version,binding)
            value=deepcopy(current)
            value["pending"]=dict(source_map_id=source_map_id,draft_id=draft_id,payload=draft["payload"],request_id=request_id,version=version)
            self._write(target_map_id,value)
            return None

    def finish_save(self, map_id, binding, response):
        with self.lock:
            current=self._get(map_id);pending=current.get("pending")
            if not pending:
                raise ValueError("图纸提交记录不存在")
            payload=deepcopy(pending["payload"])
            if not same_geometry(payload["info"].get("binding",{}),binding):
                raise ValueError("保存后的地图坐标发生变化，需重新对齐图纸")
            version=int(current.get("info",{}).get("version",0))+1
            info=payload.get("info",{})
            info.update(version=version,status=0 if payload.get("remove_layer") else 1,stale_reason="",binding=binding,published_at_ms=int(time.time()*1000))
            for region in payload.get("foundations",[]):
                region.update(source="drawing",subtype="foundation",layer_version=version,enabled=int(region.get("drawing_action",0))==0)
            response.request_id=pending["request_id"];response.drawing_layer_version=version
            payload.update(last_request_id=pending["request_id"],last_response=base64.b64encode(response.SerializeToString()).decode("ascii"))
            self._write(map_id,payload)
            self.drafts.pop(pending["draft_id"],None)
