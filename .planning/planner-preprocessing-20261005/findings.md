# 证据与实现决定

mst27 的 stage obstacles 使用新 logical_or 数组而非原地写；核心网格只读，适配器先把障碍/擦除/裁剪应用到 binary grid。
build_angle_aligned_inflated 包含贴地图边界的连通域分类和最大尺寸判断，ROI 会改变几何结果，本次保留全图范围。
C++ ABI 接收 int32；保留 ABI，Python 缓存 int32 转换。Python 常驻 occupancy 用 uint8，不保存 float64 距离场；同请求的同图同角度直接复用最终规划图。
缓存上下文固定地图快照及三项像素预处理参数，角度用精确值，不舍入；不同请求不共享缓存，避免 map_version 未更新/配置旁路写入导致错用。
需连 base inflation 与角度图一起复用，并缓存诊断 occupied count，避免只移除 copy 后日志 np.sum 仍反复全图扫描。
