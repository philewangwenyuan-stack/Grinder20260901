# 全局核查证据

既有 AUDIT 20260928/20261004 作为问题索引，结论须对照本轮代码。用户要求全局、细致、效率优先；前后台切换只占 APP 子项。机器人 root HEAD 与前次相同，F04/F05 暂存；APP 工作树无差异。

新增已核对：规划连接失败可降级原始地图再直线（adapter538–735）；完整组合路径没有最终碰撞复检。纯源码 AST 探针直线穿墙为 collision_free=false。A* 八邻域+Manhattan 不可采纳，固定 seed0 20x20 栅格返回33.898，Dijkstra32.726，样例多3.58%，不能外推全局比例。C++同样启发式313。
多圈重复只复制列表，复用点对象并修改 path_type，三圈最终全部 lap3（adapter469–471）；selected IDs全不匹配时回退所有区域444。
C++ plan_core1604–1607没有释放GIL，地图转换后纯C++计算仍持GIL。adapter332每区域整图copy，mst2211–2240重复膨胀/对齐，preprocess O(R*H*W)。待RK3588实测收益。
协议 expected_map_version只有proto663，scheduler未读取；有序队列满时popleft静默丢旧状态请求，日志没有负响应。Python/C SDK未校验version/flags；接收半帧无解析截止时间。平台上传664–676读完整文件后join，上传缓冲至少约2F。
RPP默认confirm15、controller10Hz、front clearance0.25m、maxv0.15；安全预算需现场制动/频率验证。非候选分支调用clearCollisionLatchIfSafe，但未锁存时该方法893直接返回，没有清候选累计数；说明中的连续确认不成立。
回环上限8000*2000点；满时冻结377–384，有界但无长运行滚动策略。metrics事件ring、快照export有界，不能称所有日志/队列无上限。
本轮再次复现F01/F07、F02及R01。Windows原生numpy/scipy/cv2导入打印版本后异常退出，避免把环境OOM/页文件问题解释为机器人内存泄漏。
