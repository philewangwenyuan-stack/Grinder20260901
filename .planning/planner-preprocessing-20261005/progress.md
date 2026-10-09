# 进度

核对现有改动，准备增加请求级缓存。Windows Python310/NumPy/SciPy 可用性待验证；之前内存问题不假定本次持续存在。

已实现请求级只读uint8地图/基础膨胀、精确角度LRU、C++int32转换缓存、适配器复用。Python3.8语法通过，Windows依赖本轮正常。首次9项测试中7项通过/跳过，2项失败因为旧算法在障碍地图y方向已返回空；修正黄金对比为同时验证既有接受路径和拒绝结果，不改变算法。WSL可启动，Python3.8/Numpy1.17/Scipy1.3可用；Python未安装pybind11模块，继续检查系统头文件，不安装依赖。

系统已有pybind11/Python3.8开发头文件；在artifacts/native重新编译未修改的C++内核，未覆盖现有.so。Linux9项核心回归全部通过（含实际native），3项适配器集成通过。适配器首轮fixture缺width/height，第二轮缺合适内部起终点导致旧模式空路径，补齐fixture后验证成功；未修改生产算法来迎合测试。
1000²地图10区3次独立进程完整适配器基准：旧中位6523.53ms，新782.78ms，降88.00%，2451点及所有records digest一致。不是RK3588实测。继续3000²单样本验证资源上界。

3000²整条WSL基准命令退出1且无输出/结果，不推断为规划器缺陷。主机剩余虚拟内存约2.55GiB，WSL进程已退出；保留1000²成功基准，尝试直接Windows进行大图缓存占用验证，实际RK3588仍待验。Windows最终10核心（跳过1 native）+3适配（跳过1 native）+4方向共15通过/2跳过；Linux此前9核心+3适配均通过。新增零预算路径测试也通过Windows。

最终Linux从当前source执行17项测试全部通过，无跳过；C++使用artifacts/native新编译内核。实际runtime backend校验的1000²单样本6514.60→800.55ms，path digest一致。Windows3000²缓存检查完成：base18MB+derived81MB=99MB（不是RSS），3miss/27hit。
完成RESULTS.md、mst27 README共享缓存/测试说明、AGENTS定位表。Python3.8语法/diff检查通过，原有F04/F05暂存与用户删除未覆盖；未提交。ROI/跨请求cache/硬件/RK3588性能明确未宣称完成。
