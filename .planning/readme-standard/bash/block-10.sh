ip -br addr
ip route
# 将下面变量替换为实际雷达 IP，再检查对应出口网卡
LIVOX_IP=192.168.1.100
ip route get "$LIVOX_IP"
