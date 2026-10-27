
## 运行

```bash
# 查看可用网卡
./build/ecat_rtt_test

# 需要 root 或 CAP_NET_RAW；网卡必须 up，且不要配置 IP
sudo ./build/ecat_rtt_test enp2s0

关闭中断聚合
sudo ethtool -C enp2s0 rx-usecs 0
