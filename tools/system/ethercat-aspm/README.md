# EtherCAT 专用 I225 禁用 ASPM

目标：NUC `0000:58:00.0`，Intel `8086:15f2`。将 `l1_aspm`、`l1_1_aspm`、`l1_1_pcipm` 设为0，保留其他设备的设置。

在 NUC 部署目录执行：

```bash
sudo bash ~/dust-ecat-bench/ethercat-aspm/install.sh
```

安装 `/usr/local/sbin/dust-ethercat-disable-aspm` 和 `/etc/systemd/system/ethercat.service.d/60-disable-aspm.conf`。立即应用，并在每次 EtherCAT 服务启动后重新应用；该服务已开机启用。安装不重启服务或网关。身份不符、控制文件缺失或同名已安装文件内容不符时退出。

安装成功应显示三个开关均为0，最后显示 `enabled`。后续实验脚本的保存/恢复逻辑会保存0并恢复0，不会重新开启ASPM。

2026-10-30：用户已安装成功；随后读取三个开关均为0，systemd drop-in存在，EtherCAT服务开机启用。复位网关后直接以普通用户运行125us延长版启动检查，`OP_CYCLES 1000 WKC_MISSES 0`，Ready校验通过。未重启操作系统，跨重启效果尚未实测。
