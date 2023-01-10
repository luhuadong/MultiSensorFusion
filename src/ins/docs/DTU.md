# DTU 使用说明



INS570 组合导航系统可选配 DTU（包含天线、4G 流量卡等），型号为 MD-649，该 DTU 内嵌厘米级高精度 RTK 服务，可用于连接 INS570 的 RS232 接口，实现高精度定位。



## 准备工作

**硬件**

- 串口线，用于连接 MD-649 DTU 与电脑或用户设备
- 4G 全频段天线
- 电源适配器（5-36V DC）
- 开通 4G 功能并且能访问互联网的 SIM 卡（中国联通、中国电信、中国移动）

**软件**

- DTU 批量配置程序 DTUcfg2.exe



## 串口定义

**RS-232**

| 针脚 | 定义 | 描述 |
| ---- | ---- | ---- |
| 2    | TXD  | 输出 |
| 3    | RXD  | 输入 |
| 5    | GND  | 地线 |

**RS-485**

| 针脚 | 定义 | 描述 |
| ---- | ---- | ---- |
| 7    | A    | 差分 |
| 8    | B    | 差分 |



## 指示灯说明

绿灯 :green_book: 指示上线状态，具体说明如下：

| 状态 | 描述                   |
| ---- | ---------------------- |
| 常亮 | 已经连接到差分数据中心 |
| 熄灭 | 没有连接到差分数据中心 |
| 快闪 | 正在连接差分数据中心   |
| 慢闪 | 正在拨号               |

红灯 :red_circle: 指示发送状态，具体说明如下：

| 状态 | 描述                  |
| ---- | --------------------- |
| 闪烁 | 正在传送数据中 / 待机 |
| 熄灭 | 当前没有数据传送      |



## 配置说明

MD-649 DTU 可以连接 NTRIP 差分定位平台和驿唐的 mServer（驿云），下面介绍 NTRIP 差分定位平台的连接配置。

使用串口线连接 DTU 与电脑，打开 DTUcfg2.exe 配置软件，点击“设置”输入正确的 COM 口编号，然后点击“开始配置”，在 30 秒内迅速接通电源。此时可以看到配置参数列表，点击“导出配置”可以将配置文件导出到电脑，内容大致如下。

```ini
[CFG]
1. NTRIP Caster域名或IP=rtk.ntrip.qxwz.com
2. NTRIP Caster端口=8001
3. NTRIP类型[1:Client/2:Server]=1
4. NTRIP账号=
5. NTRIP密码=
6. 挂载点[1:RTCM32_GGB/2:RTCM30_GG]=1
7. 自定义挂载点:=
8. 数据源[0:无/1:COM1/2:COM2]=1
9. 数据中心2域名或IP=ckt.lianwangbao.com
10. 数据中心2端口=8080
11. 中心2连接mServer[Y/N]=Y
12. 中心2数据源[0:无/1:COM1/2:COM2]=0
13. 数据中心3域名或IP=
14. 数据中心3端口=
15. 中心3连接mServer[Y/N]=N
16. 中心3数据源[0:无/1:COM1/2:COM2]=0
17. 用户名=
18. APN名称=
19. 网络协议[UDP/TCP]=TCP
20. 串口输出连接信息[Y/N]=N
21. 拨号帐号=ctnet@mycdma.cn
22. 拨号密码=vnet.mobi
23. 短信配置密码=1234
24. 心跳间隔秒=60
25. 无mServer时自定义注册包=ETUNG:240305004108335\x00
26. 无mServer时自定义心跳包=ETUNG\x00
27. 串口1波特率bps=9600
28. 串口1数据位bit[7(须带校验)/8]=8
29. 串口1奇偶校验[N/E/O]=N
30. 串口1停止位bit[1/1.5/2]=1
31. 串口2波特率bps=9600
32. 串口2数据位bit[7(须带校验)/8]=8
33. 串口2奇偶校验[N/E/O]=N
34. 串口2停止位bit[1/1.5/2]=1
35. 网络选择[1:自动/2:2G/3:3G/4:4G]=1
36. 调试模式[Y/N]=N
37. 电源管理[1:高性能/2:低功耗]=1
```

参数配置说明：

- 第一项是“数据中心域名或 IP”，第二项是“数据中心端口”，默认为千寻差分定位平台（域名：rtk.ntrip.qxwz.com，端口：8001），用户可根据需要自行修改。
- 另外比较重要的参数是串口参数，默认为波特率 9600、数据位 8、奇偶校验 N、停止位 1，用户需根据定位板卡的串口参数进行配置，保证双方完全一致。
- 接下来配置 NTRIP 协议类型，支持 NTRIP Client 和 NTRIP Server 两种协议，默认为 1，即 NTRIP Client。
  - 如果选择 NTRIP Client 协议，需要配置数据中心账号、密码（账号和密码可在差分定位平台申请）。然后配置挂载点，目前千寻差分定位平台可选两个挂载点（1：RTCM32_GGB，2：RTCM30_GG），默认选 1。
  - 如果选择 NTRIP Server 协议，需要配置密码和自定义挂载点。密码是差分定位平台分配的，自定义挂载点是该 NTRIP Server 的唯一标识号，可配置为该终端的 IMEI 号。

将修改后的配置文件重新导入 MD-649 DTU 使其生效。
