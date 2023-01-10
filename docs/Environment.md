# RK3588 平台环境搭建



## 准备工作

更换软件源为国内镜像源

```bash
$ sudo cp /etc/apt/sources.list /etc/apt/sources.list.org
$ sudo sed -i -e 's/ports.ubuntu.com/mirrors.ustc.edu.cn/g' /etc/apt/sources.list
```

更新软件包列表

```bash
$ sudo apt-get update
```

将当前用户（非 root 用户）添加到 dialout 用户组（重启系统后生效）

```bash
sudo usermod -a -G dialout $USER
```



## 安装软件包

```bash
$ sudo apt install gpiod libgpiod-dev
$ sudo apt install gpsd gpsd-clients pps-tools linuxptp chrony
$ sudo apt install libpcap-dev libyaml-cpp-dev
```



## 安装 MVS

在 [HIKROBOT 官网](https://www.hikrobotics.com/cn/machinevision/service/download) 下载最新的 MVS 软件包，例如 MVS_STD_GML_V2.1.2_221024-linux.zip，解压缩后得到多个不同硬件平台的软件包，将 MVS-2.1.2_aarch64_20221024.deb 拷贝到 RK3588 系统中，执行下面命令安装。

```bash
sudo dpkg -i MVS-2.1.2_aarch64_20221024.deb
```

默认安装目录为 `/opt/MVS`，执行下面命令启动 MVS 客户端程序。

```bash
cd /opt/MVS/bin/
./MVS.sh
```



