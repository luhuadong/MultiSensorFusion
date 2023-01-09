# 时间同步方案

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

安装软件包

```bash
$ sudo apt install gpiod libgpiod-dev
$ sudo apt install gpsd gpsd-clients pps-tools linuxptp chrony
```
