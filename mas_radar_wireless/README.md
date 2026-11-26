# RMUC2026-NC：雷达无线链路仿真与 NanoSDR 工程

本工程实现 RoboMaster 2026 雷达无线链路的协议生成、GFSK 调制/解调、NanoSDR 控制和网页调试界面。它既可在没有硬件时完成纯软件协议回环，也可接入 NanoSDR 完成单机回环、独立发射、独立接收及多机部署调试。

Python 负责协议、状态、网页 GUI、ROS 2 和 GNU Radio 的通信；GNU Radio 负责调制、解调、SDR 收发、频谱显示和功率遥测。两者通过本机 ZMQ 数据链路与 XMLRPC 参数控制链路协作。

> 本文只说明 **NanoSDR 虚拟网卡（IP）直连**方式。USB 直通、虚拟串口、板端串口登录等方式不在本工程的使用范围内。

**该项工程是ROS2的一个包，可以自行添加到原有的雷达工作空间下的src目录内。**

## 1. 功能与运行模式

| 模式 | TX | RX | SDR 数据链路 | 适用场景 |
| --- | --- | --- | --- | --- |
| **debug** | 开启 | 开启 | 内存中的虚拟 SDR | 纯软件协议回环，不需要 GNU Radio 或硬件 |
| **loopback** | 开启 | 开启 | GNU Radio + NanoSDR | 单台电脑、单 SDR 的硬件回环 |
| **tx** | 开启 | 关闭 | GNU Radio + NanoSDR | 独立信息波/干扰波发射端 |
| **rx** | 关闭 | 开启 | GNU Radio + NanoSDR | 独立接收与解调端 |

**debug** 用于验证 Python 协议层，**loopback** 用于验证完整收发流程。三机部署时，两台发射机运行 **tx**，接收机运行 **rx**；所有设备均使用同一份流图，根据角色关闭不使用的链路并将无用发射增益降至最低。

### 硬件选型：NanoSDR+羊角天线

SDR一个用来接收，两个用来发射。

链接如下：

【淘宝】https://e.tb.cn/h.89GjRfYWL3qhrce?tk=hYlCT2EXAa4 HU591 「PlutoSDR 兼容NanoSDR 软件无线电 ZYNQ7010 AD9363 开发板」

![](.\picture\SDR.png)

【淘宝】https://e.tb.cn/h.8QLwHp3LSe2B3bG?tk=PUdWT2EXxed CZ007 「433MHZ/GSM羊角天线T型无线智能家居扫地机贴片式接收发射天线」

![](.\picture\天线.png)

## 2. 仓库内容

~~~text
.
├── grc/
│   ├── RMUC2026_NanoSDR_TRX_SIC.grc  # 主 GNU Radio 流图
│   └── block_rotating_file_sink.py   # 自定义 IQ 分段录制块
├── msg/                              # ROS 2 自定义消息定义
├── picture/                          # README所引用图片
├── resource/radar_parse_em_wave      # ROS 2 包索引标记，必须保留
├── rmuc/
│   ├── coder.py                      # CRC、串口帧、空口帧、业务包编码/解码
│   ├── constants.py                  # 规则参数、频点、带宽、端口和 CRC 表
│   ├── state.py                      # 运行状态与 SDR 参数快照
│   ├── tx.py / rx.py                 # 发送与接收工作线程
│   ├── sdr_link.py                   # 虚拟 SDR 与 ZMQ 数据面
│   ├── sdr_control.py                # XMLRPC 参数同步与 RX 增益自适应
│   ├── node.py                       # 软件节点总控
│   ├── ros2_bridge.py                # ROS 2 话题桥接
│   ├── flowgraph_runtime.py          # GNU Radio Python 环境探测
│   └── web_gui.py、web/index.html    # 网页调试台
├── scripts/
│   ├── rmuc_node.py                  # 脚本模式入口
│   └── rmuc_ros2_node.py             # ROS 2 入口
├── CMakeLists.txt、package.xml       # ROS 2 构建与安装配置
└── setup.py、setup.cfg               # Python 包安装配置
~~~

GNU Radio Companion 自动生成的流图 Python 脚本不纳入版本控制，需要本地生成。**grc/block_rotating_file_sink.py** 是手写自定义块，必须保留。

## 3. 环境准备（调试环境）

### 3.1 WSL 与 GNU Radio

推荐在 Windows 的 WSL 2 Ubuntu 22.04 环境中运行 GNU Radio。需要 WSLg 图形界面能力，并使 WSL 处于 mirrored networking（镜像网络）模式，以便 WSL、Windows 和 NanoSDR 位于同一网络可达范围。

安装常用工具：

~~~bash
sudo apt update
sudo apt install -y git curl ca-certificates xterm
~~~

GNU Radio 可使用系统包，或安装 Radioconda。若使用 Radioconda，建议安装在 Linux 主目录，随后验证流图所需模块：

~~~bash
source "$HOME/radioconda/bin/activate"
which python
which gnuradio-companion
python -c "from gnuradio import gr, iio, zeromq, qtgui; import numpy; print('GNU Radio import OK')"
~~~

**xterm** 仅用于补齐部分 GNU Radio 图形运行场景依赖；若看到 **missing xterm executable**，安装它即可。未使用音频模块时，ALSA 音频警告通常不影响射频流图运行。

ROS 2 节点可以使用系统 Python，而流图可使用 Radioconda Python。不要强行把两者混为同一个 Python 环境；通过 **--flowgraph-python** 将流图解释器明确传给节点即可。

### 3.2 虚拟网卡直连 NanoSDR

当前流图默认主 SDR URI 为 **ip:192.168.2.1**。虚拟网卡模式下，主机常见地址为 **192.168.2.10**；实际地址应以本机网络配置和设备设置为准。

在 Windows PowerShell 中确认主机可达设备：

~~~powershell
ping 192.168.2.1
~~~

确认 WSL 的镜像网络互通：先在 Windows PowerShell 启动一个本机 HTTP 服务，随后在 WSL 中访问它并测试 SDR 地址。

~~~powershell
python -m http.server 18080 --bind 127.0.0.1
~~~

~~~bash
curl http://127.0.0.1:18080
ping 192.168.2.1
iio_info -u ip:192.168.2.1 | grep -E "IIO context|Backend|Device|ad936|cf-ad936|dds|xadc|voltage"
~~~

curl 能返回目录页面且 ping、iio_info 正常时，说明 WSL 可经虚拟网卡访问 NanoSDR。iio_info 输出中通常应能看到 **ad9361-phy**、**cf-ad9361-lpc**、**cf-ad9361-dds-core-lpc** 等 IIO 设备。

若使用第二发射源，请在流图变量 **sdr_uri2** 中设置第二台 SDR 的 IP；默认值为 **ip:192.168.20.1**。**sdr_uri1** 同时供主 TX Sink 和 RX Source 使用。

**注意！需要在插上SDR设备的USB口之后在电脑修改SDR内的配置文件用于修改SDR的IP地址！**

## 4. 首次打开 GNU Radio 流图

主流图使用 Embedded Python Block 导入 **grc/block_rotating_file_sink.py**。**手动打开 GRC 前必须在同一个终端设置 PYTHONPATH**，否则会出现 **No module named 'block_rotating_file_sink'**。

~~~bash
cd /path/to/sp_rf_2026-OpenSource
source "$HOME/radioconda/bin/activate"
if [ -n "$PYTHONPATH" ]; then
  export PYTHONPATH="$PWD/grc:$PYTHONPATH"
else
  export PYTHONPATH="$PWD/grc"
fi
gnuradio-companion grc/RMUC2026_NanoSDR_TRX_SIC.grc
~~~

上述 export 会把当前仓库的 **grc/** 加入**正在运行 GRC 的 Python**模块搜索路径。它使用 **$PWD**，不依赖用户名或工作空间的绝对位置；无需配置 GRC_BLOCKS_PATH。

打开后先检查流图变量：

| 变量 | 作用 |
| --- | --- |
| **sdr_uri1** | 主收发 NanoSDR 的 IIO URI，例如 ip:192.168.2.1 |
| **sdr_uri2** | 第二 TX NanoSDR 的 IIO URI |
| **record_prefix** | IQ 分段录制目录；默认 records/ |
| **rx_gain** | 手动接收增益；运行时可由 Python 自适应控制覆盖 |
| **tx_attenuation1 / tx_attenuation2** | 两个 TX Sink 的发射衰减 |

保存并运行一次流图，使 GRC 在 **grc/** 目录生成 Python 脚本。当前节点自动查找 **RMUC2026_NanoSDR_TRX_SIC.py**；若 GRC 根据内部 ID 生成文件名，也会查找 **RMUC2026_NanoSDR_TRX.py**。

流图中保留了禁用的文件源和实验对照模块。它们不会随当前主链路运行；若手动启用，须先把其 cfile 路径替换为本机实际文件，不能直接依赖开发机记录路径。

### 4.1 流图介绍

下图为流图中的发射部分，默认状态为disable用于节省系统资源，需要进行loopback测试/tx测试时按情况使能。其中默认使用主发射源，而第二发射源只在Web上使能了对应选项后才会发射信号，且第二发射源固定为对应颜色的信息波。

**注意，要按照实际连接的SDR的ip来进行使能，如果使能了某个发射源而没有插入对应SDR会导致无法运行！**

![](.\picture\TX.png)

下图为流图中的接收部分，默认为开启状态，可在TX模式下失能。其中rotating_file_sink为内录块，会以1分钟为一个切片的方式在项目文件下的records文件夹（该文件夹默认不被git跟踪）内自动创建内录片段，可以按需关闭。

输入的基带信号会被分为两路，一路直接经过数字变频和滤波后进入干扰波通道，另一路则是原始基带信号减去干扰波通道内容之后的部分经过数字变频和滤波然后进入信息波通道。两个通道同时进行数字增益控制、限幅、匹配滤波以及符号同步、二值化。此外，流图会实时监测几个通道的功率，从而修改硬件门控增益，防止出现因ADC饱和导致的无法解调问题，从而适应更大的解调距离范围。实测比赛中可以在距离官方信号源10cm~2m的范围内稳定完成信息波解调。

![](.\picture\RX.png)

受限于SDR硬件性能，该方案只能实现1MHz采样频率，即在复数域上±500KHz总计1MHz的采样带宽，故本工程在1~2级干扰波情况下是不会对信息波进行解析的，仅在三级干扰波情况下会同时解析干扰波秘钥与信息波内容。比赛中这一限制并无实质上的影响，至多于倒计时6:49时升级到三级干扰波等级，并于6:45时开始并永久锁定对方所有地面目标，且信息波解调延迟小于100ms。

### 4.2 运行界面介绍

下面两张图是运行过程中对通道的监测UI，其中包含了原始基带数据的瀑布图与频谱图、鉴频后两个通道的电压波形图与符号同步后的眼图，以及两个通道各自的瀑布图与频谱图，还有最后的功率监测图。

![](.\picture\运行窗口展示1.png)

![](.\picture\运行窗口展示2.png)

## 5. 脚本模式

从仓库根目录运行：

~~~bash
python3 scripts/rmuc_node.py --mode debug
~~~

启动后访问 http://127.0.0.1:53100 。网页用于设置阵营、发送波形、干扰等级、功率、第二发射源和 RX 解调选择，并显示已解出的业务包和运行日志。

### 常用启动方式

#### 纯软件协议回环

~~~bash
python3 scripts/rmuc_node.py --mode debug
~~~

不启动 GNU Radio、不访问 NanoSDR。发送线程将 packed 空口字节写入内存虚拟 SDR，接收线程从中读取并完成空口帧、串口帧和业务包解析。

#### 单 SDR 硬件回环，由脚本守护流图

~~~bash
python3 scripts/rmuc_node.py \
  --mode loopback \
  --flowgraph-python "$HOME/radioconda/bin/python3"
~~~

非 debug 模式默认启用流图守护。守护进程会使用指定 Python 启动生成的流图 Python 脚本；流图退出后每 2 秒重启，并在重启后重新同步 SDR 参数。守护启动流图时会自动补齐 PYTHONPATH。

#### 手动运行流图，再启动软件节点

在第一个终端中按第 4 节设置 PYTHONPATH 并打开/运行 GRC；在第二个终端中执行：

~~~bash
python3 scripts/rmuc_node.py --mode loopback --no-flowgraph-watchdog
~~~

**--no-flowgraph-watchdog** 只是不再由软件节点启动和重启流图，ZMQ 与 XMLRPC 通信仍会照常进行。

#### 独立发射或接收

~~~bash
python3 scripts/rmuc_node.py --mode tx
python3 scripts/rmuc_node.py --mode rx
~~~

**tx** 适用于信息波或干扰波发射机；**rx** 适用于接收解调机。三级干扰波与信息波同时解调时，在网页 RX 区选择对应阵营和 3 级干扰波。
**在启用了radioconda的情况下可以不写--flowgraph-python "$HOME/radioconda/bin/python3"。**

### 脚本参数

以下参数除开**--mode**，均可以在无特殊需求的时候可直接忽略不写。不写--no-flowgraph-watchdog时会自动开启守护进程。

| 参数 | 默认值 | 作用 |
| --- | --- | --- |
| **--mode** | debug | 选择 debug、loopback、tx 或 rx |
| **--rate-hz** | 10 | 软件 TX 生成周期频率，单位 Hz |
| **--port** | 53100 | 网页 GUI 监听端口 |
| **--sdr-tx-port** | 53101 | 主发射源 Python PUB 到 GRC SUB 的端口 |
| **--sdr-tx-second-port** | 53102 | 第二发射源 Python PUB 到 GRC SUB 的端口 |
| **--sdr-rx-info-port** | 53103 | GRC PUB 到 Python SUB 的信息波输出端口 |
| **--sdr-rx-interfere-port** | 53104 | GRC PUB 到 Python SUB 的干扰波输出端口 |
| **--sdr-xmlrpc-port** | 53106 | Python 下发流图变量的 XMLRPC 端口 |
| **--flowgraph-script PATH** | auto | 指定生成的流图 Python 脚本；auto 自动查找主流图 |
| **--flowgraph-python PATH** | $HOME/radioconda/bin/python3 | 指定用于运行 GNU Radio 流图的 Python |
| **--no-flowgraph-watchdog** | 关闭守护 | 不自动启动、监控和重启流图（不写该参数时会开启守护） |

除网页端口和上述 ZMQ/XMLRPC 端口外，脚本不提供 SDR IP 参数；IP 在流图的 **sdr_uri1**、**sdr_uri2** 变量中配置。修改端口时，必须同步修改流图中的对应 ZMQ/XMLRPC Block。

## 6. ROS 2 模式

ROS 2 包名为 **radar_parse_em_wave**。建议把本仓库置于工作空间：

~~~text
~/rmuc_ws/src/radar_parse_em_wave
~~~

首次使用的推荐顺序如下：

1. 按第 4 节设置 PYTHONPATH，打开流图并生成 Python 脚本；
2. 回到 ROS 2 工作空间构建，使 grc/ 一并安装；
3. source ROS 2 与工作空间环境；
4. 使用系统 ROS 2 环境启动节点，并用 --flowgraph-python 指向 GNU Radio 环境。

~~~bash
cd ~/rmuc_ws
colcon build --packages-select radar_parse_em_wave
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run radar_parse_em_wave radar_parse_em_wave_node \
  --mode loopback
~~~

ROS 2 入口同样会启动网页 GUI。需要由外部 launch 或手动 GRC 管理流图时，添加 **--no-flowgraph-watchdog**：

~~~bash
ros2 run radar_parse_em_wave radar_parse_em_wave_node \
  --mode rx --no-flowgraph-watchdog
~~~

ROS 2 入口支持的参数只有 **--mode**、**--flowgraph-python** 与 **--no-flowgraph-watchdog**；端口使用工程默认值。

### ROS 2 话题

| 输入话题 | 类型 | 作用 |
| --- | --- | --- |
| **/rmuc/demod_config** | RadarParseEmWaveDemodConfig | 设置接收阵营和干扰等级 |

~~~bash
ros2 topic pub --once /rmuc/demod_config \
  radar_parse_em_wave/msg/RadarParseEmWaveDemodConfig \
  "{team: blue, interference_level: 2}"
~~~

成功解析并通过 CRC 校验后，节点按包类型发布：

| 话题 | 对应串口包 |
| --- | --- |
| **/rmuc/rx/packet_0x0a01** | 0x0A01 机器人位置 |
| **/rmuc/rx/packet_0x0a02** | 0x0A02 机器人血量 |
| **/rmuc/rx/packet_0x0a03** | 0x0A03 机器人弹量 |
| **/rmuc/rx/packet_0x0a04** | 0x0A04 场地状态 |
| **/rmuc/rx/packet_0x0a05** | 0x0A05 机器人增益与哨兵姿态 |
| **/rmuc/rx/packet_0x0a06** | 0x0A06 干扰波密钥 |

## 7. 运行后的预期效果

| 场景 | 正常现象 |
| --- | --- |
| debug | 网页日志显示虚拟 SDR 链路；TX/RX 计数增长；业务包区域持续更新 |
| loopback | GRC 打开 QT GUI 频谱、瀑布图和解调观察窗口；网页能收到解调后的业务包 |
| tx | 网页 TX 设置会同步到流图，中心频点、衰减、调制门控随设置变化 |
| rx | GUI/ROS 2 修改接收阵营和等级后，流图 RX 中心频点、带宽和信道频率更新；CRC 正确的包会显示/发布 |
| 三级接收 | 干扰波支路保持解调，信息波支路经过 SIC 后仍可输出解调比特 |

网页默认地址为 http://127.0.0.1:53100 。若页面无法打开，先确认软件节点进程仍在运行且端口未被占用；若网页有日志但没有 RX 数据，依次检查 NanoSDR IP、流图是否运行、ZMQ 端口和接收阵营/等级。

如下图为Web的UI显示，左侧可以配置发送设置，右侧可以配置接收配置并查看接收情况。对于发射配置，可以自主修改发送数据内容并对某个包进行开关，关闭的包所占空口包数据位置会被替换为随机字节。

**注意，只有开启第二发射源之后才会向第二发射源中提供数据流，且第二发射源固定为信息波。不开启第二发射源时主发射源也可以选择发送信息波；接收存在软件门控，接收配置中为1级干扰波和2级干扰波时不会处理信息波通道的数据，仅3级干扰波时才会处理信息波**

![](.\picture\WebUI1.png)

## 8. 软件架构

~~~text
网页 GUI ───────┐
ROS 2 话题 ────┼──> RuntimeState ──> SoftwareProtocolNode
               │                         ├─> TxWorker ─> ZMQ ─> GNU Radio TX
               │                         ├─> RxWorker <─ ZMQ <─ GNU Radio RX
               │                         └─> XMLRPC ───> GNU Radio 参数 setter
               │
               └──> 网页状态、日志、RX 业务包显示

GNU Radio 功率遥测 ──> ZMQ ──> SdrParameterController ──> RX gain XMLRPC 更新
~~~

- **coder.py** 处理 CRC、串口包与 27 字节空口帧；空口帧由 8 字节 Access Code、4 字节固定 Header 和 15 字节 Payload 组成。
- **tx.py** 以默认 10 Hz 生成数据；信息波业务包按连续字节流填充，干扰波密钥包在 15 字节 Payload 内改变 SOF 偏移，用于覆盖不同空口对齐情况。
- **rx.py** 从两条解调比特流中重建空口帧、串口帧和 0x0A01 至 0x0A06 业务包。
- **state.py** 将网页或 ROS 2 的选择转换为完整 SDR 参数快照；**sdr_control.py** 通过 XMLRPC 下发，并依据 GRC 功率遥测自适应调整 RX gain。

## 9. GNU Radio 流图处理流程

### TX 数据流

~~~text
网页 / ROS 2 设置
  → RuntimeState 计算频点、衰减与调制门控
  → XMLRPC 写入流图变量

业务包 → 串口帧 → 15 B 空口 Payload → ZMQ 输入
  → 信息波或 1/2/3 级干扰波 GFSK 调制
  → 主 TX 多路门控与叠加 / 第二 TX 信息波支路
  → NanoSDR Sink 发射
~~~

规则参数集中在 **rmuc/constants.py**：采样率为 1 MHz、2-GFSK、BT 为 0.35、Gaussian span 为 4、每符号 47 采样点，码率约为 21.277 kHz。实际发射衰减按经验使用 **tx_attenuation ≈ -tx_power_dbm**，软件允许的功率范围为 -70 dBm 至 -10 dBm。

### RX 数据流

~~~text
NanoSDR Source
  → 原始基带校正与功率观测
  ├─> 干扰波：频移 → 信道低通 → AGC → 四鉴频 → 高斯滤波/符号同步 → 比特切片 → ZMQ
  └─> 信息波：频移与预滤波 → 干扰参考支路的时延/相位估计与 SIC 相减
             → AGC → 四鉴频 → 高斯滤波/符号同步 → 比特切片 → ZMQ

GRC 功率遥测 → Python 自适应 RX gain → XMLRPC 回写流图
~~~

1、2 级主要验证干扰波解调；3 级接收保留干扰波和信息波并行链路，信息波支路利用干扰参考进行 SIC。GRC 禁用块是实验记录的一部分，不应因未参与当前主链路而删除。

## 10. 端口约定与排错

| 端口 | 方向 | 用途 |
| ---: | --- | --- |
| 53100 | 浏览器 → Python | 网页 GUI |
| 53101 | Python PUB → GRC SUB | 主 TX packed bytes |
| 53102 | Python PUB → GRC SUB | 第二 TX packed bytes |
| 53103 | GRC PUB → Python SUB | 信息波解调 bits |
| 53104 | GRC PUB → Python SUB | 干扰波解调 bits |
| 53105 | GRC PUB → Python SUB | 原始/干扰/信息三路功率遥测 |
| 53106 | Python → GRC | XMLRPC 流图变量同步 |

Python 侧默认连接本机 **127.0.0.1**。若更改任何端口，Python 参数与 GRC Block 必须同步修改。

| 现象 | 首先检查 |
| --- | --- |
| GRC 报找不到 block_rotating_file_sink | 是否在启动 GRC 的同一终端执行了 PYTHONPATH export |
| 节点日志提示找不到流图脚本 | 是否已由 GRC 生成 Python 脚本；或手动传入 --flowgraph-script |
| 流图守护跳过启动 | --flowgraph-python 是否指向能导入 gr、iio、zeromq、qtgui 的 Python |
| 连接不到 NanoSDR | Windows 与 WSL 是否都能 ping 192.168.2.1；iio_info 是否成功 |
| 有 GRC 画面但网页不收包 | 检查 53103/53104 端口、RX 阵营/等级和 ZMQ Block 地址 |
| CPU 持续满载 | 纯软件独立流图应有 Throttle；接入硬件 Source/Sink 时不应额外添加 Throttle |

## 11. 使用提示

- IQ 录制文件、运行日志、虚拟环境和 GNU Radio 自动生成的 Python 脚本已由 .gitignore 排除，不应提交到仓库。
- 流图中的本机 cfile 实验路径仅供保留的禁用模块参考；发布或复现时请使用自己的样本文件。
- 修改 GNU Radio 流图后，重新生成 Python 脚本；ROS 2 安装运行时还需要重新执行 colcon build。
- 实际赛场上的信息波功率与干扰波功率差值并没有官方规则描述中那么差距巨大，且笔者在与复旦大学EGA战队进行交流时发现：许多高级算法并不能对解调效果起到正向作用，相反会有负面影响，所以调试过程当中对于流图中的参数和模块的调整需谨慎。

## 12.项目成员及联系方式

文达：3411096740（QQ）

赵崇烨：alan15766027530（微信）

## 13. 效果展示

下图为全国赛蓝方适应性训练内录数据复盘效果：

![](.\picture\场上解调效果.png)

如下图所示，在26赛季全国赛的赛场上，搭载本算法的雷达最高可以达到2070.5s的易伤时长，平均易伤时间为1867.9s，位列全国赛第2（除去表演赛队伍）。除了半决赛打东北大学的第一小局因串口线没插紧导致整局无雷达外，所有对局均实现信息波与干扰波的稳定解析。

![](.\picture\最终成绩.png)

![](.\picture\mvp.png)

## 14. 致谢

感谢深圳大学RobotPilots战队提供的技术指导。
