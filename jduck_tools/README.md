# jduck_tools — 舵机通信与关节标定

UBT-12HC 总线舵机（协议见 `~/Downloads/UBT12HC_总线舵机通讯协议说明.pdf`）的通信和标定工具。
用**系统 python3** 运行（已带 pyserial、pyyaml），不需要 ROS、不需要 uv 环境。

| 文件 | 作用 |
|---|---|
| `servo_bus.py` | 协议编解码 + 半双工总线驱动（之后部署控制循环也复用它） |
| `calibration.py` | ADC ↔ URDF 关节角换算、`calibration.yaml` 读写、把范围写回 URDF |
| `calibrate.py` | 命令行标定工具 |
| `servo_map.yaml` | **关节名 → 舵机 ID**（先按你的实际接法改好） |
| `calibration.yaml` | 标定结果（标定时自动生成） |
| `test_tools.py` | 离线测试（不需要硬件）：`python3 -m pytest jduck_tools/test_tools.py -q` |

> ⚠ 这套工具还**没在真实硬件上跑过**，只用模拟舵机测过协议和计算。第一次用时先接一个舵机试 `scan`。

## 换算关系

```
q(rad, URDF) = sign × (adc − adc_zero) × k
```
- `adc_zero`：关节在 URDF 0 位时的 ADC 读数
- `sign`：往 URDF 正方向转时 ADC 增大为 +1，减小为 −1
- `k`：每个 ADC 计数多少弧度（规格书标称 2π/1024 ≈ 0.00614，要实测）

## 标定流程（在 `rl_control/` 下运行）

### 0. 准备
- 机器人吊起来或放平，舵机供电，USB 转 TTL 插电脑。
- `ls /dev/ttyUSB* /dev/ttyACM*` 找到串口名，下面以 `/dev/ttyUSB0` 为例。
- 先在 `python3 view_urdf.py jduck_description/urdf/jduck_description.urdf`（或 RViz）里看好
  **每个关节 0 位的样子和正方向**——截图留着对照。

### 1. 扫描总线
```bash
python3 jduck_tools/calibrate.py --port /dev/ttyUSB0 scan
```
应该看到 14 个 ID 都应答。没有应答时依次检查：供电、接线、`--direction`（模块不能自动切收发
时试 `--direction rts` 或 `rts-inv`）、`--timeout 0.05`。

### 2. 确认 ID ↔ 关节
```bash
python3 jduck_tools/calibrate.py --port /dev/ttyUSB0 monitor
```
全部卸力，用手逐个掰关节，看哪一行在变，把对应关系改进 `servo_map.yaml`。

### 3. 测 k（做一次）
```bash
python3 jduck_tools/calibrate.py --port /dev/ttyUSB0 measure-k --id 3 --angle-deg 90
```
摆两个相差已知角度（默认 90°）的姿态各按一次空格。结果接近 0.35°/计数说明是 1024 计数/360°。

### 4. 逐个关节标定
```bash
python3 jduck_tools/calibrate.py --port /dev/ttyUSB0 joint              # 全部
python3 jduck_tools/calibrate.py --port /dev/ttyUSB0 joint left_knee    # 单个
```
每个关节三步，每步按空格确认：
1. 摆到 **URDF 0 位** → 记录零位
2. 往 **URDF 正方向** 转 20° 以上 → 判断方向
3. **慢慢扫过两端**（碰到机械限位或别的零件就停），回中间 → 记录范围

每标完一个关节就自动保存。`s` 跳过当前关节，`q` 退出。

### 5. 检查结果
```bash
python3 jduck_tools/calibrate.py report
```
注意 ⚠ 提示：
- **ADC 范围贴近 50/950**：固件会把目标钳在 50–950，把舵盘拆下转一个齿重装，再标这个关节
- **扫动时有跳变**：可能经过电位器盲区，同样挪齿
- **零位不在可用范围内**：零位或方向标错了

### 6. 上电验证方向（机器人吊起来！）
```bash
python3 jduck_tools/calibrate.py --port /dev/ttyUSB0 verify left_knee
```
只给这一个舵机上电，在当前位置附近 ±5° 动一下。确认往 URDF 正方向走，否则重标这个关节。
建议 14 个都过一遍。

### 7. 写回 URDF，重新生成模型
```bash
python3 jduck_tools/calibrate.py apply        # 原 URDF 备份为 *.urdf.precal.bak
cd third_party/microduck_rl && uv run --frozen python ../../jduck_mjcf/build_mjcf.py
```
然后需要重找 HOME 站姿（`jduck_constants.py`）并重新训练。

**拆装过舵盘的关节必须重新标定。**
