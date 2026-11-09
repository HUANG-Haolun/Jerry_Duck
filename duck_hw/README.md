# duck_hw 硬件测试程序

本目录包含 13 个独立的 C 程序，用于在 Linux 开发板上测试 ADS1115、GPIO、ICM42688P 和 UBT-12HC 舵机。每个 `.c` 文件单独编译成一个可执行程序，不应将所有源文件链接为一个程序。

## 编译与运行环境

以下命令在目标 Linux 开发板的本目录中执行，使用 Bash 等 Linux Shell。代码依赖 Linux 的 I2C、SPI、GPIO 或串口接口，不能直接在 Windows PowerShell 中编译运行。

Debian/Ubuntu 系统可安装：

```bash
sudo apt update
sudo apt install build-essential linux-libc-dev libgpiod-dev
```

`gpio_test`、`servo_test`、`servo_read` 使用 **libgpiod 1.x API**（如 `gpiod_chip_get_line`）。如果系统只有 libgpiod 2.x，需要准备兼容的 1.x 开发环境，或另行移植源码。

运行前需启用开发板对应接口，并确认设备节点及接线：

| 接口 | 源码使用的设备 | 用途 |
| --- | --- | --- |
| I2C | `/dev/i2c-2`，地址 `0x48` | ADS1115 |
| SPI | `/dev/spidev3.0` | ICM42688P |
| GPIO | `/dev/gpiochip3` | GPIO 输出、部分舵机程序的收发使能 |
| UART | `/dev/ttyS3` | 舵机通信，8N1 |

下文用 `sudo` 运行以取得设备访问权限；已有权限时可省略。持续运行的程序可按 `Ctrl+C` 结束。同一设备的测试程序应逐个运行，避免争用接口。舵机动作程序会实际驱动舵机，运行前需确认供电、接线及机械活动空间。

## 一次编译全部程序

```bash
gcc -std=gnu11 -O2 -Wall -Wextra ads1115_read.c -o ads1115_read
gcc -std=gnu11 -O2 -Wall -Wextra ads1115_bat.c -o ads1115_bat
gcc -std=gnu11 -O2 -Wall -Wextra gpio_test.c -o gpio_test -lgpiod
gcc -std=gnu11 -O2 -Wall -Wextra icm_test.c -o icm_test
gcc -std=gnu11 -O2 -Wall -Wextra icm42688_test.c -o icm42688_test -lm
gcc -std=gnu11 -O2 -Wall -Wextra icm42688_diag.c -o icm42688_diag -lm
gcc -std=gnu11 -O2 -Wall -Wextra servo_test.c -o servo_test -lgpiod
gcc -std=gnu11 -O2 -Wall -Wextra servo_read.c -o servo_read -lgpiod
gcc -std=gnu11 -O2 -Wall -Wextra servo_send_read.c -o servo_send_read
gcc -std=gnu11 -O2 -Wall -Wextra servo_sweep_test.c -o servo_sweep_test
gcc -std=gnu11 -O2 -Wall -Wextra servo_position_stability.c -o servo_position_stability
gcc -std=gnu11 -O2 -Wall -Wextra servo_multi_read.c -o servo_multi_read
gcc -std=gnu11 -O2 -Wall -Wextra servo_position_limits.c -o servo_position_limits
```

其中 `-lgpiod` 用于链接 GPIO 库，`-lm` 用于数学库（诊断程序使用 `sqrtf`）。旧版 Linux 工具链若提示 `clock_gettime` 或 `clock_nanosleep` 未定义，可在相关舵机程序的编译命令末尾加 `-lrt`。

## 各程序功能与运行命令

### 1. ads1115_read.c：读取四路 ADC 原始值

通过 `I2C_RDWR` 依次对 AIN0～AIN3 进行单端单次转换，配置量程为 ±4.096 V、采样率为 128 SPS，轮询转换完成状态。输出各通道的有符号十进制原始值及十六进制值；结束时尝试恢复原配置寄存器。每个通道读取一次后退出，不直接换算电压。

```bash
sudo ./ads1115_read
# 可选参数：I2C 设备路径
sudo ./ads1115_read /dev/i2c-2
```

### 2. ads1115_bat.c：电池电压与电量估算

固定读取 `/dev/i2c-2` 上地址 `0x48` 的 ADS1115 AIN0，约每秒输出 ADC 值、AIN0 电压、电池电压和估算电量。换算方式：

- `AIN0电压 = ADC × 4.096 / 32768`。
- 按上端 22 kΩ、下端 10 kΩ 分压电阻，`电池电压 = AIN0电压 × 3.2`。
- 按 `BAT_EMPTY = 3.5 V`、`BAT_FULL = 8.4 V` 线性估算百分比，并限制在 0%～100%。这是源码中的电压估算法，并非电池荷电状态模型。

```bash
sudo ./ads1115_bat
```

### 3. gpio_test.c：GPIO 高低电平翻转

将 `/dev/gpiochip3` 的 **line 8** 设为输出，初始为低电平，然后高、低电平各保持 1 秒，持续循环并打印状态。

```bash
sudo ./gpio_test
```

注意：源码注释及输出文字写的是 `GPIO3_A7`，但实际调用为 `gpiod_chip_get_line(chip, 8)`，请按实际 line 8 核对接线。

### 4. icm_test.c：SPI 最小通信测试

打开 `/dev/spidev3.0`，使用 SPI mode 0、8 位数据、100 kHz，发送 `F5 00`，即读取寄存器 `0x75`（WHO_AM_I），输出收到的两个字节后退出。程序本身不判断芯片 ID 是否正确；ICM42688P 的预期 ID 为 `0x47`，通常查看第二个接收字节。

```bash
sudo ./icm_test
```

### 5. icm42688_test.c：IMU 连续数据读取

使用 `/dev/spidev3.0`，SPI mode 0、8 位、1 MHz。检查 WHO_AM_I 是否为 `0x47`，执行软件复位，配置陀螺仪、加速度计并启用低噪声模式。连续输出三轴加速度、三轴角速度及温度，每轮休眠 10 ms。

```bash
sudo ./icm42688_test
```

### 6. icm42688_diag.c：IMU 详细诊断

接口参数与 `icm42688_test` 相同。增加复位前后芯片 ID 检查和配置寄存器回读；持续输出加速度、陀螺仪的原始值及换算值、加速度向量模长和温度，每轮休眠 10 ms，便于检查通信和数据状态。

```bash
sudo ./icm42688_diag
```

上述两个 IMU 程序实际向 `GYRO_CONFIG0` 和 `ACCEL_CONFIG0` 写入 `0x27`，按 `8192 LSB/g`、`65.5 LSB/(°/s)` 换算，温度按 `raw / 132.48 + 25` 换算。源码将量程注释为 ±4 g 和 ±500 °/s；使用物理量前应依据芯片数据手册核对寄存器位及换算系数是否匹配。10 ms 是循环休眠时间，实际输出频率还受 SPI 传输和打印耗时影响。

### 7. servo_test.c：GPIO 控制收发方向的舵机动作测试

使用 `/dev/ttyS3`、115200 波特率、舵机 ID 6，通过 `/dev/gpiochip3` 的 line 7（TXEN）、line 8（RXEN）切换收发方向。发送一条 **10 字节**动作命令，实际参数为 `position = 150`、`time = 50`；源码将时间 50 注释为约 1 秒。发送完成后切换接收模式，随后持续休眠，不解析反馈。

```bash
sudo ./servo_test
```

### 8. servo_read.c：读取单个舵机角度

使用 `/dev/ttyS3`、115200 波特率、舵机 ID 6，以及 GPIO line 7/8 控制收发方向。发送 **10 字节** `CMD 0x02` 查询帧，接收并校验回复，打印状态、目标角度和实际角度，完成后退出。

```bash
sudo ./servo_read
```

查询帧校验字节在源码中固定为 `0x08`，与当前 `ID 6 + CMD 2` 对应；修改 ID 时也需要同步修改该字节。

### 9. servo_send_read.c：先移动，再读取位置与速度

直接使用 `/dev/ttyS3`，通过 Linux `termios2/BOTHER` 设置 **230460** 波特率，无 GPIO 收发方向控制。实际舵机 ID 为 **15**，发送 **8 字节** `CMD 0x01` 动作帧，目标位置为 **550**；等待约 1.1 秒后发送 `CMD 0x02` 查询，解析 **9 字节**回复中的无符号位置和有符号速度，打印收发数据及结果后退出。

```bash
sudo ./servo_send_read
```

这里的 ID 15、位置 550 以宏和函数实参为准；代码中仍有 ID 6、位置 120 的旧注释。位置和速度为协议原始值，代码没有换算为角度或转速。

### 10. servo_sweep_test.c：舵机往复运动

使用 `/dev/ttyS3`、230460 波特率、舵机 ID 6，发送 8 字节动作帧。先发送位置 100，再在 **100 与 900** 之间往复，每 20 ms 更新一次目标位置，每个方向 125 步、约 2.5 秒，一个往返周期约 5 秒。不读取反馈。

```bash
# 持续往复，Ctrl+C 停止
sudo ./servo_sweep_test
# 可选参数：正整数往返次数，例如 3 次
sudo ./servo_sweep_test 3
```

这些时间描述的是目标位置指令的发送节奏，实际机械运动取决于舵机响应。结束程序只停止发指令，代码未发送卸力命令。

### 11. servo_position_stability.c：单舵机高频查询与帧完整性检查

使用 `/dev/ttyS3`、230460 波特率，针对 ID 6 以目标 **750 次/秒**发送 8 字节 `CMD 0x03` 查询，不发送动作命令。解析 9 字节位置/速度回复，统计有效回复数、未匹配请求数、回显、校验错误、其他帧、噪声、发送延迟以及位置和速度范围，最后输出帧完整性 PASS/FAIL。

```bash
# 默认测试 10 秒
sudo ./servo_position_stability
# 可选时长：1～60 秒
sudo ./servo_position_stability 20
# 仅发送一次查询，并打印接收到的原始字节
sudo ./servo_position_stability --probe
```

PASS/FAIL 主要依据请求与有效回复数量、帧错误和残留数据，不代表机械位置稳定性通过某个阈值；位置波动需查看输出范围。

### 12. servo_multi_read.c：多舵机位置与速度轮询

使用 `/dev/ttyS3`、230460 波特率，以 `CMD 0x03` 轮询 **ID 1～5、7～16，共 15 个舵机，不含 ID 6**。目标为每秒 50 轮，即每个 ID 每秒查询 50 次、合计每秒 750 条查询。约每秒显示各舵机最近位置、速度、回复和发送计数，结束时汇总帧错误与实际发送速率。不发送动作命令。

```bash
# 默认持续运行，Ctrl+C 停止
sudo ./servo_multi_read
# 可选时长：1～3600 秒
sudo ./servo_multi_read 30
```

位置与速度均为协议原始值；速度按有符号 16 位整数解析。指定时长运行且完全未收到目标 ID 的有效回复时，程序返回失败。

### 13. servo_position_limits.c：记录多舵机位置最小值和最大值

串口、波特率、ID 列表及目标轮询频率与 `servo_multi_read` 相同，发送 `CMD 0x03` 查询。约每秒显示各 ID 的当前位置、运行以来的最小位置、最大位置、新增及累计回复数，退出时打印最终统计。

```bash
# 默认持续运行，Ctrl+C 停止
sudo ./servo_position_limits
# 可选时长：1～3600 秒
sudo ./servo_position_limits 30
```

该程序只记录收到的位置范围，**不会主动驱动舵机寻找机械极限，也不会设置限位**。没有有效回复的 ID 显示 `--`，位置单位为协议原始值。

## 参数修改与排查

- 除 `ads1115_read` 可指定 I2C 路径外，设备路径、地址、波特率、ID 和目标位置通常写在源码宏、数组或函数实参中，修改后需重新编译。
- 新版舵机程序使用的波特率是 **230460**，不是 230400；串口驱动需支持 `termios2/BOTHER` 自定义波特率。
- `servo_test`、`servo_read` 使用 115200 波特率和 10 字节命令；其余舵机程序使用 230460 波特率和 8 字节命令。需根据舵机固件及收发电路选择程序。
- 遇到 `No such file or directory`，检查设备节点、设备树和驱动；遇到 `Permission denied`，检查设备访问权限。
- 舵机无反馈时，核对 ID、波特率、协议、供电、共地和收发方向。目标查询频率是软件调度目标，实际速率应查看统计输出。

本文依据当前源码整理；尚未在目标 Linux 开发板上实际编译或执行硬件测试。
