#include <Arduino.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>

// ============================================================
// ATmega8 + 14.7456 MHz
// UBT-12HC Servo Emulator
//
// 接收：
// FA AF ID CMD PosH PosL SUM ED
// CMD 01：设置目标位置并闭环控制
// CMD 02：以当前位置为目标，闭环保持
// CMD 03：电机失力（PWM=0）
//
// 回复：
// FA AF ID PosH PosL SpeedH SpeedL SUM ED
//
// UART：230400 8N1
//
// RX：USART 接收中断
// PID：1 kHz
// 速度计算：固定 40 ms
//
// ------------------------------------------------------------
// 修改版（ubt12hc_pd）相对 send_rev_230400_ult.ino 的改动：
//
// 1. 控制律（pid_update）
//    原来：pwm = clamp(|err|, 48, 255)，|err| <= 2 停转
//          → 误差 1°~17° 一律固定 48（开关式），关节最快 ~32°/s
//    现在：pwm = KP_PWM * |err| + FF_PWM，上限 255，|err| <= DEADBAND 停转
//          默认 KP_PWM = 16, FF_PWM = 15, DEADBAND = 1
//          → 比例刚度提高 16 倍，误差 ~15 计数（~5°）就能给满 PWM
//    参数来自仿真（rl_control/jduck_tools/screen_fw_law.py 与
//    Mjlab-Velocity-Flat-JDuck 训练，JDUCK_FW=16,15,1）。
//
// 2. 通信超时保护
//    超过 TIMEOUT_10MS × 10 ms（默认 200 ms）没收到发给本 ID 的合法命令
//    → 自动失力，防止主控死机时舵机一直追最后一个目标。0 = 关闭。
//
// 3. 参数运行时可调 + 存 EEPROM（新增 CMD 0x10 / 0x11 / 0x12）
//    KP_PWM / FF_PWM / DEADBAND / TIMEOUT_10MS 可以通过总线改，不用重新烧录。
//    上电时从 EEPROM 读，没存过就用默认值。
//
// 原有 CMD 0x01/0x02/0x03、帧格式、ID、波特率、方向引脚、速度计算全部不变，
// 主控端（jduck_tools/servo_bus.py）不需要改；参数命令见 jduck_tools/servo_params.py。
// ============================================================

#define F_CPU 14745600UL


// ============================================================
// 舵机 ID
// ============================================================

#define SERVO_ID 1


// ============================================================
// 半双工方向控制
//
// 发送：TXEN LOW  + RXEN HIGH
// 接收：TXEN HIGH + RXEN LOW
// ============================================================

#define TXEN_PIN 2
#define RXEN_PIN 3


// ============================================================
// 电位器 ADC
// ============================================================

#define ADC_CHANNEL 2       // A2 / ADC2

#define ADC_MIN 50
#define ADC_MAX 950

#define ADC_COUNTS_PER_REV 1024UL
#define SERVO_RANGE_DEG 360UL


// ============================================================
// 电机驱动
// ============================================================

#define MOTOR_SLEEP_PIN 5
#define MOTOR_DIR_PIN   6
#define MOTOR_PWM_PIN   9

// ATmega8 OC1A (Arduino D9): 8-bit phase-correct PWM, no prescaler.
// 14.7456 MHz / 510 = 28.9 kHz, above the audible range.

#define MAX_output_pwm 255


// ============================================================
// 指令
// ============================================================

#define CMD_POSITION 0x01
#define CMD_HOLD_CURRENT 0x02
#define CMD_TORQUE_OFF 0x03


// ============================================================
// 控制周期
// ============================================================

#define PID_PERIOD_US   1000UL
#define SPEED_PERIOD_US 40000UL


// ============================================================
// PID 参数
// ============================================================

float Kp = 0.5f;
float Ki = 0.0f;
float Kd = 0.0f;


// ============================================================
// PID 限制
// ============================================================

#define PID_OUTPUT_MAX 255
#define INTEGRAL_MAX 500.0f

// 原版：
// Runtime J-Link samples at target 300 settled at position 292-293.
// Tighten the stop window; one ADC count is about 0.35 degrees.
// #define POSITION_DEADBAND 2
// At target 700 the servo repeatedly stopped at 684 with zero speed;
// a minimum PWM of 16 was insufficient at that operating point.
// #define MIN_OUTPUT_PWM 48

// ------------------------------------------------------------
// 修改版控制律：pwm = KP_PWM * |err| + FF_PWM（|err| > DEADBAND 时），上限 255
//
// KP_PWM   比例刚度，PWM / ADC 计数。原版相当于 1。
// FF_PWM   静摩擦前馈：只要误差超出死区就加上，帮电机起步。
//          原版靠"最小 48"硬顶，这里改成和比例项平滑叠加。
// DEADBAND 停转窗口（计数）。1 计数 ≈ 0.35°。
//
// 出厂前先拿一个舵机上台架：
//   - 若目标附近来回抖（嗡嗡响），先把 KP_PWM 降到 12，再不行加大 DEADBAND 到 2
//   - 若停在离目标几个计数处不动，把 FF_PWM 加到 20~25
//
// 这四个参数**运行时可调**（CMD 0x10/0x11/0x12，见下面"参数命令"），
// 上电时从 EEPROM 读；EEPROM 里没有合法数据时用这里的默认值。
// ------------------------------------------------------------
#define DEFAULT_KP_PWM        16
#define DEFAULT_FF_PWM        15
#define DEFAULT_DEADBAND      1
// 通信超时，单位 10 ms：超过这么久没收到发给本 ID 的合法命令就自动失力。0 = 关闭。
// 主控 50 Hz 每 20 ms 发一次，20（= 200 ms）相当于连续丢 10 帧。
#define DEFAULT_TIMEOUT_10MS  20

uint8_t KP_PWM       = DEFAULT_KP_PWM;
uint8_t FF_PWM       = DEFAULT_FF_PWM;
uint8_t DEADBAND     = DEFAULT_DEADBAND;
uint8_t TIMEOUT_10MS = DEFAULT_TIMEOUT_10MS;


// ------------------------------------------------------------
// 参数命令（帧格式不变：FA AF ID CMD PosH PosL SUM ED）
//
// CMD 0x10 设参数：PosH = 参数号，PosL = 值（0~255）。只改 RAM，立即生效。
// CMD 0x11 读参数：PosH = 参数号。回复帧的 Speed 字段 = 参数值（位置照常）。
// CMD 0x12 存 EEPROM：PosH = 0xA5，PosL = 0x5A（防误写）。存当前 4 个参数。
//
// 参数号：0 = KP_PWM，1 = FF_PWM，2 = DEADBAND，3 = TIMEOUT_10MS
// 参数号非法 / 0x12 校验字节不对：照常回复位置，Speed 字段 = -1，什么都不改。
//
// 这三条命令不改变力矩状态（不上力也不卸力），但会刷新通信超时计时。
// ------------------------------------------------------------
#define CMD_SET_PARAM   0x10
#define CMD_GET_PARAM   0x11
#define CMD_SAVE_PARAMS 0x12

#define PARAM_KP       0
#define PARAM_FF       1
#define PARAM_DEADBAND 2
#define PARAM_TIMEOUT  3
#define PARAM_COUNT    4

// EEPROM 布局：[0] = 魔数 0x5D，[1..4] = 4 个参数，[5] = 校验（参数和 ^ 0xFF）
#define EE_ADDR_MAGIC 0
#define EE_ADDR_PARAM 1
#define EE_ADDR_SUM   (EE_ADDR_PARAM + PARAM_COUNT)
#define EE_MAGIC      0x5D


// ============================================================
// 全局变量
// ============================================================

// 当前实际位置
volatile uint16_t current_position = 0;

// 当前目标位置
uint16_t target_position = 0;

// 是否已经收到过目标位置
bool target_received = false;

// Power up without torque/position control. CMD 0x01 or 0x02 enables control;
// CMD 0x03 disables motor drive again.
bool torque_enabled = false;

// Remember the previous operating command so repeated CMD 0x02 packets only
// report position and do not continually move the closed-loop target.
uint8_t previous_command = 0;

// 修改版：最后一次收到发给本 ID 的合法命令的时间（millis），通信超时保护用
unsigned long last_command_ms = 0;

// PID 积分
float integral = 0.0f;

// PID 上一次误差
float previous_error = 0.0f;


// ============================================================
// 速度反馈
// ============================================================

// 最近一次 40 ms 计算得到的速度
int16_t current_speed = 0;

// 上一次速度采样位置
uint16_t last_speed_position = 0;

// 上一次速度计算时间
unsigned long last_speed_time = 0;


// ============================================================
// UART RX 中断协议解析
//
// FA
// AF
// ID
// CMD
// PosH
// PosL
// SUM
// ED
// ============================================================

volatile uint8_t rx_state = 0;

volatile uint8_t rx_id = 0;
volatile uint8_t rx_cmd = 0;
volatile uint8_t rx_posH = 0;
volatile uint8_t rx_posL = 0;
volatile uint8_t rx_checksum = 0;

// 收到完整合法命令后置 1
volatile bool command_ready = false;

// 完整命令暂存区
volatile uint8_t command_id = 0;
volatile uint8_t command_cmd = 0;
volatile uint8_t command_posH = 0;
volatile uint8_t command_posL = 0;


// ============================================================
// UART 错误统计
//
// 方便后面判断 230400 是否还有硬件接收问题
// ============================================================

volatile uint16_t uart_dor_count = 0;
volatile uint16_t uart_fe_count = 0;


// ============================================================
// UART 初始化
//
// F_CPU = 14.7456 MHz
// Baud = 230400
//
// UBRR = 3
//
// 实际波特率误差 = 0%
// ============================================================

void uart_init(void)
{
    // 14.7456 MHz / (16 * 230400) - 1 = 3
    UBRRH = 0;
    UBRRL = 3;

    // 清状态寄存器
    UCSRA = 0;

    // 开启：
    // RX
    // TX
    // RX Complete Interrupt
    UCSRB =
        (1 << RXEN) |
        (1 << TXEN) |
        (1 << RXCIE);

    // 8N1
    UCSRC =
        (1 << URSEL) |
        (1 << UCSZ1) |
        (1 << UCSZ0);
}


// ============================================================
// UART RX 接收中断
//
// 注意：
// ISR 里面绝对不能：
// - adc_read()
// - delay()
// - delayMicroseconds()
// - motor_set()
// - uart_send_byte()
//
// 这里只负责快速拿走 UDR + 协议解析
// ============================================================

ISR(USART_RXC_vect)
{
    // 必须先读取 UCSRA
    uint8_t status = UCSRA;

    // 然后马上读取 UDR
    uint8_t data = UDR;


    // --------------------------------------------------------
    // Data Overrun
    // --------------------------------------------------------

    if (status & (1 << DOR))
    {
        uart_dor_count++;

        // 当前字节可能已经不可靠
        rx_state = 0;

        return;
    }


    // --------------------------------------------------------
    // Frame Error
    // --------------------------------------------------------

    if (status & (1 << FE))
    {
        uart_fe_count++;

        rx_state = 0;

        return;
    }


    // --------------------------------------------------------
    // 协议状态机
    // --------------------------------------------------------

    switch (rx_state)
    {
        // ====================================================
        // 等待 FA
        // ====================================================

        case 0:

            if (data == 0xFA)
            {
                rx_state = 1;
            }

            break;


        // ====================================================
        // 等待 AF
        // ====================================================

        case 1:

            if (data == 0xAF)
            {
                rx_state = 2;
            }
            else if (data == 0xFA)
            {
                // 连续出现 FA
                // 继续等待 AF
                rx_state = 1;
            }
            else
            {
                rx_state = 0;
            }

            break;


        // ====================================================
        // ID
        // ====================================================

        case 2:

            rx_id = data;

            rx_state = 3;

            break;


        // ====================================================
        // CMD
        // ====================================================

        case 3:

            rx_cmd = data;

            rx_state = 4;

            break;


        // ====================================================
        // PosH
        // ====================================================

        case 4:

            rx_posH = data;

            rx_state = 5;

            break;


        // ====================================================
        // PosL
        // ====================================================

        case 5:

            rx_posL = data;

            rx_state = 6;

            break;


        // ====================================================
        // SUM
        // ====================================================

        case 6:
        {
            rx_checksum = data;

            uint8_t calculated =
                (uint8_t)(
                    rx_id +
                    rx_cmd +
                    rx_posH +
                    rx_posL
                );


            if (rx_checksum == calculated)
            {
                rx_state = 7;
            }
            else
            {
                // 校验失败
                rx_state = 0;
            }

            break;
        }


        // ====================================================
        // ED
        // ====================================================

        case 7:

            if (data == 0xED)
            {
                // --------------------------------------------
                // 收到完整合法数据
                //
                // 注意：
                // 这里只复制数据
                // 不执行 process_command()
                // --------------------------------------------

                command_id   = rx_id;
                command_cmd  = rx_cmd;
                command_posH = rx_posH;
                command_posL = rx_posL;

                command_ready = true;
            }

            rx_state = 0;

            break;


        // ====================================================
        // 异常状态
        // ====================================================

        default:

            rx_state = 0;

            break;
    }
}


// ============================================================
// UART 发送 1 字节
// ============================================================

void uart_send_byte(uint8_t data)
{
    while (!(UCSRA & (1 << UDRE)))
    {
    }

    UDR = data;
}


// ============================================================
// 切换到发送模式
// ============================================================

void set_tx_mode()
{
    digitalWrite(TXEN_PIN, LOW);
    digitalWrite(RXEN_PIN, HIGH);
}


// ============================================================
// 切换到接收模式
// ============================================================

void set_rx_mode()
{
    digitalWrite(TXEN_PIN, HIGH);
    digitalWrite(RXEN_PIN, LOW);
}


// ============================================================
// ADC 初始化
//
// AVCC = ADC参考
// ADC2
// ADC时钟 = F_CPU / 128
// ============================================================

void adc_init()
{
    ADMUX =
        (1 << REFS0) |
        ADC_CHANNEL;

    ADCSRA =
        (1 << ADEN) |
        (1 << ADPS2) |
        (1 << ADPS1) |
        (1 << ADPS0);
}


// ============================================================
// ADC 读取
//
// 返回 0~1023
// ============================================================

uint16_t adc_read()
{
    ADCSRA |= (1 << ADSC);

    while (ADCSRA & (1 << ADSC))
    {
    }

    // AVR ADC必须先读ADCL，再读ADCH
    uint8_t low = ADCL;
    uint8_t high = ADCH;

    return ((uint16_t)high << 8) | low;
}


// ============================================================
// 电机停止
// ============================================================

void motor_stop()
{
    analogWrite(
        MOTOR_PWM_PIN,
        0
    );
}


// ============================================================
// 电机控制
//
// pwm > 0：正转
// pwm < 0：反转
// pwm = 0：停止
// ============================================================

void motor_set(int16_t pwm)
{
    // 限制
    if (pwm > 255)
        pwm = 255;

    if (pwm < -255)
        pwm = -255;


    // 正转
    if (pwm > 0)
    {
        digitalWrite(
            MOTOR_DIR_PIN,
            HIGH
        );

        analogWrite(
            MOTOR_PWM_PIN,
            pwm
        );
    }


    // 反转
    else if (pwm < 0)
    {
        digitalWrite(
            MOTOR_DIR_PIN,
            LOW
        );

        analogWrite(
            MOTOR_PWM_PIN,
            -pwm
        );
    }


    // 停止
    else
    {
        analogWrite(
            MOTOR_PWM_PIN,
            0
        );
    }
}


// ============================================================
// PID控制
//
// 固定 1 kHz 调用
// ============================================================

void pid_update(void)
{
    current_position = adc_read();

    // Torque-off mode keeps sampling/reporting position but never drives PWM.
    if (!torque_enabled)
    {
        motor_stop();
        integral = 0;
        return;
    }

    int16_t error =
        (int16_t)target_position -
        (int16_t)current_position;


    // 到达目标位置（死区）
    if (abs(error) <= DEADBAND)
    {
        motor_stop();

        integral = 0;

        previous_error = error;

        return;
    }


    // 修改版：P + 静摩擦前馈。用 int32 计算，KP_PWM 调大也不会溢出
    // （|error| 最大约 900，900 * KP_PWM 在 int16 下 KP_PWM > 36 就会溢出）。
    int32_t pwm32 = (int32_t)KP_PWM * (int32_t)abs(error) + FF_PWM;


    // 限制最大 PWM
    if (pwm32 > MAX_output_pwm)
        pwm32 = MAX_output_pwm;

    int16_t pwm = (int16_t)pwm32;


    // 注意：
    // 这里把原来的方向反过来了
    if (error > 0)
        motor_set(-pwm);
    else
        motor_set(+pwm);


    previous_error = error;
}


// ============================================================
// 40 ms速度计算
//
// speed = ΔADC × 360° × 25
//         ------------------
//             1024 counts
// ============================================================

int16_t calculate_speed_40ms(
    uint16_t current,
    uint16_t previous)
{
    int32_t delta =
        (int32_t)current -
        (int32_t)previous;


    int32_t speed =
        delta *
        (int32_t)SERVO_RANGE_DEG *
        25L;


    speed /=
        (int32_t)ADC_COUNTS_PER_REV;


    // 限制 int16_t
    if (speed > 32767)
        speed = 32767;

    if (speed < -32768)
        speed = -32768;


    return (int16_t)speed;
}


// ============================================================
// 更新40ms速度
// ============================================================

void update_speed()
{
    uint16_t position =
        adc_read();


    current_position =
        position;


    current_speed =
        calculate_speed_40ms(
            position,
            last_speed_position
        );


    last_speed_position =
        position;
}


// ============================================================
// 发送当前位置 + 速度
//
// FA AF ID PosH PosL SpeedH SpeedL SUM ED
// ============================================================

void send_position_speed(
    uint16_t position,
    int16_t speed)
{
    uint8_t posH =
        (position >> 8) & 0xFF;

    uint8_t posL =
        position & 0xFF;


    uint16_t speedValue =
        (uint16_t)speed;


    uint8_t speedH =
        (speedValue >> 8) & 0xFF;

    uint8_t speedL =
        speedValue & 0xFF;


    uint8_t checksum =
        (uint8_t)(
            SERVO_ID +
            posH +
            posL +
            speedH +
            speedL
        );


    // -----------------------------------------
    // TX
    // -----------------------------------------

    set_tx_mode();

    delayMicroseconds(5);


    // -----------------------------------------
    // Frame
    // -----------------------------------------

    uart_send_byte(0xFA);
    uart_send_byte(0xAF);

    uart_send_byte(SERVO_ID);

    uart_send_byte(posH);
    uart_send_byte(posL);

    uart_send_byte(speedH);
    uart_send_byte(speedL);

    uart_send_byte(checksum);

    uart_send_byte(0xED);


    // -----------------------------------------
    // 等待最后一位真正发送完成
    // -----------------------------------------

    while (!(UCSRA & (1 << TXC)))
    {
    }


    // 清除TXC
    UCSRA |= (1 << TXC);


    // -----------------------------------------
    // 立即回RX
    // -----------------------------------------

    set_rx_mode();
}


// ============================================================
// 修改版：运行时参数 + EEPROM
// ============================================================

uint8_t param_get(uint8_t idx)
{
    switch (idx)
    {
        case PARAM_KP:       return KP_PWM;
        case PARAM_FF:       return FF_PWM;
        case PARAM_DEADBAND: return DEADBAND;
        case PARAM_TIMEOUT:  return TIMEOUT_10MS;
        default:             return 0;
    }
}


void param_set(uint8_t idx, uint8_t value)
{
    switch (idx)
    {
        case PARAM_KP:       KP_PWM = value;       break;
        case PARAM_FF:       FF_PWM = value;       break;
        case PARAM_DEADBAND: DEADBAND = value;     break;
        case PARAM_TIMEOUT:  TIMEOUT_10MS = value; break;
        default:                                   break;
    }
}


uint8_t params_checksum(const uint8_t *p)
{
    uint8_t s = 0;
    for (uint8_t i = 0; i < PARAM_COUNT; i++)
        s += p[i];
    return s ^ 0xFF;
}


// 上电时调用。EEPROM 魔数/校验不对，或数值超出合理范围 → 保持默认值。
void params_load()
{
    if (eeprom_read_byte((const uint8_t *)EE_ADDR_MAGIC) != EE_MAGIC)
        return;

    uint8_t p[PARAM_COUNT];
    for (uint8_t i = 0; i < PARAM_COUNT; i++)
        p[i] = eeprom_read_byte((const uint8_t *)(EE_ADDR_PARAM + i));

    if (eeprom_read_byte((const uint8_t *)EE_ADDR_SUM) != params_checksum(p))
        return;

    // 合理范围：KP 1..64，FF 0..120，DEADBAND 0..10，TIMEOUT 任意（0 = 关闭）
    if (p[PARAM_KP] < 1 || p[PARAM_KP] > 64 ||
        p[PARAM_FF] > 120 ||
        p[PARAM_DEADBAND] > 10)
        return;

    for (uint8_t i = 0; i < PARAM_COUNT; i++)
        param_set(i, p[i]);
}


void params_save()
{
    uint8_t p[PARAM_COUNT];
    for (uint8_t i = 0; i < PARAM_COUNT; i++)
        p[i] = param_get(i);

    // eeprom_update_byte 只写变化了的字节（EEPROM 约 10 万次擦写寿命）
    for (uint8_t i = 0; i < PARAM_COUNT; i++)
        eeprom_update_byte((uint8_t *)(EE_ADDR_PARAM + i), p[i]);
    eeprom_update_byte((uint8_t *)EE_ADDR_SUM, params_checksum(p));
    eeprom_update_byte((uint8_t *)EE_ADDR_MAGIC, EE_MAGIC);
}


// ============================================================
// 收到合法指令
//
// 注意：
// 这个函数现在只在 loop() 中执行
// 不在 UART ISR 中执行
// ============================================================

void process_command(
    uint8_t id,
    uint8_t cmd,
    uint8_t posH,
    uint8_t posL)
{
    // -----------------------------------------
    // ID检查
    // -----------------------------------------

    if (id != SERVO_ID)
        return;


    // -----------------------------------------
    if (cmd != CMD_POSITION &&
        cmd != CMD_HOLD_CURRENT &&
        cmd != CMD_TORQUE_OFF &&
        cmd != CMD_SET_PARAM &&
        cmd != CMD_GET_PARAM &&
        cmd != CMD_SAVE_PARAMS)
        return;


    // 修改版：记录最后一次收到合法命令的时间（通信超时保护用）
    last_command_ms = millis();


    // -----------------------------------------
    // 修改版：参数命令。不改力矩状态、不改 previous_command
    // （否则紧跟的 CMD 0x02 会把目标重置成当前位置）。
    // -----------------------------------------

    if (cmd == CMD_SET_PARAM ||
        cmd == CMD_GET_PARAM ||
        cmd == CMD_SAVE_PARAMS)
    {
        int16_t reply = -1;   // -1 = 参数号非法 / 校验字节不对

        if (cmd == CMD_SET_PARAM && posH < PARAM_COUNT)
        {
            param_set(posH, posL);
            reply = param_get(posH);
        }
        else if (cmd == CMD_GET_PARAM && posH < PARAM_COUNT)
        {
            reply = param_get(posH);
        }
        else if (cmd == CMD_SAVE_PARAMS && posH == 0xA5 && posL == 0x5A)
        {
            params_save();
            reply = 0;
        }

        current_position = adc_read();
        send_position_speed(current_position, reply);
        return;
    }


    // -----------------------------------------
    // 测试：收到完整且正确的数据包
    // 输出一个10us脉冲
    // -----------------------------------------

//    digitalWrite(13, HIGH);
//    delayMicroseconds(10);
//    digitalWrite(13, LOW);


    // -----------------------------------------
    // 当前实际位置
    // -----------------------------------------

    uint16_t position =
        adc_read();

    current_position =
        position;


    // -----------------------------------------
    if (cmd == CMD_POSITION)
    {
        uint16_t new_target =
            ((uint16_t)posH << 8) | posL;

        if (new_target < ADC_MIN)
            new_target = ADC_MIN;
        if (new_target > ADC_MAX)
            new_target = ADC_MAX;

        target_position = new_target;
        target_received = true;

        if (!torque_enabled)
        {
            motor_stop();
            digitalWrite(MOTOR_SLEEP_PIN, HIGH);
        }
        torque_enabled = true;
        previous_command = CMD_POSITION;
    }
    else if (cmd == CMD_HOLD_CURRENT)
    {
        // Capture once on entry from CMD 0x01/CMD 0x03. Repeated CMD 0x02
        // packets are polls and must not make the target follow the shaft.
        if (previous_command != CMD_HOLD_CURRENT)
            target_position = position;

        target_received = true;

        if (!torque_enabled)
        {
            motor_stop();
            digitalWrite(MOTOR_SLEEP_PIN, HIGH);
        }
        torque_enabled = true;
        previous_command = CMD_HOLD_CURRENT;
    }
    else // CMD_TORQUE_OFF
    {
        // Disable the bridge output immediately and retain the measured position.
        target_position = position;
        torque_enabled = false;
        motor_stop();
        digitalWrite(MOTOR_SLEEP_PIN, LOW);
        previous_command = CMD_TORQUE_OFF;
    }

    integral = 0.0f;
    previous_error = (float)target_position - (float)current_position;

    // Acknowledge each supported command with the current measured position.
    send_position_speed(current_position, current_speed);
}


// ============================================================
// 处理 UART 中断收到的完整命令
//
// 注意：
// command_ready 是 ISR 和主循环共享变量
// ============================================================

void process_received_command()
{
    if (!command_ready)
        return;


    // 暂时关闭中断
    // 防止读取多字节数据过程中被 ISR 修改
    cli();


    uint8_t id =
        command_id;

    uint8_t cmd =
        command_cmd;

    uint8_t posH =
        command_posH;

    uint8_t posL =
        command_posL;


    command_ready =
        false;


    // 恢复中断
    sei();


    // -----------------------------------------
    // 真正处理命令
    // -----------------------------------------

    process_command(
        id,
        cmd,
        posH,
        posL
    );
}


// ============================================================
// Arduino setup
// ============================================================

void setup()
{
    // -----------------------------------------
    // 半双工方向
    // -----------------------------------------

    pinMode(
        TXEN_PIN,
        OUTPUT
    );

    pinMode(
        RXEN_PIN,
        OUTPUT
    );


    // -----------------------------------------
    // D13 测试输出
    // -----------------------------------------

    pinMode(
        13,
        OUTPUT
    );

    digitalWrite(
        13,
        HIGH
    );


    // -----------------------------------------
    // 电机
    // -----------------------------------------

    pinMode(
        MOTOR_SLEEP_PIN,
        OUTPUT
    );

    // Keep the motor driver disabled throughout startup until CMD 0x01/0x02.
    digitalWrite(
        MOTOR_SLEEP_PIN,
        LOW
    );

    pinMode(
        MOTOR_DIR_PIN,
        OUTPUT
    );

    pinMode(
        MOTOR_PWM_PIN,
        OUTPUT
    );

    // Raise D9 PWM from the Arduino default (~490 Hz) to ~28.9 kHz.
    // Keep analogWrite() usable through OCR1A; phase-correct 8-bit mode.
    TCCR1A = (1 << COM1A1) | (1 << WGM10);
    TCCR1B = (1 << CS10);

    // Ensure the motor output is off before waking the driver.
    motor_stop();


    // -----------------------------------------
    // 默认RX
    // -----------------------------------------

    set_rx_mode();


    // -----------------------------------------
    // ADC2
    // -----------------------------------------

    DDRC &= ~(1 << PC2);


    // -----------------------------------------
    // ADC
    // -----------------------------------------

    adc_init();


    // -----------------------------------------
    // 修改版：从 EEPROM 读控制参数（没有合法数据就用默认值）
    // -----------------------------------------

    params_load();


    // -----------------------------------------
    // UART
    // -----------------------------------------

    uart_init();


    // -----------------------------------------
    // 初始位置
    // -----------------------------------------

    current_position =
        adc_read();


    target_position =
        current_position;

    // Keep the power-up state torque-free until a valid CMD 0x01/0x02 arrives.
    torque_enabled = false;
    motor_stop();


    // -----------------------------------------
    // 速度计算初始值
    // -----------------------------------------

    last_speed_position =
        current_position;


    last_speed_time =
        micros();


    delay(100);


    // -----------------------------------------
    // 开启全局中断
    // -----------------------------------------

    sei();
}


// ============================================================
// 主循环
// ============================================================

void loop()
{
    static unsigned long last_pid_time = 0;
    static unsigned long last_speed_update = 0;


    unsigned long now =
        micros();


    // ========================================================
    // 1. 处理 UART 中断收到的完整命令
    //
    // UART 接收本身已经完全由 ISR 完成
    // 这里不再轮询 RXC
    // ========================================================

    process_received_command();


    // ========================================================
    // 1b. 修改版：通信超时 → 失力（与 CMD 0x03 完全相同的处理）
    //
    // 只在已上力时检查；unsigned 减法跨 millis() 溢出也正确。
    // ========================================================

    if (TIMEOUT_10MS > 0 && torque_enabled &&
        (unsigned long)(millis() - last_command_ms) > (unsigned long)TIMEOUT_10MS * 10UL)
    {
        target_position = adc_read();
        torque_enabled = false;
        motor_stop();
        digitalWrite(MOTOR_SLEEP_PIN, LOW);
        previous_command = CMD_TORQUE_OFF;
    }


    // ========================================================
    // 2. PID：固定1kHz
    // ========================================================

    if ((unsigned long)(
            now - last_pid_time) >= PID_PERIOD_US)
    {
        last_pid_time +=
            PID_PERIOD_US;

        pid_update();
    }


    // ========================================================
    // 3. 速度：固定40ms
    // ========================================================

    if ((unsigned long)(
            now - last_speed_update) >= SPEED_PERIOD_US)
    {
        last_speed_update +=
            SPEED_PERIOD_US;

        update_speed();
    }
}
