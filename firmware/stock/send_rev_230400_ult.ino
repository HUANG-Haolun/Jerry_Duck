#include <Arduino.h>
#include <avr/interrupt.h>

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

// Runtime J-Link samples at target 300 settled at position 292-293.
// Tighten the stop window; one ADC count is about 0.35 degrees.
#define POSITION_DEADBAND 2
// At target 700 the servo repeatedly stopped at 684 with zero speed;
// a minimum PWM of 16 was insufficient at that operating point.
#define MIN_OUTPUT_PWM 48


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


    // 到达目标位置
    if (abs(error) <= POSITION_DEADBAND)
    {
        motor_stop();

        integral = 0;

        previous_error = error;

        return;
    }


    // P 控制。接近目标时维持最低驱动输出，克服电机静摩擦。
    int16_t pwm = abs(error);


    // 限制最大 PWM
    if (pwm > MAX_output_pwm)
        pwm = MAX_output_pwm;

    if (pwm < MIN_OUTPUT_PWM)
        pwm = MIN_OUTPUT_PWM;


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
        cmd != CMD_TORQUE_OFF)
        return;


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
