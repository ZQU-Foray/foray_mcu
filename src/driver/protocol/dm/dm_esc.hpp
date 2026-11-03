#ifndef DRIVER_PROTOCOL_DM_DM_ESC_HPP
#define DRIVER_PROTOCOL_DM_DM_ESC_HPP

#include <array>
#include <cstddef>
#include <cstdint>

namespace protocol
{

/**
 * @brief 达妙（DAMIAO）一体化电机的 CAN 协议编解码（依据《DM-J4310-2EC V1.2 减速电机使用说明书》V1.4，2026-09-14）。
 *        只做字节与物理量的换算，不碰硬件、不保存状态。
 *
 * 帧的种类（全部是标准帧）：
 * - 控制帧：帧 ID = 控制模式的偏移 + ESC_ID（MIT 0、位置速度 0x100、速度 0x200、力位混控 0x300），数据格式随模式不同；
 * - 系统命令（使能、失能、保存零点、清除错误）：也是控制帧，帧 ID 跟随电机当前的控制模式；
 * - 反馈帧：电机每收到一帧 ID 低 8 位与 ESC_ID 相同的帧就回一帧，帧 ID 为 MST_ID，格式与模式无关；
 * - 寄存器读写：请求发到 0x7FF，应答也发到 MST_ID。MST_ID 默认为 0，应答与反馈可能共用同一个帧 ID，
 *   要按内容区分（decode_register_reply() 核对 ESC_ID 与操作码，驱动再核对寄存器号）。
 *
 * 位置、速度、扭矩按量程线性映射到 16/12/12 位无符号数。量程（PMAX、VMAX、TMAX）是电机里的寄存器，
 * 不同型号默认值不同，也能被改；编解码必须用电机里的值（Limits），否则控制量会被等比例缩放。
 */
class DmEsc final
{
  public:
    static constexpr std::size_t FRAME_DATA_LENGTH{8U};
    static constexpr std::uint32_t REGISTER_IDENTIFIER{0x7FFU}; // 寄存器读写、存储的请求帧 ID
    static constexpr std::uint16_t MAX_ESC_ID{0xFFU}; // 控制帧 ID = 偏移 + ESC_ID，ESC_ID 超过 0xFF 会与别的模式重叠
    static constexpr float MIT_KP_MAX{500.0F};                       // MIT 模式 Kp 的映射范围 [0, 500]
    static constexpr float MIT_KD_MAX{5.0F};                         // MIT 模式 Kd 的映射范围 [0, 5]
    static constexpr std::uint16_t FORCE_POSITION_SCALE_MAX{10000U}; // 力位混控的限速（×100）与电流标幺（×10000）上限

    /** 控制模式，取值即 CTRL_MODE 寄存器（0x0A）的编码。 */
    enum class ControlMode : std::uint8_t
    {
        MIT = 1U,
        POSITION_VELOCITY = 2U, // 位置速度（三环串级，v 为限速）
        VELOCITY = 3U,          // 速度
        FORCE_POSITION = 4U     // 力位混控（位置速度 + 电流限幅）
    };

    /** 反馈帧 D[0] 高 4 位的状态码。 */
    enum class State : std::uint8_t
    {
        DISABLED = 0x0U, // 失能（上电默认）
        ENABLED = 0x1U,
        OUTPUT_ENCODER_CALIBRATION_ERROR = 0x3U,
        SENSOR_OUTPUT_ERROR = 0x4U,
        MOTOR_ENCODER_CALIBRATION_ERROR = 0x5U,
        OVER_VOLTAGE = 0x8U,
        UNDER_VOLTAGE = 0x9U,
        OVER_CURRENT = 0xAU,
        MOS_OVER_TEMPERATURE = 0xBU,
        COIL_OVER_TEMPERATURE = 0xCU,
        COMMUNICATION_LOST = 0xDU,
        OVERLOAD = 0xEU
    };

    /** 系统命令：数据为 7 个 0xFF 加命令码。 */
    enum class SystemCommand : std::uint8_t
    {
        CLEAR_ERROR = 0xFBU,
        ENABLE = 0xFCU,
        DISABLE = 0xFDU,
        SAVE_ZERO = 0xFEU // 把当前输出轴位置设为零点
    };

    /** 寄存器地址（说明书"寄存器列表及范围"）。RO 只读，其余可读写；写入立即生效，掉电丢失，存储见 encode_save_registers()。 */
    enum class Register : std::uint8_t
    {
        UNDER_VOLTAGE = 0x00U,          // 低压保护值，float
        TORQUE_CONSTANT = 0x01U,        // 扭矩系数，float
        OVER_TEMPERATURE = 0x02U,       // 过温保护值，float
        OVER_CURRENT = 0x03U,           // 过流保护值（0–1），float
        ACCELERATION = 0x04U,           // float
        DECELERATION = 0x05U,           // float
        MAX_SPEED = 0x06U,              // float
        MASTER_ID = 0x07U,              // 反馈 ID（MST_ID），uint32
        ESC_ID = 0x08U,                 // 接收 ID，uint32
        TIMEOUT = 0x09U,                // 通讯超时报警时间，uint32
        CONTROL_MODE = 0x0AU,           // 控制模式（ControlMode），uint32
        DAMPING = 0x0BU,                // RO，粘滞系数，float
        INERTIA = 0x0CU,                // RO，转动惯量，float
        HARDWARE_VERSION = 0x0DU,       // RO，保留，uint32
        SOFTWARE_VERSION = 0x0EU,       // RO，uint32（实测为 4 个 ASCII 字符）
        SERIAL_NUMBER = 0x0FU,          // RO，保留，uint32
        POLE_PAIRS = 0x10U,             // RO，uint32
        PHASE_RESISTANCE = 0x11U,       // RO，float
        PHASE_INDUCTANCE = 0x12U,       // RO，float
        FLUX = 0x13U,                   // RO，float
        GEAR_RATIO = 0x14U,             // RO，float
        POSITION_MAX = 0x15U,           // PMAX，位置映射范围，float
        VELOCITY_MAX = 0x16U,           // VMAX，速度映射范围，float
        TORQUE_MAX = 0x17U,             // TMAX，扭矩映射范围，float
        CURRENT_BANDWIDTH = 0x18U,      // float
        SPEED_KP = 0x19U,               // float
        SPEED_KI = 0x1AU,               // float
        POSITION_KP = 0x1BU,            // float
        POSITION_KI = 0x1CU,            // float
        OVER_VOLTAGE = 0x1DU,           // float
        GEAR_EFFICIENCY = 0x1EU,        // float
        SPEED_DAMPING = 0x1FU,          // float
        SPEED_FILTER_BANDWIDTH = 0x20U, // float
        CURRENT_ENHANCEMENT = 0x21U,    // float
        SPEED_ENHANCEMENT = 0x22U,      // float
        CAN_BAUD_RATE = 0x23U,          // 波特率代码（4 = 1 Mbps），uint32
        SUB_VERSION = 0x24U,            // RO，uint32
        BOOT_VERSION = 0x25U,           // RO，uint32
        DIRECTION = 0x37U,              // RO，float
        MOTOR_OFFSET = 0x38U,           // RO，float
        DRIVER_MAX_CURRENT = 0x3BU,     // RO，驱动板最大电流，float
        BUS_VOLTAGE = 0x3CU,            // RO，电源电压，float
        DRIVER_TEMPERATURE = 0x3DU,     // RO，float
        COIL_TEMPERATURE = 0x3EU,       // RO，float
        MOTOR_POSITION = 0x50U,         // RO，转子折算到输出轴的位置，float
        OUTPUT_POSITION = 0x51U         // RO，输出轴编码器的位置，float
    };

    /** 寄存器操作码（请求与应答的 D[2]）。 */
    enum class RegisterOperation : std::uint8_t
    {
        READ = 0x33U,
        WRITE = 0x55U,
        SAVE = 0xAAU
    };

    /** 位置、速度、扭矩的映射量程，即电机里的 PMAX、VMAX、TMAX（对称：−max…+max）。 */
    struct Limits final
    {
        float position_rad;
        float velocity_rad_per_second;
        float torque_newton_meter;
    };

    struct Frame final
    {
        std::uint32_t identifier{0U};
        std::uint8_t length{0U};
        std::array<std::uint8_t, FRAME_DATA_LENGTH> data{};
    };

    /** MIT 模式的给定。kp = kd = 0 时就是纯力矩控制；控制位置时 kd 不能为 0（说明书：否则会振荡甚至失控）。 */
    struct MitCommand final
    {
        float position_rad{0.0F};
        float velocity_rad_per_second{0.0F};
        float kp{0.0F};
        float kd{0.0F};
        float torque_newton_meter{0.0F};
    };

    /** 反馈帧的原始字段。 */
    struct FeedbackRaw final
    {
        std::uint8_t id_low_bits{0U}; // D[0] 低 4 位：ESC_ID 的低 4 位
        State state{State::DISABLED}; // D[0] 高 4 位
        std::uint16_t position{0U};   // 16 位
        std::uint16_t velocity{0U};   // 12 位
        std::uint16_t torque{0U};     // 12 位
        std::uint8_t mos_temperature_celsius{0U};
        std::uint8_t rotor_temperature_celsius{0U};
    };

    /** 换算成物理量的反馈（输出轴，单位 rad、rad/s、N·m）。 */
    struct Feedback final
    {
        std::uint8_t id_low_bits{0U};
        State state{State::DISABLED};
        float position_rad{0.0F};
        float velocity_rad_per_second{0.0F};
        float torque_newton_meter{0.0F};
        std::uint8_t mos_temperature_celsius{0U};
        std::uint8_t rotor_temperature_celsius{0U};
    };

    /** 寄存器读、写、存储的应答。 */
    struct RegisterReply final
    {
        std::uint16_t esc_id{0U};
        RegisterOperation operation{RegisterOperation::READ};
        std::uint8_t register_address{0U}; // 读写时为寄存器地址，存储时为 0x01
        std::uint32_t value{0U};           // 读写的 4 字节数据（小端）；存储没有数据
    };

    // 线性映射（与说明书的 float_to_uint / uint_to_float 相同；超出量程先钳位）

    [[nodiscard]] static std::uint16_t
    to_unsigned(float value, float minimum, float maximum, std::uint8_t bits) noexcept;
    [[nodiscard]] static float
    from_unsigned(std::uint16_t value, float minimum, float maximum, std::uint8_t bits) noexcept;

    /** 量程有效：三个值都是有限的正数。 */
    [[nodiscard]] static bool is_valid(const Limits &limits) noexcept;

    /** 控制帧（含系统命令）的帧 ID；ESC_ID 越界时返回 0。 */
    [[nodiscard]] static std::uint32_t control_identifier(std::uint16_t esc_id, ControlMode mode) noexcept;

    // 编码：参数无效（ESC_ID 越界、非有限值、量程无效）时返回 false，frame 不变

    /** MIT：各量超出映射范围时钳位。 */
    [[nodiscard]] static bool
    encode_mit(std::uint16_t esc_id, const MitCommand &command, const Limits &limits, Frame &frame) noexcept;

    /** 位置速度：p 为目标位置，v 为运动中的最大速度（绝对值）。 */
    [[nodiscard]] static bool encode_position_velocity(std::uint16_t esc_id,
                                                       float position_rad,
                                                       float velocity_rad_per_second,
                                                       Frame &frame) noexcept;

    [[nodiscard]] static bool
    encode_velocity(std::uint16_t esc_id, float velocity_rad_per_second, Frame &frame) noexcept;

    /**
     * @brief 力位混控。
     * @param velocity_limit_rad_per_second 限速，0–100 rad/s（编码为 ×100，超出钳位）
     * @param current_limit_per_unit 电流限幅标幺值（实际电流 / 驱动最大相电流），0–1（编码为 ×10000，超出钳位）
     */
    [[nodiscard]] static bool encode_force_position(std::uint16_t esc_id,
                                                    float position_rad,
                                                    float velocity_limit_rad_per_second,
                                                    float current_limit_per_unit,
                                                    Frame &frame) noexcept;

    /** 系统命令：帧 ID 跟随电机当前的控制模式。 */
    [[nodiscard]] static bool
    encode_system(std::uint16_t esc_id, ControlMode mode, SystemCommand command, Frame &frame) noexcept;

    [[nodiscard]] static bool encode_read_register(std::uint16_t esc_id, Register address, Frame &frame) noexcept;

    [[nodiscard]] static bool
    encode_write_register(std::uint16_t esc_id, Register address, std::uint32_t value, Frame &frame) noexcept;

    [[nodiscard]] static bool
    encode_write_register_float(std::uint16_t esc_id, Register address, float value, Frame &frame) noexcept;

    /** 把全部参数写进电机的 flash：只在失能时生效，最长 30 ms，擦写寿命约 1 万次，不要频繁发送。 */
    [[nodiscard]] static bool encode_save_registers(std::uint16_t esc_id, Frame &frame) noexcept;

    // 解码

    /** 反馈帧（8 字节）。不核对帧 ID：反馈与寄存器应答可能共用 MST_ID，由调用方先排除应答。 */
    [[nodiscard]] static bool decode_feedback(const Frame &frame, FeedbackRaw &feedback) noexcept;

    /** 反馈帧换算成物理量；量程无效时返回 false。 */
    [[nodiscard]] static bool decode_feedback(const Frame &frame, const Limits &limits, Feedback &feedback) noexcept;

    /**
     * @brief 寄存器应答：D[0..1] 等于 esc_id、D[2] 为操作码，读写为 8 字节、存储为 4 字节（D[3] = 0x01）。
     * @note 反馈帧也可能碰巧满足这些条件（MST_ID 共用、位置高字节为 0 且低字节恰好是 0x33/0x55 时），
     *       调用方应在有未完成的请求时才按应答处理，并核对寄存器地址。
     */
    [[nodiscard]] static bool
    decode_register_reply(const Frame &frame, std::uint16_t esc_id, RegisterReply &reply) noexcept;

    /** 把应答的 4 字节数据按 float 解释。 */
    [[nodiscard]] static float value_as_float(std::uint32_t value) noexcept;
};

} // namespace protocol

#endif // DRIVER_PROTOCOL_DM_DM_ESC_HPP
