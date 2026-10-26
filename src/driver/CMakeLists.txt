# driver：设备驱动与设备协议（评审报告 §5.7）。可以依赖 hal、lib 与驱动内的协议；不得依赖 msg、OS、app。
# 迁移期源码在 legacy/Libraries/{Device,Protocol}，P2 分别搬到本目录与 driver/protocol/。
set(device_dir "${LEGACY_DIR}/Libraries/Device")
set(protocol_dir "${LEGACY_DIR}/Libraries/Protocol")

legacy_library(driver_protocol_dji
    "${protocol_dir}/dji/DjiEsc.cpp"
    "${protocol_dir}/dji/DjiEsc.hpp")

legacy_library(driver_protocol_sbus
    "${protocol_dir}/sbus/Sbus.cpp"
    "${protocol_dir}/sbus/Sbus.hpp")

# 电机的通用接口与编译期装配（不含任何具体品牌）。
legacy_interface(driver_motor
    "${device_dir}/motor/Motor.hpp"
    "${device_dir}/motor/MotorGroup.hpp")
target_link_libraries(driver_motor INTERFACE hal)

# DJI 电机：驱动、适配器与型号映射（型号固有参数属驱动层，Q11、例外 X-05）。
legacy_library(driver_motor_dji
    "${device_dir}/motor/dji/DjiMotor.cpp"
    "${device_dir}/motor/dji/DjiMotor.hpp"
    "${device_dir}/motor/dji/DjiMotorAdapter.cpp"
    "${device_dir}/motor/dji/DjiMotorAdapter.hpp"
    "${device_dir}/motor/dji/DjiMotorModelMap.hpp")
target_link_libraries(driver_motor_dji PUBLIC driver_motor driver_protocol_dji hal)

legacy_library(driver_bmi088
    "${device_dir}/bmi088/Bmi088.cpp"
    "${device_dir}/bmi088/Bmi088.hpp"
    "${device_dir}/bmi088/Bmi088Accel.cpp"
    "${device_dir}/bmi088/Bmi088Accel.hpp"
    "${device_dir}/bmi088/Bmi088Gyro.cpp"
    "${device_dir}/bmi088/Bmi088Gyro.hpp"
    "${device_dir}/bmi088/Bmi088Heater.cpp"
    "${device_dir}/bmi088/Bmi088Heater.hpp"
    "${device_dir}/bmi088/Bmi088SampleQueue.hpp"
    "${device_dir}/bmi088/Bmi088Temperature.cpp"
    "${device_dir}/bmi088/Bmi088Temperature.hpp"
    "${device_dir}/bmi088/Bmi088Transfer.hpp")
target_link_libraries(driver_bmi088 PUBLIC hal lib_control)

legacy_library(driver_buzzer
    "${device_dir}/buzzer/Buzzer.cpp"
    "${device_dir}/buzzer/Buzzer.hpp")
target_link_libraries(driver_buzzer PUBLIC hal)

legacy_library(driver_ws2812
    "${device_dir}/ws2812/Ws2812.cpp"
    "${device_dir}/ws2812/Ws2812.hpp")
target_link_libraries(driver_ws2812 PUBLIC hal)
