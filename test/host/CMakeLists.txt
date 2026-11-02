# 宿主测试：由根工程在宿主构建中引入（cmake --preset host-tests），用 ctest 运行。
#
#   cmake --preset host-tests
#   cmake --build --preset host-tests
#   ctest --preset host-tests
#
# 单元测试直接链接被测 target（src/ 的框架库、app/ 的能力库）与 platform_mock（Decision-20261031-13）。
# 测试来源：ControllerCode 提交 fd1edbb 的 Tests/ 与 F1 起新写的测试，适配记录见 README.md。
find_package(Threads REQUIRED)

# host_test(<名称> [NO_TEST] SOURCES <源文件>... [LINK <target>...])
#   可执行文件为 test_<名称>，ctest 名称为 <名称>。
#   LINK：链接的被测 target（框架库、能力库、platform_mock）。
#   NO_TEST：只建可执行文件，由调用方自行登记测试（用于带参数的场景测试）。
function(host_test name)
    cmake_parse_arguments(PARSE_ARGV 1 ARG "NO_TEST" "" "SOURCES;LINK")
    set(target test_${name})
    add_executable(${target} ${ARG_SOURCES})
    enable_strict_warnings(${target})
    target_compile_options(${target} PRIVATE -UNDEBUG)
    # 测试以 test/host 为根引用支持文件（"support/..."）；被测代码的头文件随链接的 target 而来。
    target_include_directories(${target} PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}")
    target_link_libraries(${target} PRIVATE ${ARG_LINK})
    if(NOT ARG_NO_TEST)
        add_test(NAME ${name} COMMAND ${target})
    endif()
endfunction()

# ---- 单元测试（框架） ----
host_test(dji_protocol
    SOURCES unit/protocol/dji_esc_test.cpp
    LINK driver_protocol_dji)
host_test(dm_protocol
    SOURCES unit/protocol/dm_esc_test.cpp
    LINK driver_protocol_dm)
# DJI 电机：总线收发与时间经 platform_mock 的 hal::Can、hal_os 替身（Decision-20261101-01）。
host_test(dji_motor
    SOURCES unit/motor/dji_motor_test.cpp
    LINK driver_motor_dji platform_mock)
host_test(dji_adapter
    SOURCES unit/motor/dji_motor_adapter_test.cpp
    LINK driver_motor_dji platform_mock)
host_test(angle_wrap
    SOURCES unit/motor/angle_wrap_test.cpp
    LINK driver_motor_dji platform_mock)
host_test(motor_group
    SOURCES unit/motor/motor_group_test.cpp
    LINK driver_motor_dji platform_mock)
# 达妙电机：启动回读核对、使能、反馈与故障，电机由测试按说明书的帧格式扮演（Decision-20261102-01）。
host_test(dm_motor
    SOURCES unit/motor/dm_motor_test.cpp
    LINK driver_motor_dm platform_mock)
host_test(dm_adapter
    SOURCES unit/motor/dm_motor_adapter_test.cpp
    LINK driver_motor_dm platform_mock)
host_test(brand_extension
    SOURCES unit/motor/brand_extension_test.cpp
    LINK driver_motor)
# 旧 app 的数据就绪邮箱与诊断采集两段随旧 app 归档删除（Decision-20261031-13），只测驱动本身。沿用历史的 -O2。
host_test(bmi088_acquisition
    SOURCES unit/imu_acquisition/bmi088_acquisition_test.cpp
    LINK driver_bmi088 platform_mock)
target_compile_options(test_bmi088_acquisition PRIVATE -O2)
# hal 契约 v1 在 platform/mock 上的语义与改用它的驱动（Decision-20261101-01）。
host_test(hal_gpio_pwm
    SOURCES unit/hal/gpio_pwm_test.cpp
    LINK platform_mock)
host_test(hal_can
    SOURCES unit/hal/can_test.cpp
    LINK platform_mock)
host_test(hal_spi
    SOURCES unit/hal/spi_test.cpp
    LINK platform_mock)
host_test(hal_uart
    SOURCES unit/hal/uart_test.cpp
    LINK platform_mock)
host_test(hal_usb_cdc
    SOURCES unit/hal/usb_cdc_test.cpp
    LINK platform_mock)
host_test(ws2812
    SOURCES unit/ws2812/ws2812_test.cpp
    LINK driver_ws2812 platform_mock)
host_test(buzzer
    SOURCES unit/buzzer/buzzer_test.cpp
    LINK driver_buzzer platform_mock)
host_test(hal_os_mock
    SOURCES unit/hal_os/mock_os_test.cpp
    LINK platform_mock)
host_test(signal
    SOURCES unit/hal_os/signal_test.cpp
    LINK platform_mock)
# 误用检查：每个场景都应打印 mock 的报错并 abort()，按输出判定（PASS_REGULAR_EXPRESSION 不看退出状态）。
host_test(signal_misuse NO_TEST
    SOURCES unit/hal_os/signal_misuse_test.cpp
    LINK platform_mock)
set(SIGNAL_MISUSE_SCENARIOS
    "notify_in_critical|notify\\(\\) 不能在 CriticalSection 内调用"
    "wait_in_critical|wait\\(\\) 不能在 CriticalSection 内调用"
    "two_waiters|两个线程同时等同一个 Signal")
foreach(entry IN LISTS SIGNAL_MISUSE_SCENARIOS)
    string(FIND "${entry}" "|" split)
    string(SUBSTRING "${entry}" 0 ${split} scenario)
    math(EXPR split "${split} + 1")
    string(SUBSTRING "${entry}" ${split} -1 message)
    add_test(NAME signal_misuse_${scenario} COMMAND test_signal_misuse ${scenario})
    set_tests_properties(signal_misuse_${scenario} PROPERTIES PASS_REGULAR_EXPRESSION "platform_mock: .*${message}")
endforeach()
host_test(heater_control
    SOURCES unit/control/heater_control_test.cpp
    LINK lib_control)
host_test(axis_mapping
    SOURCES unit/estimate/axis_mapping_test.cpp
    LINK lib_estimate)
host_test(crc16
    SOURCES unit/crc/crc16_test.cpp
    LINK utils_crc)
# 并发原语：含多线程压力与计数器回绕，用 -O2（编译器重排更激进，回绕测试也快得多）。
host_test(spsc_queue
    SOURCES unit/concurrency/spsc_queue_test.cpp
    LINK utils_concurrency Threads::Threads)
host_test(latest_value
    SOURCES unit/concurrency/latest_value_test.cpp
    LINK utils_concurrency Threads::Threads)
target_compile_options(test_spsc_queue PRIVATE -O2)
target_compile_options(test_latest_value PRIVATE -O2)
# topic：含一个发布线程对三个订阅线程的压力测试，同样用 -O2。
host_test(topic
    SOURCES unit/topic/topic_test.cpp
    LINK middleware_topic platform_mock Threads::Threads)
target_compile_options(test_topic PRIVATE -O2)
# bringup 能力（Decision-20261031-12）：直接链接 app_bringup（Decision-20261031-13）。
host_test(bringup
    SOURCES unit/bringup/bringup_test.cpp
    LINK app_bringup platform_mock)
# bringup 的外设检查（Decision-20261101-03）：BMI088 由 SPI 替身扮演，中断由 GPIO 替身触发
host_test(bringup_imu
    SOURCES unit/bringup/imu_check_test.cpp
    LINK app_bringup platform_mock)
host_test(bringup_io
    SOURCES unit/bringup/io_checks_test.cpp
    LINK app_bringup platform_mock)
# 达妙电机检查（Decision-20261102-02）：电机由 unit/motor/dm_fake_motor.hpp 扮演
host_test(bringup_dm
    SOURCES unit/bringup/dm_check_test.cpp
    LINK app_bringup platform_mock)

# ---- 架构断言：通用电机头不得引用具体电机协议、适配器或驱动 ----
# 通用头允许声明通用型号、电机组与品牌映射包含（装配选择）。清单显式列出，
# 任一文件缺失即报错，提醒搬迁时同步更新；文件改动后会自动重新配置。
# 旧 app 的底盘控制器与任务随旧 app 归档（Decision-20261031-13），阶段 A 重建底盘能力时把它的文件加回来。
set(MOTOR_BOUNDARY_FILES
    "${PROJECT_SOURCE_DIR}/src/driver/motor/motor.hpp"
    "${PROJECT_SOURCE_DIR}/src/driver/motor/motor_group.hpp")
foreach(source IN LISTS MOTOR_BOUNDARY_FILES)
    if(NOT EXISTS "${source}")
        message(FATAL_ERROR "边界检查清单中的文件不存在：${source}")
    endif()
    file(READ "${source}" body)
    # 类名与（P2 改名后的）文件名都要拦：DjiEsc、DjiMotorAdapter、dji_esc.hpp、dji_motor.hpp、达妙的 DmEsc、DmMotor 等。
    if(body MATCHES "DjiEsc|DjiMotorAdapter|DjiMotorBackend|dji_esc|dji_motor\\.(hpp|cpp)|dji_motor_adapter|DmEsc|DmMotor|dm_esc|dm_motor")
        message(FATAL_ERROR "具体电机实现泄漏到 ${source}")
    endif()
endforeach()
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS ${MOTOR_BOUNDARY_FILES})

# ---- 编译期断言：电机数量与型号在编译期校验 ----
# 预期编译失败是验收内容，不是工程错误。每次配置都重新检查，不沿用缓存结果。
include(CheckCXXSourceCompiles)
set(CMAKE_REQUIRED_INCLUDES "${PROJECT_SOURCE_DIR}/src")

# check_motor_code(<结果变量> <应能编译：TRUE|FALSE> <源码>)
function(check_motor_code result should_compile code)
    unset(${result} CACHE)
    check_cxx_source_compiles("${code}" ${result})
    if(should_compile AND NOT ${result})
        message(FATAL_ERROR "应能编译却失败：${result}")
    elseif(NOT should_compile AND ${result})
        message(FATAL_ERROR "应被编译期拒绝却通过：${result}")
    endif()
endfunction()

set(DJI_MAPPING_INCLUDE "#include <driver/motor/dji/dji_motor_model_map.hpp>")
set(DM_MAPPING_INCLUDE "#include <driver/motor/dm/dm_motor_model_map.hpp>")
set(MOTOR_GROUP_INCLUDE "#include <driver/motor/motor_group.hpp>")

# 对照：头文件本身可编译，保证下面"应被拒绝"的检查不是因为无关错误而失败。
check_motor_code(MOTOR_GROUP_HEADERS_COMPILE TRUE
    "${DJI_MAPPING_INCLUDE}\n${MOTOR_GROUP_INCLUDE}\nint main() {}")
check_motor_code(MOTOR_GROUP_HEADER_ALONE_COMPILES TRUE
    "${MOTOR_GROUP_INCLUDE}\nint main() {}")

function(check_motor_group result should_compile model count)
    check_motor_code(${result} ${should_compile}
        "${DJI_MAPPING_INCLUDE}\n${MOTOR_GROUP_INCLUDE}
static_assert(sizeof(device::MotorGroup<${model}, ${count}>) > 0);
int main() {}")
endfunction()
# 每个被拒绝的型号都配一个能编译的对照，保证"被拒绝"来自数量检查，而不是型号名写错。
check_motor_group(MOTOR_GROUP_VALID TRUE device::MotorModel::M2006 4)
check_motor_group(MOTOR_ZERO_COMPILES FALSE device::MotorModel::M2006 0)
check_motor_group(MOTOR_NINE_COMPILES FALSE device::MotorModel::M2006 9)
check_motor_group(GIMBAL_GROUP_VALID TRUE device::MotorModel::GM6020_CURRENT 4)
check_motor_group(GIMBAL_EIGHT_COMPILES FALSE device::MotorModel::GM6020_CURRENT 8)
# 达妙：一条总线最多 4 台；只包含 DJI 映射时达妙型号无特化。
function(check_dm_group result should_compile count)
    check_motor_code(${result} ${should_compile}
        "${DM_MAPPING_INCLUDE}\n${MOTOR_GROUP_INCLUDE}
static_assert(sizeof(device::MotorGroup<device::MotorModel::DM_J4310, ${count}>) > 0);
int main() {}")
endfunction()
check_dm_group(DM_GROUP_VALID TRUE 4)
check_dm_group(DM_FIVE_COMPILES FALSE 5)
check_motor_group(DM_WITHOUT_MAPPING_COMPILES FALSE device::MotorModel::DM_J4310 1)
# 未映射型号：即使已包含 DJI 映射，255 仍无特化，必须在编译期拒绝。
check_motor_group(UNKNOWN_MODEL_COMPILES FALSE "static_cast<device::MotorModel>(255)" 1)
# 未包含任何品牌映射时，合法型号同样无法实例化（映射是唯一入口）。
check_motor_code(MAPPING_REQUIRED_COMPILES FALSE
    "${MOTOR_GROUP_INCLUDE}
static_assert(sizeof(device::MotorGroup<device::MotorModel::M2006, 4>) > 0);
int main() {}")

# ---- 编译期断言：topic 的消息约束（规则 17、Decision-20261031-05） ----
# 消息必须可平凡拷贝，且不超过 MAX_MESSAGE_SIZE 字节；对照检查保证"被拒绝"来自这两条约束。
set(TOPIC_INCLUDE "#include <middleware/topic/topic.hpp>\n#include <array>\n#include <cstdint>")
function(check_topic_message result should_compile message_type)
    check_motor_code(${result} ${should_compile}
        "${TOPIC_INCLUDE}
${message_type}
topic::Topic<Message> g_topic{\"check\"};
int main() {}")
endfunction()
check_topic_message(TOPIC_MAX_SIZE_COMPILES TRUE
    "struct Message { std::array<std::uint8_t, topic::MAX_MESSAGE_SIZE> bytes; };")
check_topic_message(TOPIC_OVERSIZED_COMPILES FALSE
    "struct Message { std::array<std::uint8_t, topic::MAX_MESSAGE_SIZE + 1U> bytes; };")
check_topic_message(TOPIC_NON_TRIVIAL_COMPILES FALSE
    "struct Message { Message() = default; Message(const Message &) {} std::uint32_t value; };")
