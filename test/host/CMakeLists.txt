# 宿主测试（P0 独立工程）
#
# 在宿主机上编译 legacy/ 中的被测源码，用 ctest 运行。P0 还没有根 CMakeLists，
# 所以这里是独立工程；P1 建根工程后改由根工程引入，被测源码路径随 P2 搬迁更新。
#
#   cmake -S test/host -B build/host-tests -G Ninja
#   cmake --build build/host-tests
#   ctest --test-dir build/host-tests --output-on-failure
#
# 测试来源：ControllerCode 提交 fd1edbb 的 Tests/，适配记录见 README.md。
cmake_minimum_required(VERSION 3.22)
project(foray_mcu_host_tests LANGUAGES CXX)

if(NOT CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    message(FATAL_ERROR "宿主测试需要 GCC 或 Clang（当前为 ${CMAKE_CXX_COMPILER_ID}）")
endif()

# 与固件工程一致：C++17，不开 GNU 扩展。
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

enable_testing()
find_package(Threads REQUIRED)

get_filename_component(LEGACY_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../legacy" ABSOLUTE)
set(SUPPORT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/support")

# ---- 被测源码（显式列出；P2 搬迁时同步更新路径） ----
set(DJI_PROTOCOL_SOURCES
    "${LEGACY_ROOT}/Libraries/Protocol/dji/DjiEsc.cpp")
set(DJI_MOTOR_SOURCES
    "${LEGACY_ROOT}/Libraries/Device/motor/dji/DjiMotor.cpp"
    ${DJI_PROTOCOL_SOURCES})
set(DJI_ADAPTER_SOURCES
    "${LEGACY_ROOT}/Libraries/Device/motor/dji/DjiMotorAdapter.cpp"
    ${DJI_MOTOR_SOURCES})
set(CHASSIS_CONTROL_SOURCES
    "${LEGACY_ROOT}/Application/Task/1_chassis/ChassisController.cpp"
    "${LEGACY_ROOT}/Libraries/Algorithm/alg_controller/Pid.cpp")
set(CHASSIS_TASK_SOURCES
    "${LEGACY_ROOT}/Application/Task/1_chassis/ChassisTask.cpp")
set(BMI088_SOURCES
    "${LEGACY_ROOT}/Libraries/Device/bmi088/Bmi088.cpp"
    "${LEGACY_ROOT}/Libraries/Device/bmi088/Bmi088Accel.cpp"
    "${LEGACY_ROOT}/Libraries/Device/bmi088/Bmi088Gyro.cpp"
    "${LEGACY_ROOT}/Libraries/Device/bmi088/Bmi088Temperature.cpp")

# foray_host_test(<名称> [MOCK_BACKEND] [NO_TEST] SOURCES <源文件>...)
#   MOCK_BACKEND：额外链接模拟电机适配器与生产 ChassisTask，并把 support/shadow
#                 放在 include 路径最前，用同名影子头拦截生产的 DJI 映射头与
#                 cmsis_os2.h（生产任务代码不改、不加测试开关）。
#   NO_TEST：只建可执行文件，由调用方自行登记测试（用于带参数的场景测试）。
function(foray_host_test name)
    cmake_parse_arguments(PARSE_ARGV 1 ARG "MOCK_BACKEND;NO_TEST" "" "SOURCES")
    add_executable(${name} ${ARG_SOURCES})
    target_compile_options(${name} PRIVATE -Wall -Wextra -Werror -UNDEBUG)
    target_include_directories(${name} PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}"
        "${LEGACY_ROOT}")
    if(ARG_MOCK_BACKEND)
        target_sources(${name} PRIVATE "${SUPPORT_DIR}/MockMotor.cpp" ${CHASSIS_TASK_SOURCES})
        target_include_directories(${name} BEFORE PRIVATE "${SUPPORT_DIR}/shadow")
    endif()
    if(NOT ARG_NO_TEST)
        add_test(NAME ${name} COMMAND ${name})
    endif()
endfunction()

# ---- 单元测试 ----
foray_host_test(dji_protocol SOURCES
    unit/protocol/DjiEscTest.cpp ${DJI_PROTOCOL_SOURCES})
foray_host_test(dji_motor SOURCES
    unit/motor/DjiMotorTest.cpp ${DJI_MOTOR_SOURCES})
foray_host_test(dji_adapter SOURCES
    unit/motor/DjiMotorAdapterTest.cpp ${DJI_ADAPTER_SOURCES})
foray_host_test(angle_wrap SOURCES
    unit/motor/AngleWrapTest.cpp ${DJI_MOTOR_SOURCES})
foray_host_test(motor_group SOURCES
    unit/motor/MotorGroupTest.cpp ${DJI_ADAPTER_SOURCES})
foray_host_test(brand_extension SOURCES
    unit/motor/BrandExtensionTest.cpp)
foray_host_test(bmi088_acquisition SOURCES
    unit/imu_acquisition/bmi088_acquisition_test.cpp ${BMI088_SOURCES})
# 含并发生产者场景，沿用历史的 -O2。
target_compile_options(bmi088_acquisition PRIVATE -O2)
target_link_libraries(bmi088_acquisition PRIVATE Threads::Threads)

# ---- 集成测试（底盘任务 + 控制器 + 模拟电机） ----
foray_host_test(chassis_controller MOCK_BACKEND SOURCES
    integration/chassis/ChassisControllerTest.cpp
    "${SUPPORT_DIR}/TaskRuntimeStub.cpp"
    ${CHASSIS_CONTROL_SOURCES})
foray_host_test(controller_equivalence MOCK_BACKEND SOURCES
    integration/chassis/ControllerEquivalenceTest.cpp
    "${SUPPORT_DIR}/TaskRuntimeStub.cpp"
    ${CHASSIS_CONTROL_SOURCES})
foray_host_test(chassis_mock MOCK_BACKEND NO_TEST SOURCES
    integration/chassis/ChassisMockTest.cpp
    ${CHASSIS_CONTROL_SOURCES})
foreach(scenario IN ITEMS normal read_failure command_failure init_failure)
    add_test(NAME mock_${scenario} COMMAND chassis_mock ${scenario})
endforeach()

# ---- 架构断言：任务层与通用电机头不得引用具体电机协议、适配器或驱动 ----
# 任务允许声明通用型号、电机组与品牌映射包含（装配选择）。清单显式列出，
# 任一文件缺失即报错，提醒搬迁时同步更新；文件改动后会自动重新配置。
set(MOTOR_BOUNDARY_FILES
    "${LEGACY_ROOT}/Application/Task/1_chassis/ChassisController.hpp"
    "${LEGACY_ROOT}/Application/Task/1_chassis/ChassisController.cpp"
    "${LEGACY_ROOT}/Application/Task/1_chassis/ChassisTask.hpp"
    "${LEGACY_ROOT}/Application/Task/1_chassis/ChassisTask.cpp"
    "${LEGACY_ROOT}/Libraries/Device/motor/Motor.hpp"
    "${LEGACY_ROOT}/Libraries/Device/motor/MotorGroup.hpp")
foreach(source IN LISTS MOTOR_BOUNDARY_FILES)
    if(NOT EXISTS "${source}")
        message(FATAL_ERROR "边界检查清单中的文件不存在：${source}")
    endif()
    file(READ "${source}" body)
    if(body MATCHES "DjiEsc|DjiMotorAdapter|DjiMotorBackend|DjiMotor\\.hpp|DjiMotor\\.cpp")
        message(FATAL_ERROR "具体电机实现泄漏到 ${source}")
    endif()
endforeach()
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS ${MOTOR_BOUNDARY_FILES})

# ---- 编译期断言：电机数量与型号在编译期校验 ----
# 预期编译失败是验收内容，不是工程错误。每次配置都重新检查，不沿用缓存结果。
include(CheckCXXSourceCompiles)
set(CMAKE_REQUIRED_INCLUDES "${LEGACY_ROOT}")

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

set(DJI_MAPPING_INCLUDE "#include <Libraries/Device/motor/dji/DjiMotorModelMap.hpp>")
set(MOTOR_GROUP_INCLUDE "#include <Libraries/Device/motor/MotorGroup.hpp>")

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
# 未映射型号：即使已包含 DJI 映射，255 仍无特化，必须在编译期拒绝。
check_motor_group(UNKNOWN_MODEL_COMPILES FALSE "static_cast<device::MotorModel>(255)" 1)
# 未包含任何品牌映射时，合法型号同样无法实例化（映射是唯一入口）。
check_motor_code(MAPPING_REQUIRED_COMPILES FALSE
    "${MOTOR_GROUP_INCLUDE}
static_assert(sizeof(device::MotorGroup<device::MotorModel::M2006, 4>) > 0);
int main() {}")
