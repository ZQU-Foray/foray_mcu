# 宿主测试：由根工程在宿主构建中引入（cmake --preset host-tests），用 ctest 运行。
#
#   cmake --preset host-tests
#   cmake --build --preset host-tests
#   ctest --preset host-tests
#
# 测框架代码的单元测试直接链接框架 target（src/ 的 lib_*、driver_*）与 platform_mock；
# 测产品代码（app）的集成测试要用影子头替换 DJI 映射与 cmsis_os2.h，仍在测试里直接编译 app 源码，
# 路径随 P2/P2.5 搬迁更新（P3b 建立能力 target 的测试接缝后改为链接）。
# 测试来源：ControllerCode 提交 fd1edbb 的 Tests/，适配记录见 README.md。
find_package(Threads REQUIRED)

set(SUPPORT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/support")

# ---- 在测试里直接编译的产品代码（例外 X-10，P3b 改为链接能力 target） ----
set(CHASSIS_CONTROLLER_SOURCES
    "${PROJECT_SOURCE_DIR}/app/chassis/chassis_controller.cpp")
set(CHASSIS_TASK_SOURCES
    "${PROJECT_SOURCE_DIR}/app/tasks/chassis_task.cpp")

# host_test(<名称> [MOCK_BACKEND] [NO_TEST] SOURCES <源文件>... [LINK <target>...])
#   可执行文件为 test_<名称>，ctest 名称为 <名称>。
#   LINK：链接的被测 target（框架库、platform_mock）。
#   MOCK_BACKEND：额外编译模拟电机适配器与生产 ChassisTask，并把 support/shadow
#                 放在 include 路径最前，用同名影子头拦截生产的 DJI 映射头与
#                 cmsis_os2.h（生产任务代码不改、不加测试开关）。
#   NO_TEST：只建可执行文件，由调用方自行登记测试（用于带参数的场景测试）。
function(host_test name)
    cmake_parse_arguments(PARSE_ARGV 1 ARG "MOCK_BACKEND;NO_TEST" "" "SOURCES;LINK")
    set(target test_${name})
    add_executable(${target} ${ARG_SOURCES})
    enable_strict_warnings(${target})
    target_compile_options(${target} PRIVATE -UNDEBUG)
    # 测试以 test/host 为根引用支持文件（"support/..."），以 app/ 为根引用产品头文件（"tasks/..."）；
    # legacy_include 给仍在 legacy/ 的 Application/Imu（P2.5 搬）。
    target_include_directories(${target} PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}" "${PROJECT_SOURCE_DIR}/app")
    target_link_libraries(${target} PRIVATE ${ARG_LINK} legacy_include)
    if(ARG_MOCK_BACKEND)
        target_sources(${target} PRIVATE "${SUPPORT_DIR}/mock_motor.cpp" ${CHASSIS_TASK_SOURCES})
        target_include_directories(${target} BEFORE PRIVATE "${SUPPORT_DIR}/shadow")
        target_link_libraries(${target} PRIVATE driver_motor)
    endif()
    if(NOT ARG_NO_TEST)
        add_test(NAME ${name} COMMAND ${target})
    endif()
endfunction()

# ---- 单元测试（框架） ----
host_test(dji_protocol
    SOURCES unit/protocol/dji_esc_test.cpp
    LINK driver_protocol_dji)
host_test(dji_motor
    SOURCES unit/motor/dji_motor_test.cpp
    LINK driver_motor_dji)
host_test(dji_adapter
    SOURCES unit/motor/dji_motor_adapter_test.cpp
    LINK driver_motor_dji)
host_test(angle_wrap
    SOURCES unit/motor/angle_wrap_test.cpp
    LINK driver_motor_dji)
host_test(motor_group
    SOURCES unit/motor/motor_group_test.cpp
    LINK driver_motor_dji)
host_test(brand_extension
    SOURCES unit/motor/brand_extension_test.cpp
    LINK driver_motor)
# 测试直接 #include 了 app 的 ImuDiagnostics.cpp（访问其内部状态），因此也看得到 app 头文件。
host_test(bmi088_acquisition
    SOURCES unit/imu_acquisition/bmi088_acquisition_test.cpp
    LINK driver_bmi088)
# 含并发生产者场景，沿用历史的 -O2。
target_compile_options(test_bmi088_acquisition PRIVATE -O2)
target_link_libraries(test_bmi088_acquisition PRIVATE Threads::Threads)
host_test(hal_os_mock
    SOURCES unit/hal_os/mock_os_test.cpp
    LINK platform_mock)

# ---- 集成测试（底盘任务 + 控制器 + 模拟电机） ----
host_test(chassis_controller MOCK_BACKEND
    SOURCES integration/chassis/chassis_controller_test.cpp
            "${SUPPORT_DIR}/task_runtime_stub.cpp"
            ${CHASSIS_CONTROLLER_SOURCES}
    LINK lib_control)
host_test(controller_equivalence MOCK_BACKEND
    SOURCES integration/chassis/controller_equivalence_test.cpp
            "${SUPPORT_DIR}/task_runtime_stub.cpp"
            ${CHASSIS_CONTROLLER_SOURCES}
    LINK lib_control)
host_test(chassis_mock MOCK_BACKEND NO_TEST
    SOURCES integration/chassis/chassis_mock_test.cpp
            ${CHASSIS_CONTROLLER_SOURCES}
    LINK lib_control)
foreach(scenario IN ITEMS normal read_failure command_failure init_failure)
    add_test(NAME mock_${scenario} COMMAND test_chassis_mock ${scenario})
endforeach()

# ---- 架构断言：任务层与通用电机头不得引用具体电机协议、适配器或驱动 ----
# 任务允许声明通用型号、电机组与品牌映射包含（装配选择）。清单显式列出，
# 任一文件缺失即报错，提醒搬迁时同步更新；文件改动后会自动重新配置。
set(MOTOR_BOUNDARY_FILES
    "${PROJECT_SOURCE_DIR}/app/chassis/chassis_controller.hpp"
    "${PROJECT_SOURCE_DIR}/app/chassis/chassis_controller.cpp"
    "${PROJECT_SOURCE_DIR}/app/tasks/chassis_task.hpp"
    "${PROJECT_SOURCE_DIR}/app/tasks/chassis_task.cpp"
    "${PROJECT_SOURCE_DIR}/src/driver/motor/motor.hpp"
    "${PROJECT_SOURCE_DIR}/src/driver/motor/motor_group.hpp")
foreach(source IN LISTS MOTOR_BOUNDARY_FILES)
    if(NOT EXISTS "${source}")
        message(FATAL_ERROR "边界检查清单中的文件不存在：${source}")
    endif()
    file(READ "${source}" body)
    # 类名与（P2 改名后的）文件名都要拦：DjiEsc、DjiMotorAdapter、dji_esc.hpp、dji_motor.hpp 等。
    if(body MATCHES "DjiEsc|DjiMotorAdapter|DjiMotorBackend|dji_esc|dji_motor\\.(hpp|cpp)|dji_motor_adapter")
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
# 未映射型号：即使已包含 DJI 映射，255 仍无特化，必须在编译期拒绝。
check_motor_group(UNKNOWN_MODEL_COMPILES FALSE "static_cast<device::MotorModel>(255)" 1)
# 未包含任何品牌映射时，合法型号同样无法实例化（映射是唯一入口）。
check_motor_code(MAPPING_REQUIRED_COMPILES FALSE
    "${MOTOR_GROUP_INCLUDE}
static_assert(sizeof(device::MotorGroup<device::MotorModel::M2006, 4>) > 0);
int main() {}")
