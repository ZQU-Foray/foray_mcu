# 平台无关的 Porting Kit：SPSC 环形队列 / 中断通知 / 短临界区。
# 唯一依赖是 CMSIS 内联函数（cmsis_compiler.h），包含路径由 stm32cubemx 目标提供。
add_library(portkit OBJECT PortKit.c)
target_include_directories(portkit PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(portkit PUBLIC stm32cubemx)
