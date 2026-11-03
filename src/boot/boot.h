#ifndef BOOT_BOOT_H
#define BOOT_BOOT_H

#include <stdbool.h>

// 启动入口（架构 v2 §3.3）。
// CubeMX 生成的 main.c、freertos.c 只在 USER CODE 区调用这三个函数；实现在组合根（config/assembly），
// 每个 profile 一份。生成代码是 C，所以这里是 C 头，函数为 C 链接（规则 16 的 §9.3 第 1 处）。

#ifdef __cplusplus
extern "C"
{
#endif

    /**
     * @brief 核前初始化：外设初始化之后、内核初始化之前调用（main.c 的 USER CODE 2）。
     * @return false 时生成代码进入 Error_Handler()。
     * @note 调用 hal::init()，构造对象、注入参数、取得 topic 的发布句柄；不得创建任务或调用 RTOS。
     */
    bool boot_init(void);

    /**
     * @brief 创建任务：内核初始化之后、启动之前调用（freertos.c 的 USER CODE RTOS_THREADS）。
     * @return false 时生成代码进入 Error_Handler()。
     */
    bool boot_init_rtos(void);

    /**
     * @brief 默认任务的周期工作：内核启动后由默认任务每拍调用一次（freertos.c 的 StartDefaultTask）。
     * @note 可以为空；不得长时间阻塞。
     */
    void boot_run(void);

#ifdef __cplusplus
}
#endif

#endif // BOOT_BOOT_H
